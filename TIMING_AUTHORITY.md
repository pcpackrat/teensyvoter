# Timing Authority

The TeensyVoter firmware acts as the timing master for the system. It MUST ensure the Voting Server (Asterisk) receives a perfect stream of audio packets with monotonic timestamps, identical to the behavior of the hardware-synced Reference Implementations (VOTER/Voter2).

## 1. The Challenge (Clock Domains)

### Reference Implementations (VOTER / Voter2)
*   **Method**: Hardware Sync + Clock Discipline or Tightly Coupled Loop.
*   **Mechanism**: The audio sampling engine and the timestamping engine are locked or disciplined to the GPS PPS.
*   **Result**: 1 Second of Audio matches exactly 1 Second of GPS Time.

### TeensyVoter (Current)
*   **Method**: Hardware Sync (PPS) but Asynchronous Audio Clock.
*   **Mechanism**: 1PPS Interrupt captures strict GPS Epoch, BUT the Audio Crystal runs independently.
*   **The Problem**: The "Audio Second" (50 x 20ms frames) rarely matches the "GPS Second" exactly.
    *   If Audio is faster: You produce 50 frames before the GPS second finishes.
    *   If Audio is slower: The GPS second finishes before you prodcue 50 frames.
*   **Result**: "Strict Frame Counting" (resetting on PPS) forces a realignment every second, causing audible pops or timestamp stutter/jumps.

## 2. The Solution: Continuous Frame Counting (Free-Running)
To solve the Clock Domain Crossing issue, TeensyVoter must use **Continuous Frame Counting**.

### Rules:
1.  **Anchor Once**: Capture `BaseEpoch` only at the start of a transmission.
2.  **Count Frames**: `CurrentTime = BaseEpoch + (FramesSent * 20ms)`.
3.  **Ignore Second Boundary**: Do NOT reset `FramesSent` when the wall clock changes. This eliminates phase jumps.
4.  **Drift Guard**: Continuously compare `CurrentTime` vs `WallClock`. If drift > 2 seconds (gross error), forced resync is allowed.

This strategy guarantees the server sees a stable, monotonic stream of 20ms packets, indistinguishable from the hardware-synced reference.

## 3. Delay Offset
*   **Delay**: 180ms.
*   `FinalTimestamp = CalculatedTimestamp - Delay`.

## 4. Protocol Implementation Details (Voter2 Reference)

Based on Voter2 developer feedback:

*   **Packet Counter**:
    *   This is a **free-running counter** that increments every **20ms**.
    *   It counts up by 1 every 20ms starting from 0 at boot, regardless of whether packets are being transmitted or not.
    *   If reception pauses for 1 second, the next received packet will have a counter value approximately +50 higher than the last packet.
    *   **ADPCM Exception**: If using ADPCM (40ms transmission interval), the packet counter increments by **2** for each transmitted packet to align with the 20ms time base.

*   **Timestamps**:
    *   **Seconds**: Standard Unix time.
    *   **Nanoseconds**: 
        *   **General Purpose Packets (GPS/Auth)**: Increments by **1** for every packet sent (sequence counter).
        *   **Audio Payload Packets**: Acts as a high-resolution sub-second timestamp, incrementing by exactly **20,000,000** per 20ms packet to satisfy the server's jitter buffer requirements.
