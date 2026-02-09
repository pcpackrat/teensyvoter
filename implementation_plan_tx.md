# Plan: Implement Transmit Capabilities

## Goal
Enable `TeensyVoter` to receive audio from the Voter Server (RTCM protocol) and play it out through the Teensy Audio Shield. This allows the node to function as a bi-directional radio interface (Receive and Transmit).
**CRITICAL**: The system must support **Full Duplex** operation (Simultaneous Transmit and Receive) to support Repeater operation.

## Analysis of Reference Implementation (`Voter.c`)
1.  **Protocol**: Uses UDP. Audio packets are `PAYLOAD_ULAW` (1) or `PAYLOAD_ADPCM` (3).
2.  **Jitter Buffer**: A large ring buffer (`txaudio`, ~6400 bytes) is used.
    -   Incoming packets are placed into the buffer based on their timestamp difference relative to the system time.
    -   This effectively handles out-of-order packets and jitter.
3.  **Decoding**:
    -   uLaw: Simple table lookup (`ulawtabletx`).
    -   ADPCM: Intel/DVI ADPCM algorithm.
4.  **Playback**:
    -   An ISR (`_DAC1LInterrupt`) running at 8kHz reads from the buffer.
    -   If buffer underrun/empty, it plays silence.
    -   It also handles CWID tone generation.

### Reference Implementation Analysis (`Voter2`)
- **Frequency Synchronization:** `Voter2` uses a software-defined PLL (`Packet_PLL` / `A2ddac_PLL` in `Timers.c`) to phase-lock the 8kHz sample clock to the GPS 1PPS signal. This guarantees that the *rate* of playback is identical across all nodes (0 ppm drift relative to GPS).
- **Phase (Launch) Synchronization:** `Voter2` does **not** appear to implement precise "Launch at Time T" logic in `Tx_DSP.c`. Playback starts as soon as the jitter buffer accumulates a user-defined delay (`GlobTxBufDly`).
- **Conclusion:** `Voter2` supports "Voting Client" operation and non-precision transmission. It is likely **not** compatible with strict simulcast systems that require microsecond-aligned launch times (like some PIC voter configurations), unless those systems also tolerate buffer-based alignment.
- **TeensyVoter Strategy:** We will mimic `Voter2`:
    1.  Implement Jitter Buffer with configurable delay.
    2.  Start playback when buffer threshold is reached.
    3.  **(Future - DO NOT IMPLEMENT YET)** Investigate adding strict timestamp-based launch if hardware allows (Teensy 4.1 has excellent timer capabilities).

### Reference Implementation Analysis (`PIC Voter`)
- **Simulcast Capability:** The original PIC firmware *does* implement precise simulcast logic (`SIMULCAST_ENABLE`).
- **Launch Mechanism:** It uses the GPS 1PPS signal to trigger a hardware timer (`Timer 4`). When this timer expires (after `AppConfig.LaunchDelay`), the DAC is enabled (`_T4Interrupt`). This guarantees audio starts exactly at `1PPS + Delay`.
- **Buffer Management:** It calculates a specific write index in the audio buffer based on the packet timestamp (`Voter.c` ~line 3800), ensuring audio samples are placed at the correct time offset even if packets arrive out of order.
- **Frequency Discipline:** It resets the sample timer (`TMR3`) every second on the PPS tick ("jam sync") to keep the clock roughly aligned.

### Compatibility Conclusion
- **Voter2 (STM32):** Optimizes for smooth audio/frequency stability (PLL) but lacks precise launch timing. Good for "Voting Client".
- **PIC Voter:** Optimizes for precise launch/phase alignment. Essential for "Simulcast Transmitter".
- **TeensyVoter:** To "play nice" in a mixed simulcast system, we must eventually implement the **PIC** method (Precise Launch + Timestamp Buffer). The current `Voter2`-style plan is a stepping stone.

## Proposed Architecture for TeensyVoter

### 1. `VoterClient` Enhancements (Full Duplex)
*   **Non-Blocking Loop**: The `update()` method will handle both directions in every iteration.
    *   **TX Path**: Check `recordQueue` -> Encode -> Send UDP.
    *   **RX Path**: Check `Udp.parsePacket()` -> JitterBuffer -> Decode -> `playQueue`.
*   **Jitter Buffer**: Implement a `JitterBuffer` class (or struct inside `VoterClient`) that mimics `txaudio`.
    -   Size: ~300ms to 600ms.
    -   **Concurrency**: Ensure writing to JitterBuffer (from UDP) and reading (from Audio Loop) is safe (though strictly single-threaded in `loop()` is fine if `playQueue` logic is fast).
    -   *Correction*: `playQueue` needs to be fed. We can do this in `loop()`. If `loop()` is blocked, audio underruns. We must ensure `loop()` allows simultaneous Rx/Tx.

### 2. Audio Pipeline Integration
*   **New Audio Object**: `AudioPlayQueue playQueue;`
    -   **Upsampling**: In `loop()` (or `processAudio()` helper), read 8kHz samples from `JitterBuffer`.
    -   **Interpolation**: Simple linear interpolation (160 -> ~882 samples).
    -   Push to `playQueue`.
*   **Separate Paths**:
    -   Input: `i2s_in` -> `recordQueue` (Already working).
    -   Output: `playQueue` -> `mixer1` -> `i2s_out`.
    -   These operate independently in hardware (I2S Full Duplex).

### 3. Audio Graph (`main.cpp`)
*   **Mixer Update**:
    -   `mixer1` channel 0: `playQueue` (RX Audio from Server).
    -   `mixer1` channel 1: `i2s_in` (Local Monitor - Optional, maybe mute to prevent feedback loop unless desired).
    -   `i2s_out` (Line Out) sends `mixer1` output to the Radio Transmitter.

## Implementation Steps

### Phase 1: VoterClient Receive Logic
1.  Add `processIncomingPacket()` to `VoterClient`.
2.  Implement `JitterBuffer` storage.
3.  Ensure `update()` calls both `sendAudio()` and `processIncomingPacket()`.

### Phase 2: Decoder & Upsampler
1.  Implement `uLawToPcm(byte)` and `adpcmToPcm(byte)`.
2.  Implement `upsampleAndPush()` logic.

### Phase 3: Main Integration
1.  Add `AudioPlayQueue` to `main.cpp`.
2.  Update `loop()` to feed the play queue from `VoterClient`.

### Phase 4: Server Options
1.  Handle `AppConfig.TxBufferLength` (if sent by server) or use default.

### Phase 5: Configuration & User Interface
**Goal**: Allow end-users to tune Transmit parameters via Serial and Web.

1.  `ConfigManager` Updates:
    *   Add `uint8_t txGain` (0-31, SGTL5000 Line Out Level).
    *   Add `uint16_t txBufferMs` (Jitter Buffer Latency, e.g., 100-500ms).
    *   Add `bool requestMixMinus` (Crucial for Full Duplex/Repeater to avoid echo).
2.  `SerialCLI` Updates (`main.cpp`):
    *   Update `MENU_RADIO_CONFIG` or create `MENU_TX_CONFIG`.
    *   Add commands to set Gain, Buffer, and Mix-Minus.
3.  `WebInterface` Updates:
    *   Update `radio.html` (or settings payload) to include these fields.
    *   Ensure API handles saving them.

## Future Simulcast Implementation (DO NOT IMPLEMENT YET)
**NOTE:** The following features are required for strict simulcast compatibility with PIC voters but are explicitly **OUT OF SCOPE** for the current implementation phase.
1.  **Precise Launch**: Implement hardware timer (IntervalTimer) triggered by PPS interrupt to start playback at `1PPS + Delay`.
2.  **Timestamp-Based Buffer Writing**: Modify `JitterBuffer` to write samples to specific indices based on packet timestamps, allowing for out-of-order packet correction.
