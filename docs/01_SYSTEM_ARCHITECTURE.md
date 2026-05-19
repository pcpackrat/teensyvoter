# System Architecture

## Overview
TeensyVoter is a replacement firmware for radio voting receivers. It captures audio from a radio receiver (Discriminator Audio), processes it (filtering, decimation, squelch), time-tags it precisely using GPS, and transmits it to a central voting server (Asterisk/App_Rpt/Voter) via UDP.

## Hardware Stack
- **MCU**: Teensy 4.1 (ARM Cortex-M7 @ 600MHz)
- **Audio**: SGTL5000 Audio Shield (Line In / Mic In)
- **GPS**: standard NMEA GPS with PPS (Pulse Per Second) output (connected to Pin 2)
- **Network**: 
  - Standard Ethernet (NativeEthernet) 
  - OR ESP32-S3 Co-Processor (SPI) for WiFi
- **Inputs**:
  - `RSSI_PIN` (A14): Analog voltage 0-3.3V representing signal strength.
  - `COS_PIN` (41): Digital Carrier Operated Switch input.

## Signal Flow

### 1. Audio Ingestion
- **Input**: Line In or Mic (Configurable gain).
- **Format**: I2S @ 44.1kHz, 16-bit.
- **Hardware**: SGTL5000 mixes input to Left/Right.
- **Buffer**: `AudioRecordQueue` captures blocks of 128 samples.

### 2. Signal Processing (DSP)
The 44.1kHz stream undergoes a multi-stage DSP pipeline to match the 8kHz requirement of the Voter protocol while maintaining high quality.

1. **Anti-Aliasing Filter**:
   - Hardware `AudioFilterBiquad` (2 stages, 3.6kHz cutoff, 24dB/octave) in the Teensy audio path before it enters the record queue.
   - Prevents high-frequency aliasing during downsampling.

2. **Fractional Resampling (Decimation)**:
   - Converts 44.1kHz → 8kHz.
   - Technique: Linear Interpolation with floating-point phase accumulator.
   - Solves timing drift issues caused by integer division (44100 / 6 = 7350Hz != 8000Hz).

3. **Frame Assembly**:
   - Samples accumulated into 160-sample frames (20ms length).
   - GPS Timestamp captured exactly when frame is full.

4. **Audio Filtering**:
   - **Voice Filter**: 65-tap FIR Bandpass (300Hz - 2400Hz @ 8kHz) implemented via a custom circular-buffered `VoiceFilter` class to remove CTCSS (PL) tones and out-of-band noise.
   - **De-Emphasis**: Single-pole IIR Low-Pass (Alpha 0.20) to restore FM audio balance.
   - **RSSI Calculation**: RMS measurement of High-Passed (>2.4kHz) noise content (using CMSIS-DSP `_rssiFilter` for DSP Squelch/RSSI).

5. **Encoding**:
   - Linear PCM → uLaw (G.711) compression (using bias 33).

### 3. Precise Timing (The "Voter" Standard)
- **GPS Manager**: Tracks Global Time using PPS interrupt + NMEA data.
- **Interpolation**: Microsecond-precision timestamping between PPS pulses.
- **Strict Frame Counting (Dead Reckoning)**:
  - To match the timing expectation of the Asterisk/App_Rpt voter server and prevent jitter, the transmission is anchored to the GPS time at keyup.
  - Subsequent packet timestamps are calculated continuously as `Base Time + (Frame Count * 20ms)`.
  - A configurable delay offset (default `-180ms`) is applied to position the timestamps within the server's jitter buffer window.

### 4. Networking
- **Protocol**: Cisco/Motorola Voter Protocol (UDP).
- **Security**: Challenge-Response Authentication (CRC32 digests).
- **Transport**:
  - `NetManager` abstracts underlying driver (Ethernet vs SPI/ESP32).
  - `VoterClient` handles protocol limits (keepalives, auth retries).

## Module Interaction

```mermaid
graph TD
    Hardware[Hardware: Teensy 4.1 + Audio Shield] --> AudioLib[Teensy Audio Library]
    Hardware --> GPS[GPS Module (Serial + PPS)]
    
    AudioLib -->|I2S 44.1kHz| MainLoop[Main Loop / Audio Processing]
    GPS -->|PPS Interrupt| GPSMgr[GPSManager]
    
    MainLoop -->|Resample to 8kHz| DSP[DSPProcessor]
    DSP -->|Filter & Encode| MainLoop
    
    GPSMgr -->|VTIME Timestamp| MainLoop
    
    MainLoop -->|uLaw Frame + Timestamp| Voter[VoterClient]
    
    Voter -->|UDP Packets| NetMgr[NetworkManager]
    NetMgr -->|SPI/Ethernet| Network[Network Hardware]
```
