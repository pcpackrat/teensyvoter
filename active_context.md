# Active Context / Project Memory

> **DO NOT DELETE**. This file represents the "Brain" of the project for AI Assistants. Read this first when starting a new session.

## Current State (Jan 12 2026)
- **Status**: Stable Audio Path, Critical Bug in DSP Buffer Sizing.
- **Hardware**: Teensy 4.1 + SGTL5000 + Standard GPS (PPS).
- **Core Feature**: Fractional Resampling (44.1k -> 8k) is implemented and fixes timing drift (pulsing).
- **Protocol**: Voter Protocol (Cisco/Motorola) over UDP.
- **Authentication**: Challenge/Response (CRC32) implemented and working.

## Architecture Highlights
- **Audio**: `main.cpp` accumulates 128-sample blocks, resamples to 8kHz, buffers until 160 samples, then calls `dsp.process()`.
- **Timing**: "Voter2-style" backdating used (send packet N with timestamp of N-1).
- **Network**: `NetManager` abstraction. Currently defaulting to `EspSpiDriver` (WiFi) but `NativeEthernet` is available.

## Immediate Next Steps (The To-Do List)
1.  **FIX CRITICAL BUG**: Update `DSPProcessor.h` state buffers to use `AUDIO_BLOCK_SAMPLES` size (160) instead of hardcoded 128.
2.  **Security**: Move WiFi credentials from `main.cpp` to `ConfigManager`.
3.  **Cleanup**: Encapsulate global variables in `main.cpp`.

## Tech Stack Versions
- **Teensyduino**: 1.5x (Target Teensy 4.1)
- **Libraries**: NativeEthernet, Audio, TinyGPSPlus, CMSIS-DSP.
- **Build System**: PlatformIO (`platformio.ini`).

## Key Files
- `src/main.cpp`: Core loop, audio plumbing, resampling logic.
- `src/VoterClient.cpp`: Protocol state machine & packet formatting.
- `src/DSPProcessor.cpp`: Filter logic, CMSIS wrapper.
- `src/ConfigManager.cpp`: NVRAM/EEPROM handling.
