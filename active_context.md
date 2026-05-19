# Active Context / Project Memory

> **DO NOT DELETE**. This file represents the "Brain" of the project for AI Assistants. Read this first when starting a new session.

## Current State (May 19 2026)
- **Status**: Stable Audio Path, Ethernet Driver Integrated, Web Server Config Interface & Unified Logging Integrated.
- **Hardware**: Teensy 4.1 + SGTL5000 + Standard GPS (PPS).
- **Network**: Auto-select (Ethernet first, WiFi fallback via ESP32 SPI). Both network drivers fully support configurable parameters.
- **Core Feature**: Fractional Resampling (44.1k -> 8k) is implemented and fixes timing drift (pulsing).
- **Protocol**: Voter Protocol (Cisco/Motorola) over UDP.
- **Authentication**: Challenge/Response (CRC32) implemented and working.
- **Web Interface**: Fully implemented `TeensyWebServer` providing remote status tracking and config editing over Ethernet.
- **Logging**: Unified `Logger` module handles logging levels across components.

## Architecture Highlights
- **Audio**: `main.cpp` applies a 2-stage hardware biquad LPF (3.6kHz cutoff) to the 44.1kHz stream, resamples via linear interpolation to 8kHz, accumulates 160-sample frames, then processes them via `DSPProcessor` (custom `VoiceFilter` FIR bandpass + De-emphasis + RSSI measurement).
- **Timing**: Strict frame counting / dead reckoning timestamps anchored to GPS epoch at transmission keyup with configurable timing offset.
- **Network**: `NetManager` abstraction with dual drivers:
  - `EthernetDriver` (NativeEthernet) - tries first
  - `EspSpiDriver` (WiFi via ESP32) - automatic fallback
- **Configuration**: packed `SysConfig` struct saved directly to EEPROM.

## Immediate Next Steps (The To-Do List)
1.  **Bug Fix**: Resolve the GPS Holdover Warning log discrepancy in [VoterClient.cpp:L154](file:///C:/Users/mikec/Documents/Projects/VOTER/TeensyVoter/src/VoterClient.cpp#L154) (warning says "60s holdover" but disconnects in 1s).
2.  **Cleanup**: Encapsulate global variables in [main.cpp](file:///C:/Users/mikec/Documents/Projects/VOTER/TeensyVoter/src/main.cpp).
3.  **Testing**: Perform end-to-end integration and calibration testing of the analog RSSI input thresholds.

## Tech Stack Versions
- **Teensyduino**: 1.5x (Target Teensy 4.1)
- **Libraries**: NativeEthernet, Audio, TinyGPSPlus, CMSIS-DSP.
- **Build System**: PlatformIO (`platformio.ini`).

## Key Files
- `src/main.cpp`: Core loop, audio plumbing, resampling logic.
- `src/VoterClient.cpp`: Protocol state machine & packet formatting.
- `src/DSPProcessor.cpp`: Filter logic, CMSIS wrapper.
- `src/ConfigManager.cpp`: NVRAM/EEPROM handling.
- `src/TeensyWebServer.cpp` / `include/TeensyWebServer.h`: Web configuration and status monitoring server.
- `src/Logger.cpp` / `include/Logger.h`: Unified status logging system.
