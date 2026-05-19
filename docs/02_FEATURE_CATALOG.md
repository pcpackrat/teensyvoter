# Feature Catalog

| ID | Feature | Status | Implementation Details |
|----|---------|--------|------------------------|
| **F01** | **Radio Interface** | ✅ Full | Line In/Mic, RSSI ADC (0-3.3V), and Discrete COS Input supported. |
| **F02** | **Audio Pipeline** | ✅ Full | 44.1kHz I2S → Anti-Alias → Resample (8kHz) → PL Filter → De-Emp → uLaw. |
| **F03** | **DSP Squelch** | ✅ Full | Noise-based squelch using RMS of high-frequency content (>2.4kHz). Configurable threshold. |
| **F04** | **Hardware Squelch** | ✅ Full | Uses 'COS_PIN' logic optional. Mapped to 'Active' logic in Voter protocol. |
| **F05** | **GPS Timing** | ✅ Full | Microsecond precision via PPS. NMEA parsing. Epoch tracking. Jitter correction. |
| **F06** | **Voter Protocol** | ✅ Full | Authentication (Challenge/Response), Audio Frames (Type 0), Keepalives, Legacy GPS Packets. |
| **F07** | **Fractional Resampling** | ✅ Full | Linear Interpolator fixes 44.1k/8k drift issues. Includes anti-aliasing. |
| **F08** | **Configuration** | ✅ Full | Serial CLI Menu. Persisted to EEPROM via ConfigManager. |
| **F09** | **Web Interface** | ✅ Full | Web server implemented in `TeensyWebServer.cpp` serving HTML configuration pages over Native Ethernet. |
| **F10** | **WiFi/ESP32 Support** | ✅ Full | `EspSpiDriver` provides SPI-to-WiFi bridging. Credentials are configurable via Serial CLI or Web Interface and stored in EEPROM. |

## Feature Status Updates
- **Web Interface**: A fully functional HTTP web server has been implemented, allowing remote status monitoring and editing configuration settings over the native Ethernet port.
- **WiFi Config**: SSID and Password settings are now stored in the `SysConfig` EEPROM structure and can be modified through the Network CLI menu or Web UI instead of being hardcoded in `main.cpp`.
