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
| **F08** | **Configuration** | ✅ Full | Serial CLI Menu. Persisted to EEPROM (LittleFS/EEPROM abstraction via ConfigManager). |
| **F09** | **Web Interface** | ⚠️ Skeleton | `WebInterface.cpp` exists but updates are minimal/placeholder. Dependencies on WiFi. |
| **F10** | **WiFi/ESP32 Support** | ⚠️ Partial | `EspSpiDriver` implements basic packet passing. Credentials currently hardcoded in `main.cpp`. |

## Detected Discrepancies vs Old Docs
- **Web Interface**: Documentation implies a functional web UI, but code shows it is largely a stub or minimal status page.
- **WiFi Config**: There is no CLI menu to set WiFi SSID/Password. It uses hardcoded strings in `main.cpp` ("ImWatchinYou", "n0Password").
