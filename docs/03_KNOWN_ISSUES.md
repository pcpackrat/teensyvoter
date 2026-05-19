# Known Issues, Technical Debt, and Status

## Open Issues

### Minor
- **Magic Numbers**: Code contains raw values for DSP coefficients and thresholds.
- **Global Variables**: Audio volume states and debug flags are global in `main.cpp` and should be encapsulated.

---

## Resolved & Closed Issues

*   **GPS Holdover Warning Discrepancy** (Fixed): The log message in `VoterClient.cpp` (Line 154) has been corrected to read `"Entering 1s holdover mode..."` to match the actual 1-second timeout logic in the code.
*   **DSP State Buffer Overflow** (Fixed): Mismatch in state buffer sizing resolved by declaring `_rssiState` with size `DSP_BLOCK_SAMPLES + 24` (160 + 24) in `DSPProcessor.h` to match the 160-sample block size. Circular buffer `VoiceFilter` handles voice filtering safely.
*   **Hardcoded WiFi Credentials** (Fixed): Credentials moved to default structures in `ConfigManager.cpp` and are configurable via CLI and Web UI.
*   **Unimplemented Fallback** (Fixed): GPS lock loss no longer streams invalid timestamps (`vtime_sec = 0`). The system enters a brief 1-second holdover and disconnects. GPS keepalive packets send standard empty location strings (`0000.00N`, `00000.00W`, `  0.0 `) during unlock, mimicking legacy PIC behavior.
