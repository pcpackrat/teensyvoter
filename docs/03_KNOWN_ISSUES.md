# Known Issues, Technical Debt, and Status

## Open Issues

### Major
- **GPS Holdover Warning Discrepancy**: 
    - In `VoterClient.cpp` (Line 154), the serial warning logs: `"GPS Signal Lost! Entering 60s holdover mode..."`
    - However, the disconnection logic (Line 159) triggers after a timeout of `1000ms` (1 second), immediately disconnecting the client.
    - **Consequence**: The holdover is functionally 1 second, but log output suggests 60 seconds.

### Minor
- **Magic Numbers**: Code contains raw values for DSP coefficients and thresholds.
- **Global Variables**: Audio volume states and debug flags are global in `main.cpp` and should be encapsulated.

---

## Resolved & Closed Issues

*   **DSP State Buffer Overflow** (Fixed): Mismatch in state buffer sizing resolved by declaring `_rssiState` with size `DSP_BLOCK_SAMPLES + 24` (160 + 24) in `DSPProcessor.h` to match the 160-sample block size. Circular buffer `VoiceFilter` handles voice filtering safely.
*   **Hardcoded WiFi Credentials** (Fixed): Credentials moved to default structures in `ConfigManager.cpp` and are configurable via CLI and Web UI.
*   **Unimplemented Fallback** (Fixed): GPS lock loss no longer streams invalid timestamps (`vtime_sec = 0`). The system enters a brief 1-second holdover and disconnects. GPS keepalive packets send standard empty location strings (`0000.00N`, `00000.00W`, `  0.0 `) during unlock, mimicking legacy PIC behavior.
