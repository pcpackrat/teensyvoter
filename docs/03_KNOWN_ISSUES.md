# Known Issues & Technical Debt

## Critical
- **DSP State Buffer Overflow**: 
    - `DSPProcessor.h` defines `AUDIO_BLOCK_SAMPLES` as 160.
    - However, `_rssiState` and `_voiceState` arrays are hardcoded for 128 samples: `float _rssiState[128 + 24];`
    - `arm_fir_init_f32` initializes with blockSize=160.
    - **Consequence**: Memory corruption / stack overflow potential when DSP runs.
    - **Fix**: Change `128` to `AUDIO_BLOCK_SAMPLES` in array declarations.

- **Hardcoded WiFi Credentials**: `main.cpp` (Line 542) contains hardcoded credentials `("ImWatchinYou", "n0Password")`. These must be moved to `ConfigManager` before deployment.

## Major
- **Unimplemented Fallback**: `VoterClient.cpp` (Line 271) notes "Fallback or Mix Mode Logic (Unimplemented)". If GPS is lost, the system sends `vtime_sec = 0`. Server behavior in this case is undefined/variable.

## Minor
- **Magic Numbers**: Code contains raw values for DSP coefficients and thresholds.
- **Global Variables**: `g_headphoneVol`, etc. should be encapsulated.
