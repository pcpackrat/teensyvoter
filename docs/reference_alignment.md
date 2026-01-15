# Protocol & Audio Reference Alignment

**Objective**: Eliminate "wild tangents" by strictly adhering to the proven logic of the original `VOTER` (PIC) and `VOTER2` (STM32) projects.

## 1. Reference Sources
*   **VOTER (Original)**: `C:\Users\mikec\Documents\Projects\VOTER\Voter-master`
    *   *Controller*: Microchip PIC (dsp33f?).
    *   *Key Files*: `voter.c`? `Base_Voter.X`?
*   **VOTER2 (STM32)**: `C:\Users\mikec\Documents\Projects\VOTER\Voter2`
    *   *Controller*: STM32F4.
    *   *Key Files*: `Rx_DSP.c`, `network.c`?

## 2. Audio Pipeline Comparison

| Feature | TeensyVoter (Current) | VOTER (Original) | VOTER2 (STM32) | Alignment Action |
| :--- | :--- | :--- | :--- | :--- |
| **Sample Rate** | 44.1kHz -> 8kHz (Simple Avg) | 8kHz (Native ADC) | 8kHz (Native ADC) | **Improve Decimation.** |
| **Frame Size** | 160 samples (20ms) | 160 samples | 160 samples | **Aligned.** |
| **Filtering** | Simple LPF (1-pole) | FFT-based? | **FIR Bandpass (300-2400Hz)** | **CRITICAL MISSING LINK.** |
| **Encoding** | uLaw | uLaw/ADPCM | uLaw | Aligned. |

## 3. Investigation Findings
### A. The "Buzz" Source
The "Buzz" is almost certainly **aliasing** or **out-of-band noise** failing to be filtered.
*   **Voter2** uses a 65-tap FIR Bandpass Filter (300Hz - 2400Hz) on the 8kHz stream.
*   **TeensyVoter** uses a simple averaging (Boxcar/Sinc filter) to downsample, followed by a weak 1-pole LPF.
*   **Impact**: The Teensy is passing low-frequency rumble (0-300Hz) and high-frequency aliasing (above 4kHz folded down) into the standard Voter stream.

### B. Reference Logic (Voter2)
1.  **Capture**: 8kHz clean.
2.  **MakeSigned**: Convert 12-bit ADC to 16-bit Signed.
3.  **RSSI Filter**: High-pass (2400-4000Hz) to detect noise floor.
4.  **Voice Filter**: FIR Bandpass (300-2400Hz). **We must port this.**
5.  **De-Emphasis**: Optional 75us filter.

## 4. Action Plan
1.  **Port `VoiceFilter`**: Create a class in TeensyVoter using the exact 65 coefficients from Voter2.
2.  **Integrate**: Insert this filter into `DSPProcessor` *after* downsampling to 8kHz.
3.  **Verify**: Buzz should disappear as out-of-band noise is crushed.

## 4. Discovered Divergences (To Be Filled)
*   [ ] ...
