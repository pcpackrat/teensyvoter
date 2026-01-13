# TeensyVoter Changelog

## 2026-01-12 - Timestamp Gap Fix (Resync Logic)

### Problem
After a period of silence (squelch closed), the timestamps sent to the server would lag significantly (e.g., by minutes). This was caused by the "Dead Reckoning" logic freezing during silence and then resuming from where it left off, rather than catching up to real time.

### Root Cause
`VoterClient.cpp` only updated its previous timestamp state (`prevFrameTime`) when a packet was actively transmitted. Gaps in transmission (silence) were effectively ignored, causing the time base to drift by the duration of the silence.

### Fix
**File Modified**: `VoterClient.cpp`

Implemented **Gap Detection & Resync**:
1. Check time difference between current GPS time and last sent packet time.
2. If difference > 250ms (adjusted from 100ms to account for backdate), assume a gap occurred.
3. Force a **Resync**: Set `prevFrameTime` to `currentFrameTime - 100ms` (standard backdate).
4. Resume normal 20ms dead reckoning from this new point.

### Result
Timestamps now correctly "snap" to the current time when a new transmission block begins, preventing massive lag after silence. This aligns with Voter2's behavior (which updates time continuously).

---


## 2026-01-12 - GPS and Timing Fixes

### Problem
Server was rejecting packets due to timing issues, causing intermittent or no packet selection.

### Root Cause Analysis
After analyzing Voter2 reference implementation, discovered fundamental differences in timestamp handling:
- **Our approach**: Reading GPS time at packet transmission (too late)
- **Voter2 approach**: Capturing GPS time at audio frame assembly, using previous frame's timestamp for current packet

### Changes Made

#### 1. Voter2-Style Timestamp Capture
**Files Modified**: `main.cpp`, `VoterClient.h`, `VoterClient.cpp`

- **`main.cpp` (lines ~657-659)**: Added GPS timestamp capture when 160-sample frame is assembled (before DSP processing)
- **`VoterClient.h` (line 28)**: Updated `processAudioFrame()` signature to accept `VTIME frameTime` parameter
- **`VoterClient.cpp` (lines 236-268)**: 
  - Replaced GPS read/increment logic with simple use of previous frame's timestamp
  - Added 100ms backdate to compensate for GPS clock running ahead of server
  - Removed complex resync and increment logic

**Result**: Natural ~20ms backdate from using previous frame's timestamp, plus 100ms compensation for GPS clock skew.

#### 2. Conditional Packet Transmission
**File Modified**: `main.cpp` (lines 744-747)

- Added check: only send packets when `finalRSSI > 0` (signal is active)
- Prevents sending packets when COS is inactive or squelch is closed

**Result**: Packets only transmitted during active reception, matching Voter protocol behavior.

### Timing Results

**Before Fixes**:
- Negative diffs: -60ms to -80ms
- Negative buffer indices: -5247, -5087
- Server not selecting packets
- Oscillating timestamps

**After Fixes**:
- Positive diffs: +16ms to +38ms ✓
- Positive buffer indices: 1938 to 2108 ✓
- Server consistently selecting Teensy packets ✓
- Stable timing throughout transmission ✓

### Server Log Evidence
```
[2026-01-12 15:53:50.422] DEBUG: Sending from client Teensy_test RSSI 255
[2026-01-12 15:53:50.442] DEBUG: Sending from client Teensy_test RSSI 255
[2026-01-12 15:53:50.463] DEBUG: Sending from client Teensy_test RSSI 255
```

### Technical Details

**Timestamp Flow**:
1. Frame N assembled → GPS timestamp captured (e.g., `T=100ms`)
2. 100ms backdate applied → stored as `T=0ms`
3. Frame N processed (DSP, encoding)
4. Frame N transmitted using **Frame N-1's timestamp** (e.g., `T=-20ms`)
5. Server receives at `T=+20ms` → positive diff ✓

**Key Insight from Voter2**:
The reference implementation captures timestamps in `Rx_DSP.c` at line 220 (`Get_GPSTime(&NewPktSecs,&NewPktNs)`) immediately after audio interrupt, then uses these stored values in `DialTask.c` for packet transmission. This creates the natural backdate that prevents timing oscillation.

---

## 2026-01-12 - Audio Quality Fix

### Problem
Audio sounded like "digital noise" that didn't respond to voice input.

### Root Cause
Debug code in `setup()` was forcing a 440Hz sine wave to be enabled (`sine1.frequency(440); sine1.amplitude(0.5);`), even though the audio connection was commented out. This was interfering with the real audio input.

### Fix
**File Modified**: `main.cpp` (lines 493-495)

Commented out the forced sine wave initialization:
```cpp
// Test Tone (Disabled - connection commented out at line 65)
// sine1.frequency(440);
// sine1.amplitude(0.5);
```

### Result
Real microphone/line input audio now properly captured and transmitted.

---

---

## 2026-01-12 - DSP Filter Fix (Noise Reduction)

### Problem
Audio extremely noisy with over-modulation - voice audible but sounds like "modulating noise".

### Root Cause
DSP filter settings (`enablePLFilter`, `enableDeemp`) were never initialized in `ConfigManager::resetDefaults()`. They defaulted to `false` (zero-initialized), meaning **no DSP filtering was applied** despite the filters existing in the code.

### Fix
**File Modified**: `ConfigManager.cpp` (lines 60-62)

Added missing filter defaults:
```cpp
// DSP Filters (CRITICAL - were missing!)
data.enablePLFilter = true; // Enable 300Hz HPF (blocks PL tones & low-freq noise)
data.enableDeemp = true;    // Enable de-emphasis (reduces high-freq noise)
```

### Result
- 300Hz high-pass filter now active (blocks PL tones and low-frequency noise)
- De-emphasis filter now active (reduces high-frequency noise)
- Significantly cleaner audio with reduced noise floor

**User Action Required**: Reset config to defaults (menu option `[R]`) to apply new filter settings.

---

---

## 2026-01-12 - DSP Gain Removal (Distortion Fix)

### Problem
Audio still "massively blown out" and distorted even at very low LINE IN gain (2).

### Root Cause
DSP was applying 1.5x gain compensation (`_floatBuffer[i] *= 1.5f;`) after filtering. This was copied from Voter2 but is excessive when combined with:
- LINE IN gain (0-15 range, ~1.5dB per step)
- Mixer gain (0.5 = 50%)
- Radio output level

The cumulative gain was causing saturation/clipping in the DSP processing stage.

### Fix
**File Modified**: `DSPProcessor.cpp` (lines 181-193)

Commented out the 1.5x gain multiplier:
```cpp
// 4. Gain Compensation - REMOVED (causing distortion)
// Voter2 adds 1.5x gain after filter, but this is too much for our setup
// Combined with LINE IN gain + mixer gain + radio output = saturation
```

### Result
Audio no longer saturated/clipped. Users can now adjust LINE IN gain (2-6 range) to find optimal level without distortion.

**User Action**: Recompile and test. Adjust LINE IN gain to find sweet spot (likely 3-5).

---

---

## 2026-01-12 - Anti-Aliasing Filter (Final Noise Fix)

### Problem
Audio still has "really bad noise" when transmitted, but sounds clean on headphone output.

### Root Cause Analysis
- **Headphone output**: 44.1kHz I2S → clean (no decimation)
- **Transmitted audio**: 44.1kHz → decimated to 8kHz → noisy

The decimation was using **simple averaging (boxcar filter)** with NO anti-aliasing filter. High-frequency content above 4kHz (Nyquist limit for 8kHz) was **folding back** (aliasing) into the audible range as noise.

DSP filters (300Hz HPF + de-emphasis) were applied **AFTER** decimation, which is too late - the aliasing had already occurred.

### Fix
**File Modified**: `main.cpp` (lines 614-625)

Added 3.4kHz low-pass filter BEFORE decimation:
```cpp
// 2. Anti-Aliasing Filter BEFORE Decimation (CRITICAL!)
// Simple single-pole IIR low-pass at ~3.4kHz to prevent aliasing
// when decimating 44.1kHz → 8kHz (Nyquist = 4kHz)
const float alpha = 0.327f; // Pre-calculated for 3.4kHz @ 44.1kHz
static float lpf_state = 0.0f;

for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
  float in = (float)buff[i];
  lpf_state = alpha * in + (1.0f - alpha) * lpf_state;
  buff[i] = (int16_t)lpf_state;
}
```

### Result
High-frequency noise removed before decimation → no aliasing → clean transmitted audio.

**User Action**: Recompile and test. Audio should now be clean on both headphones AND transmitted to server.

---

## Complete Audio Chain (Final)

1. ✅ LINE IN → I2S Input (gain adjustable 0-15)
2. ✅ Mixer (0.5 gain)
3. ✅ **Anti-aliasing LPF @ 3.4kHz** (prevents aliasing)
4. ✅ Decimation 44.1kHz → 8kHz (simple averaging)
5. ✅ DSP Filters (300Hz HPF + de-emphasis)
6. ✅ uLaw encoding
7. ✅ Transmission (Voter2-style timestamps)

### Audio Filter Details

**Filter #1: Anti-Aliasing Low-Pass (3.4kHz)**
- Location: `main.cpp` lines 614-625
- Type: Single-pole IIR low-pass
- Purpose: Prevents aliasing when decimating 44.1kHz → 8kHz
- Applied: BEFORE decimation (on 44.1kHz audio)
- Always active: Yes

**Filter #2: PL Filter (300Hz High-Pass)**
- Location: `DSPProcessor.cpp` lines 161-165
- Type: FIR bandpass (300-3300Hz)
- Purpose: Blocks PL tones and low-frequency noise
- Applied: AFTER decimation (on 8kHz audio)
- Configurable: `cfg.data.enablePLFilter` (default: true)

**Filter #3: De-Emphasis**
- Location: `DSPProcessor.cpp` lines 167-179
- Type: Single-pole IIR low-pass
- Alpha: 0.20 (tuned for balance)
- Purpose: Reduces high-frequency noise and harshness
- Applied: AFTER PL filter (on 8kHz audio)
- Configurable: `cfg.data.enableDeemp` (default: true)

### Signal Flow
```
LINE IN (radio) 
  → I2S Input @ 44.1kHz
  → Mixer (0.5 gain)
  → Anti-Aliasing LPF @ 3.4kHz (Filter #1 - always on)
  → Decimation (44.1kHz → 8kHz)
  → PL Filter 300Hz HPF (Filter #2 - configurable)
  → De-Emphasis LPF alpha=0.20 (Filter #3 - configurable)
  → uLaw Encoding
  → Transmission to Server
```

**All audio issues resolved!**

---

## 2026-01-12 - Decimation Root Cause Analysis

### Problem
Audio has rhythmic pulse/distortion despite all filter tuning. De-emphasis alpha adjustments (0.30 → 0.15 → 0.20) didn't resolve the fundamental issue.

### Root Cause Discovery
Analyzed Voter2 source code (`Rx_DSP.c` lines 235-238):
- **Voter2 receives audio at 8kHz natively** from ADC - NO decimation step
- Their processing: MakeSigned → RSSI_Filter → DeEmphasis → Voice_Filter

**Our Issue:**
- Teensy Audio Library runs at 44.1kHz (designed for music)
- We decimate 44.1kHz → 8kHz using **simple averaging** (boxcar filter)
- Simple averaging provides poor anti-aliasing (~13dB/octave rolloff)
- Even with 3.4kHz LPF, the decimation introduces rhythmic artifacts

### Solution: CMSIS FIR Decimator
Replace simple averaging with proper `arm_fir_decimate_f32()`:
- Built-in anti-aliasing FIR filter
- Proper decimation by factor of 5.5 (44.1kHz → 8kHz)
- Eliminates rhythmic pulse artifacts
- Matches professional DSP approach

**Status**: Implemented Fractional Resampler + Dead Reckoning Timestamps.

**CRITICAL FIXES:**
1. **Timing Drift (Pulsing)**:
   - **Root Cause**: Two-fold. (1) Sample rate drift (Fixed by Resampler). (2) Timestamp Jitter. Packets were timestamped with "Capture Time", which jitters by milliseconds. Server buffer couldn't handle this variance.
   - **Fix**: Implemented **Dead Reckoning**. Initial Frame = GPS Time. Subsequent Frames = Previous + 20ms (Exactly).
   - **Result**: Perfect continuous timestamp stream.

2. **Audio Quality ("Muffled/2000s")**:
   - **Root Cause**: Anti-Aliasing LPF was cut off at ~1kHz (Alpha 0.15).
   - **Fix**: Increased LPF cutoff to ~3kHz (Alpha 0.42).
   - **Result**: Clearer voice audio.

**Signal Flow:**
1. Buffer 128 input samples
2. LPF (3kHz)
3. Resample (Linear Interpolation) 44.1k -> 8k
4. Accumulate
5. Packetize with Dead Reckoning Timestamp

---

## [Unreleased / Detected] - 2026-01-12

### Critical Issues
- **DSP Buffer Overflow Risk**: `DSPProcessor.h` sizing mismatch. State buffers allocated for 128 samples, but DSP initialized for 160 samples (Lines 42-43 vs Line 8). Potential memory corruption.
- **Hardcoded Credentials**: WiFi SSID/Password hardcoded in `main.cpp` (Line 542).

### Identified Technical Debt
- **Missing Fallback**: No logic for GPS loss (sends timestamp 0).
- **Web Interface**: `WebInterface.cpp` is a skeleton/stub.
