#include "DSPProcessor.h"
#include "ConfigManager.h"
#include <Arduino.h>
#include <math.h>

extern ConfigManager cfg; // Access global config

DSPProcessor::DSPProcessor() {
  memset(_rssiState, 0, sizeof(_rssiState));
  memset(_voiceState, 0, sizeof(_voiceState));
}

void DSPProcessor::begin() {
  // 1. Calculate Filter Coefficients (Convert Q15 to Float)
  _calculateCoeffs();

  // 2. Init FIR Filters
  // 23 taps for RSSI, 64 taps for Voice
  arm_fir_init_f32(&_rssiFilter, 23, _rssiCoeffs, _rssiState,
                   AUDIO_BLOCK_SAMPLES);
  arm_fir_init_f32(&_voiceFilter, 64, _voiceCoeffs, _voiceState,
                   AUDIO_BLOCK_SAMPLES);

  // 3. Init Biquad HPF
  arm_biquad_cascade_df1_init_f32(&_hpf, 1, _hpfCoeffs, _hpfState);
}

void DSPProcessor::_calculateCoeffs() {
  // Coefficients from Voter2 Project (Q15 format)
  // We convert them to float by dividing by 32768.0f

  // --- RSSI Filter (High Pass > 2400Hz) - 23 Taps ---
  // Source: RSSIFilter.c (RSSIFILTER1)
  static const int16_t rssi_q15[] = {128,   402,   -1225, 1927,  -1674, 164,
                                     1729,  -2249, 54,    4461,  -9036, 10962,
                                     -9036, 4461,  54,    -2249, 1729,  164,
                                     -1674, 1927,  -1225, 402,   128};

  for (int i = 0; i < 23; i++) {
    _rssiCoeffs[i] = (float)rssi_q15[i] / 32768.0f;
  }

  // --- Voice Filter (Band Pass 300Hz - 2400Hz) - 64 Taps ---
  // Source: VoiceFilter.c (voice_filter_taps2 - Passes CTCSS)
  // User can likely switch between tap1 and tap2 later, but tap2 (Passes CTCSS)
  // is safer default. Wait, the original code uses taps2 for Flat Audio (CTCSS
  // passed) and taps1 for filtered. Let's implement Taps1 (Voice Only, No PL)
  // as standard default, unless requested otherwise. Actually, usually "Flat"
  // means NO filtering. Voter2: if (DIAL_FLATAUDIO) use taps2 (Pass CTCSS).
  // Else use taps1 (Block CTCSS). Let's use Taps1 (Block CTCSS) as the default
  // "Processed" audio.

  // --- Generate Wide Bandpass FIR (300Hz - 3300Hz) ---
  // Ideal for NBFM Voice (removes PL, passes full voice).
  // Uses Windowed Sinc method (Blackman Window).

  float fl = 300.0f / SAMPLE_RATE;  // Low Cut (Normalized)
  float fh = 3300.0f / SAMPLE_RATE; // High Cut (Normalized)
  int N = 64;                       // Taps
  int M = N - 1;

  for (int n = 0; n < N; n++) {
    float k = n - M / 2.0f;
    float h_n;

    if (k == 0.0f) {
      h_n = 2.0f * (fh - fl);
    } else {
      float pi_k = PI * k;
      // Bandpass = LowPass(fh) - LowPass(fl)
      // LowPass(f) = 2f * sinc(2*pi*f*k) = 2f * sin(2*pi*f*k)/(2*pi*f*k) =
      // sin(2*pi*f*k)/(pi*k)
      h_n = (sinf(2.0f * PI * fh * k) - sinf(2.0f * PI * fl * k)) / pi_k;
    }

    // Blackman Window
    // w[n] = 0.42 - 0.5 cos(2pi n / M) + 0.08 cos(4pi n / M)
    float w = 0.42f - 0.5f * cosf(2.0f * PI * n / M) +
              0.08f * cosf(4.0f * PI * n / M);

    _voiceCoeffs[n] = h_n * w;
  }

  // Update Biquad? No, we are switching back to FIR.
  // We can leave the Biquad junk in the header, it won't hurt.

  // --- Biquad HPF 300Hz Coeffs (2nd Order Butterworth, Fs=8000) ---
  // Calculated: b={0.846, -1.692, 0.846}, a={1.0, -1.669, 0.716}
  // CMSIS Format: {b0, b1, b2, a1, a2} where a1/a2 are negated
  // So a1_stored = -(-1.669) = 1.669. a2_stored = -(0.716) = -0.716.
  // Wait, standard form: y[n] = b0*x + ... - a1*y[n-1] ...
  // T.F. Denom: 1 - 1.669 z^-1 + 0.716 z^-2.
  // So a1 = -1.669, a2 = 0.716.
  // CMSIS expects NEGATED coefficients for the feedback path if the loop adds
  // them. Standard CMSIS df1: y[n] = b0*x + ... + a1*y[n-1] + a2*y[n-2] So we
  // store a1, a2 as positive 1.669, -0.716.
  _hpfCoeffs[0] = 0.846f;  // b0
  _hpfCoeffs[1] = -1.692f; // b1
  _hpfCoeffs[2] = 0.846f;  // b2
  _hpfCoeffs[3] = 1.669f;  // a1 (Feedback 1)
  _hpfCoeffs[4] = -0.716f; // a2 (Feedback 2)
}

uint8_t DSPProcessor::process(int16_t *samples, bool enablePLFilter,
                              bool enableDeemp) {
  // 1. Convert Input to Float
  arm_q15_to_float(samples, _floatBuffer, AUDIO_BLOCK_SAMPLES);

  // ---------------------------------------------------------
  // Path A: RSSI Calculation (Side Chain)
  // ---------------------------------------------------------
  // Apply RSSI Filter (High Pass > 2.4kHz) to a copy
  arm_fir_f32(&_rssiFilter, _floatBuffer, _scratchBuffer, AUDIO_BLOCK_SAMPLES);

  // Calculate RMS of the High-Passed Signal (Noise)
  float energy = 0.0f;
  for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
    energy += _scratchBuffer[i] * _scratchBuffer[i];
  }
  float rms = sqrtf(energy / (float)AUDIO_BLOCK_SAMPLES);

  // Convert Float RMS (0.0-1.0) back to Q15 scale (0-32768) roughly for formula
  // match Voter2 formula: cooked_rssi = 255 - (rms_accum / 13) where rms_accum
  // is Q15.
  float factor = cfg.data.dspCalib;
  if (factor < 1.0f)
    factor = 1.0f; // Safety
  float rms_q15 = rms * 32768.0f;
  float cooked_rssi = 255.0f - (rms_q15 / factor);

  // Clamp range.
  // We clamp low end to 10 ensuring we always send a "Weak" signal rather than
  // "Dead" if we are actively processing. External logic controls
  // Squelch/Zeroing.
  if (cooked_rssi < 10)
    cooked_rssi = 10;
  if (cooked_rssi > 255)
    cooked_rssi = 255;

  uint8_t finalRSSI = (uint8_t)cooked_rssi;

  // Minimal Debug: 1Hz
  static int dspLog = 0;
  if (dspLog++ > 350) { // ~350 frames = 1 sec
    dspLog = 0;
    Serial.printf("[DSP] RSSI:%d RMS:%.1f\r\n", finalRSSI, rms);
  }

  // Store noise level (inverse of RSSI) for squelch detection
  _lastNoiseLevel = 255 - finalRSSI;

  // ---------------------------------------------------------
  // Path B: Audio Processing (Main Chain)
  // ---------------------------------------------------------
  // Note: Voter2 sequence: DeEmphasis -> VoiceFilter

  // 2. Voice Filter (Wide Bandpass FIR 300-3300Hz)
  // This uses the "Sinc Generated" coefficients.
  // LINEAR PHASE (No Jitter).
  // FULL BANDWIDTH (No Thinness).
  if (enablePLFilter) {
    arm_fir_f32(&_voiceFilter, _floatBuffer, _scratchBuffer,
                AUDIO_BLOCK_SAMPLES);
    memcpy(_floatBuffer, _scratchBuffer, AUDIO_BLOCK_SAMPLES * sizeof(float));
  }

  // 3. De-emphasis
  // Alpha tuned to 0.20 (middle ground between 0.15 too weak, 0.30 too
  // aggressive) Balances noise reduction with voice clarity
  if (enableDeemp) {
    const float alpha = 0.20f; // Moderate de-emphasis
    const float beta = 1.0f - alpha;
    for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
      float in = _floatBuffer[i];
      float out = alpha * in + beta * _deempState;
      _floatBuffer[i] = out;
      _deempState = out; // Persist
    }
  }

  // 4. Gain Compensation - REMOVED (causing distortion)
  // Voter2 adds 1.5x gain after filter, but this is too much for our setup
  // Combined with LINE IN gain + mixer gain + radio output = saturation
  // for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
  //   _floatBuffer[i] *= 1.5f;
  //   // Clip
  //   if (_floatBuffer[i] > 1.0f)
  //     _floatBuffer[i] = 1.0f;
  //   if (_floatBuffer[i] < -1.0f)
  //     _floatBuffer[i] = -1.0f;
  // }

  // 5. Convert back to Int16
  arm_float_to_q15(_floatBuffer, samples, AUDIO_BLOCK_SAMPLES);

  return finalRSSI;
}

// uLaw Encode Helper
#define BIAS 0x84
#define CLIP 32635

static uint8_t linear2ulaw(int16_t sample) {
  static int16_t seg_uend[8] = {0x3F,  0x7F,  0xFF,  0x1FF,
                                0x3FF, 0x7FF, 0xFFF, 0x1FFF};

  int16_t mask;
  int16_t seg;
  uint8_t uval;

  if (sample < 0) {
    sample = BIAS - sample;
    mask = 0x7F;
  } else {
    sample += BIAS;
    mask = 0xFF;
  }

  if (sample > CLIP)
    sample = CLIP;

  seg = 0;
  for (int i = 0; i < 8; i++) {
    if (sample <= seg_uend[i]) {
      seg = i;
      break;
    }
  }

  if (seg >= 8)
    return (0x7F ^ mask);

  uval = (seg << 4) | ((sample >> (seg + 3)) & 0xF);
  return (uval ^ mask);
}

void DSPProcessor::encodeULaw(int16_t *input, uint8_t *output, int count) {
  for (int i = 0; i < count; i++) {
    output[i] = linear2ulaw(input[i]);
  }
}
