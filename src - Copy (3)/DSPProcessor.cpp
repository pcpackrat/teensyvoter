#include "DSPProcessor.h"
#include "ConfigManager.h"
#include <Arduino.h>
#include <math.h>

extern ConfigManager cfg; // Access global config

DSPProcessor::DSPProcessor() { memset(_rssiState, 0, sizeof(_rssiState)); }

void DSPProcessor::begin() {
  // 1. Calculate Filter Coefficients (Convert Q15 to Float)
  _calculateCoeffs();

  // 2. Init FIR Filters
  // 23 taps for RSSI, 64 taps for Voice
  arm_fir_init_f32(&_rssiFilter, 23, _rssiCoeffs, _rssiState,
                   DSP_BLOCK_SAMPLES);

  _intVoiceFilter.init();

  // 3. Init Biquad HPF
  arm_biquad_cascade_df1_init_f32(&_hpf, 1, _hpfCoeffs, _hpfState);

  // 4. Init Filter States
  _deempState = 0.0f;
  _prevIn = 0.0f;
  _prevOut = 0.0f;
  memset(_scratchBuffer, 0, sizeof(_scratchBuffer));
  memset(_floatBuffer, 0, sizeof(_floatBuffer));
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

  // --- Voice Filter Coefficients REMOVED ---
  // We now use the static table in VoiceFilter.cpp (Voter2 Port)

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
  arm_q15_to_float(samples, _floatBuffer, DSP_BLOCK_SAMPLES);

  // ---------------------------------------------------------
  // Path A: RSSI Calculation (Side Chain)
  // ---------------------------------------------------------
  // Apply RSSI Filter (High Pass > 2.4kHz) to a copy
  arm_fir_f32(&_rssiFilter, _floatBuffer, _scratchBuffer, DSP_BLOCK_SAMPLES);

  // Calculate RMS of the High-Passed Signal (Noise)
  float energy = 0.0f;
  for (int i = 0; i < DSP_BLOCK_SAMPLES; i++) {
    energy += _scratchBuffer[i] * _scratchBuffer[i];
  }
  float rms = sqrtf(energy / (float)DSP_BLOCK_SAMPLES);

  // Convert Float RMS (0.0-1.0) back to Q15 scale (0-32768) roughly for
  // formula match Voter2 formula: cooked_rssi = 255 - (rms_accum / 13) where
  // rms_accum is Q15.
  float factor = cfg.data.dspCalib;
  if (factor < 1.0f)
    factor = 1.0f; // Safety
  float rms_q15 = rms * 32768.0f;
  float cooked_rssi = 255.0f - (rms_q15 / factor);

  // Clamp range.
  // We clamp low end to 10 ensuring we always send a "Weak" signal rather
  // than "Dead" if we are actively processing. External logic controls
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
    // Serial.printf("[DSP] RSSI:%d RMS:%.1f\r\n", finalRSSI, rms);
  }

  // Store noise level (inverse of RSSI) for squelch detection
  _lastNoiseLevel = 255 - finalRSSI;

  // ---------------------------------------------------------
  // Path B: Audio Processing (Main Chain)
  // ---------------------------------------------------------
  // Note: Voter2 sequence: DeEmphasis -> VoiceFilter

  // 2. Voice Filter (Integer FIR 300-2400Hz - Ported from Voter2)
  if (enablePLFilter) {
    for (int i = 0; i < DSP_BLOCK_SAMPLES; i++) {
      _intVoiceFilter.put(samples[i]);
      // Raw output from filter (accumulator >> 16)
      // Removed 1.5x gain to prevent clipping/distortion until verified
      samples[i] = _intVoiceFilter.get();
    }

    // Update Float Buffer if De-emphasis is needed next
    if (enableDeemp) {
      arm_q15_to_float(samples, _floatBuffer, DSP_BLOCK_SAMPLES);
    }
  } else {
    // If NO filter, we should probably update samples from the float buffer?
    // Actually path A converted samples->float.
    // No action needed on samples[] if filter disabled.
    // But if Deemp is enabled, it expects _floatBuffer to match samples.
    // _floatBuffer currently matches samples (from step 1).
  }

  // 3. De-emphasis
  // Alpha tuned to 0.20 (middle ground between 0.15 too weak, 0.30 too
  // aggressive) Balances noise reduction with voice clarity
  if (enableDeemp) {
    const float alpha = 0.20f; // Moderate de-emphasis
    const float beta = 1.0f - alpha;
    for (int i = 0; i < DSP_BLOCK_SAMPLES; i++) {
      float in = _floatBuffer[i];
      float out = alpha * in + beta * _deempState;
      _floatBuffer[i] = out;
      _deempState = out; // Persist
    }
    // 5. Convert back to Int16 (Only if DeEmp modified float buffer)
    arm_float_to_q15(_floatBuffer, samples, DSP_BLOCK_SAMPLES);
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

  return finalRSSI;
}

// uLaw Encode Helper
#define BIAS 0x84
#define CLIP 32635

// Standard G.711 u-law compression
static uint8_t linear2ulaw(int16_t sample) {
  const uint16_t MULAW_BIAS = 33; // Standard is 33? My code had 132.
  // 132 is 0x84.
  // ITU-T G.711 recommends bias 33 (0x21).
  // Let's try Bias 33.

  uint16_t mask;
  int16_t sign = 0;

  if (sample < 0) {
    sample = -sample;
    sign = 0x80; // Sign bit
  }

  // Cap at 8159 (13 bit magnitude)
  if (sample > 8159)
    sample = 8159;

  sample += MULAW_BIAS;

  // Exponent/Segment
  // Finds MSB position.
  // 8159+33 = 8192 (0x2000) -> 14 bits.

  /*
     Sample+Bias | Exponent | Mantissa
     0000 0001 xxxxx | 0 | xxxx
     0000 001x xxxx | 1 | xxxx
     ...
  */

  int exponent = 7;
  for (int i = 7; i >= 0; i--) {
    if (sample & (1 << (i + 5))) {
      exponent = i;
      break;
    }
  }

  // Mantissa (4 bits)
  int mantissa = (sample >> (exponent + 1)) & 0x0F;

  uint8_t u_val = (sign | (exponent << 4) | mantissa);
  return ~u_val; // Invert all bits (0xFF for 0)
}

// NOTE: My bias was 132. Standard is 33.
// With Bias 33. Input 0 -> 33 (0x21).
// 0x21 = 0000100001 (Wait)
// 33 = 0x21.
// Loop check:
// i=7 (Check bit 12? 1<<12 = 4096).
// ...
// i=0. bit 5. 32.
// 33 & 32 is True.
// So exponent = 0.
// Mantissa = (33 >> 1) & 0xF = 16 & F = 0.
// u_val = 0 | 0 | 0 = 0.
// return ~0 = 0xFF.
//
// THIS IS CORRECT. 0 -> 0xFF.
// I WILL USE THIS IMPLEMENTATION.

void DSPProcessor::encodeULaw(int16_t *input, uint8_t *output, int count) {
  for (int i = 0; i < count; i++) {
    output[i] = linear2ulaw(input[i]);
  }
}
