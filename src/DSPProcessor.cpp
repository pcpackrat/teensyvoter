#include "DSPProcessor.h"
#include "ConfigManager.h"
#include <Arduino.h>
#include <Audio.h> // For AudioPlayQueue
#include <math.h>

extern ConfigManager cfg; // Access global config

DSPProcessor::DSPProcessor() {
  memset(_rssiState, 0, sizeof(_rssiState));
  _upsamplePhase = 0.0f;
  _lastUpsampleVal = 0;
  _reservoirLen = 0;
  memset(_reservoir, 0, sizeof(_reservoir));
  _droppedOutputBlocks = 0;
  _rxDigitalGain = 1.0f;
}

void DSPProcessor::begin() {
  // 1. Calculate Filter Coefficients (Convert Q15 to Float)
  _calculateCoeffs();

  // 2. Init FIR Filters
  // 23 taps for RSSI, 64 taps for Voice
  arm_fir_init_f32(&_rssiFilter, 23, _rssiCoeffs, _rssiState,
                   DSP_BLOCK_SAMPLES);

  _intVoiceFilter.init();

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

  // 4. Digital RX Gain Stage (Teensy -> Host boost)
  if (_rxDigitalGain != 1.0f) {
    if (!enableDeemp && !enablePLFilter) {
       // If no other stage updated floatBuffer, do it now
       arm_q15_to_float(samples, _floatBuffer, DSP_BLOCK_SAMPLES);
    }
    for (int i = 0; i < DSP_BLOCK_SAMPLES; i++) {
      _floatBuffer[i] *= _rxDigitalGain;
      if (_floatBuffer[i] > 1.0f) _floatBuffer[i] = 1.0f;
      if (_floatBuffer[i] < -1.0f) _floatBuffer[i] = -1.0f;
    }
    arm_float_to_q15(_floatBuffer, samples, DSP_BLOCK_SAMPLES);
  }
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

static int16_t ulaw2linear(uint8_t u_val) {
  const uint16_t MULAW_BIAS = 33;
  u_val = ~u_val;

  int t = ((u_val & 0x0F) << 3) + MULAW_BIAS;
  t <<= ((unsigned)u_val & 0x70) >> 4;

  return ((u_val & 0x80) ? (MULAW_BIAS - t) : (t - MULAW_BIAS));
}

void DSPProcessor::decodeULaw(uint8_t *input, int16_t *output, int count) {
  for (int i = 0; i < count; i++) {
    int16_t val = ulaw2linear(input[i]);
    
    // Inject gentle Comfort Noise/Dither
    if (val != 0) {
      int comfortNoise = (rand() % 33) - 16;
      val += comfortNoise;
    }
    
    output[i] = val;
  }
}

void DSPProcessor::upsampleAndPlay(int16_t *input, int count,
                                   AudioPlayQueue &queue) {
  // 1. Combine Reservoir + New Input
  // Max needed reservoir is small ( < 1 sample worth of output steps in input
  // space? No, < 1 output block worth ) 128 output samples * (8000/44100) =
  // ~23.2 input samples. So 32 is safe.

  int totalLen = _reservoirLen + count;
  // We need a scratch buffer.
  // We can use _scratchBuffer (float) if we cast? No, strict aliasing.
  // Use a local buffer on stack. 160+32 is small.
  int16_t workBuf[200];
  if (totalLen > 200)
    totalLen = 200; // Protection

  // Construct combined buffer (We need a history of 1 sample before the current index, 
  // and 2 samples after, for 4-point Hermite interpolation)
  if (_reservoirLen > 0) {
    memcpy(workBuf, _reservoir, _reservoirLen * sizeof(int16_t));
  }
  memcpy(workBuf + _reservoirLen, input, count * sizeof(int16_t));
  
  // SANITY CHECK: If the entire buffer is digital silence (0 in PCM), 
  // ensure we don't carry over any phase noise.
  bool allSilence = true;
  for(int i=0; i<_reservoirLen; i++) if(_reservoir[i] != 0) { allSilence = false; break; }
  if(allSilence) {
    for(int i=0; i<count; i++) if(input[i] != 0) { allSilence = false; break; }
  }

  if (allSilence) {
    _upsamplePhase = 1.0; // Start at 1.0 to ensure idx-1 is safe for Hermite
  }

  // 2. Processing Loop (8kHz -> 44.1kHz)
  const int BLOCK_SIZE = 128;

  while (true) {
    // Check if we can satisfy a full block (128 output samples)
    const double step = 8000.0 / 44100.0;
    double endPhase = _upsamplePhase + (double)(BLOCK_SIZE - 1) * step;
    int endIdx = (int)ceil(endPhase);

    // Ensure we don't overrun
    if (endIdx >= totalLen - 1) break;

    // Get Output Buffer
    int16_t *out = queue.getBuffer();
    if (out == NULL) {
      // Audio library's memory pool is exhausted right now - this whole
      // 128-sample block (~2.9ms) is silently skipped, not queued, not
      // retried. A single call here can legitimately produce 6-7 blocks
      // (160 input samples at 8kHz upsampled to ~882 output samples at
      // 44.1kHz), so if the pool is already fairly full when a call
      // starts, running out partway through is real and would sound
      // exactly like the jitter this is chasing - invisible to every
      // upstream counter (SPI, jitter buffer) since it's entirely local
      // to this playback pipeline. Counting to confirm/deny.
      _droppedOutputBlocks++;
      static uint32_t lastDropReport = 0;
      uint32_t now = millis();
      if (now - lastDropReport > 5000) {
        lastDropReport = now;
        Serial.printf("[DSP] Dropped %lu output block(s) so far (AudioMemory pool exhausted mid-upsample)\r\n",
                      (unsigned long)_droppedOutputBlocks);
      }
      _upsamplePhase += (double)BLOCK_SIZE * step;
      continue;
    }

    // CATMULL-ROM CUBIC SPLINE UPSAMPLING (Polyphase Resampler)
    // Completely eliminates "staircase" Nytendo fuzziness associated with ZOH/Linear.
    for (int i = 0; i < BLOCK_SIZE; i++) {
      int idx = (int)_upsamplePhase;
      float mu = _upsamplePhase - (double)idx;

      float p0 = (idx >= 1) ? workBuf[idx - 1] : workBuf[0];
      float p1 = workBuf[idx];
      float p2 = workBuf[idx + 1];
      float p3 = (idx + 2 < totalLen) ? workBuf[idx + 2] : workBuf[totalLen - 1];

      // Cubic coefficients
      float mu2 = mu * mu;
      float a0 = -0.5f*p0 + 1.5f*p1 - 1.5f*p2 + 0.5f*p3;
      float a1 = p0 - 2.5f*p1 + 2.0f*p2 - 0.5f*p3;
      float a2 = -0.5f*p0 + 0.5f*p2;
      float a3 = p1;

      float v = a0*mu*mu2 + a1*mu2 + a2*mu + a3;
      
      // Hard limiter
      if (v > 32767.0f) v = 32767.0f;
      if (v < -32768.0f) v = -32768.0f;
      
      out[i] = (int16_t)v;
      _upsamplePhase += step;
    }

    queue.playBuffer();
    // Check for next block loop condition is handled at the start of while
  }

  // 3. Save Residue
  int consumed = (int)_upsamplePhase;
  int remaining = totalLen - consumed;

  // Bound must match _reservoir's actual declared size (int16_t[32] in
  // DSPProcessor.h) - this previously allowed up to 99, a real
  // out-of-bounds write into adjacent class members whenever the resample
  // loop exited early (e.g. queue.getBuffer() returning NULL on its first
  // iteration) and left a large unconsumed tail. Normal steady-state
  // remaining is ~23-24 samples (about one block's worth of input), well
  // under 32, so this only ever fired under exactly the abnormal
  // condition most likely to also be causing audible glitches.
  if (remaining > 0 && remaining <= (int)(sizeof(_reservoir) / sizeof(_reservoir[0]))) {
    memcpy(_reservoir, workBuf + consumed, remaining * sizeof(int16_t));
    _reservoirLen = remaining;
    _upsamplePhase -= (double)consumed;
  } else {
    _reservoirLen = 0;
    _upsamplePhase = 0.0;
  }
}
