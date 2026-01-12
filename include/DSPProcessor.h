#ifndef DSP_PROCESSOR_H
#define DSP_PROCESSOR_H

#include <Arduino.h>
#include <arm_math.h>

// Audio Settings
#define AUDIO_BLOCK_SAMPLES 160 // Aligned with Voter Protocol (20ms Frame)
#define SAMPLE_RATE 8000.0f
#define FFT_SIZE 256 // Need at least block size

class DSPProcessor {
public:
  DSPProcessor();

  void begin();

  // Process a block of audio (in-place modification)
  // input: 128 samples of int16
  // enablePLFilter: High Pass > 300Hz
  // enableDeemp: Low Pass (6dB/oct)
  // Returns: Calculated RSSI (0-255) based on Noise Floor
  uint8_t process(int16_t *samples, bool enablePLFilter, bool enableDeemp);

  // Convert linear PCM to uLaw
  void encodeULaw(int16_t *input, uint8_t *output, int count);

  // Get last measured noise level (0-255, higher = more noise)
  uint8_t getNoiseLevel() const { return _lastNoiseLevel; }

private:
  // FIR Filter instances
  arm_fir_instance_f32 _rssiFilter;
  arm_fir_instance_f32 _voiceFilter;

  // Coefficients (Converted to Float)
  float _rssiCoeffs[24]; // 23 taps + 1 for alignment if needed, but f32 doesn't
                         // need align
  float _voiceCoeffs[64];

  // State Buffers (Block Size + NumTaps - 1)
  float _rssiState[128 + 24];
  float _voiceState[128 + 64];

  // Biquad HPF (300Hz)
  arm_biquad_casd_df1_inst_f32 _hpf;
  float _hpfState[4];  // 4 state vars per stage (1 stage)
  float _hpfCoeffs[5]; // {b0, b1, b2, a1, a2}

  float _scratchBuffer[256]; // Need a scratch buffer for split path

  // FFT State for Squelch
  arm_rfft_fast_instance_f32 _fft;
  float _fftBuffer[FFT_SIZE];
  float _fftOutput[FFT_SIZE];

  // Internal Buffers
  float _floatBuffer[AUDIO_BLOCK_SAMPLES];

  // Last noise measurement (for squelch)
  uint8_t _lastNoiseLevel;

  // De-emphasis Filter State
  float _deempState;

  // DC Blocker State
  float _prevIn;
  float _prevOut;

  // Helpers
  void _calculateCoeffs();
  uint8_t _calculateRSSI(float *fftMag, int bins);
};

#endif
