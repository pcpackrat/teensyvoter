#ifndef DSP_PROCESSOR_H
#define DSP_PROCESSOR_H

#include "VoiceFilter.h"
#include <Arduino.h>
#include <arm_math.h>

// Audio Settings
#define DSP_BLOCK_SAMPLES 160 // Aligned with Voter Protocol (20ms Frame)
#define SAMPLE_RATE 8000.0f
#define FFT_SIZE 256 // Need at least block size

// Forward declaration
class AudioPlayQueue;

class DSPProcessor {
public:
  DSPProcessor();

  void begin();

  // Process a block of audio (in-place modification)
  // input: 160 samples of int16 (DSP_BLOCK_SAMPLES)
  // enablePLFilter: High Pass > 300Hz
  // enableDeemp: Low Pass (6dB/oct)
  // Returns: Calculated RSSI (0-255) based on Noise Floor
  uint8_t process(int16_t *samples, bool enablePLFilter, bool enableDeemp);
  
  void setRxDigitalGainPct(uint8_t pct) { _rxDigitalGain = (float)pct / 100.0f; }

  // Convert linear PCM to uLaw
  void encodeULaw(int16_t *input, uint8_t *output, int count);

  // Convert uLaw to linear PCM
  void decodeULaw(uint8_t *input, int16_t *output, int count);

  // Upsample (8k -> 44.1k) and push to PlayQueue
  void upsampleAndPlay(int16_t *input, int count, AudioPlayQueue &queue);

  // Get last measured noise level (0-255, higher = more noise)
  uint8_t getNoiseLevel() const { return _lastNoiseLevel; }

private:
  // FIR Filter instances
  arm_fir_instance_f32 _rssiFilter;
  // arm_fir_instance_f32 _voiceFilter; // Replaced by _intVoiceFilter

  // Coefficients (Converted to Float)
  float _rssiCoeffs[24]; // 23 taps + 1 for alignment if needed, but f32 doesn't
                         // need align
  float _rssiState[DSP_BLOCK_SAMPLES + 24];

  // Integer Voice Filter
  VoiceFilter _intVoiceFilter;

  float _scratchBuffer[DSP_BLOCK_SAMPLES]; // Scratch buffer for processing

  // FFT State for Squelch
  arm_rfft_fast_instance_f32 _fft;
  float _fftBuffer[FFT_SIZE];
  float _fftOutput[FFT_SIZE];

  // Internal Buffers
  float _floatBuffer[DSP_BLOCK_SAMPLES]; // Float conversion buffer

  // Last noise measurement (for squelch)
  uint8_t _lastNoiseLevel;

  // De-emphasis Filter State
  float _deempState;

  // Upsampler State
  double _upsamplePhase;
  int16_t _lastUpsampleVal;
  int16_t _reservoir[32];
  int _reservoirLen;
  uint32_t _droppedOutputBlocks; // getBuffer()==NULL, AudioMemory pool exhausted mid-upsample

  // DC Blocker State
  float _prevIn;
  float _prevOut;

  // Helpers
  void _calculateCoeffs();
  uint8_t _calculateRSSI(float *fftMag, int bins);

  float _rxDigitalGain; // Software boost for incoming audio (Radio -> Host)
};

#endif
