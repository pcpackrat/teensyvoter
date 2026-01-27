#include "VoiceFilter.h"
#include <string.h>

// 65-Tap FIR Bandpass Filter (300Hz - 2400Hz @ 8kHz)
// Ported from Voter2 (STM32) firmware
static const int16_t voice_filter_taps1[VOICEFILTER_TAP_NUM] = {
    1237,  507,   -869,  -1251, -343,  -105,  -614,  -236,  316,   -110,  -67,
    683,   407,   57,    866,   823,   -13,   629,   945,   -391,  -211,  656,
    -1010, -1656, 31,    -1524, -3612, -674,  -1033, -6749, -1137, 14332, 14332,
    -1137, -6749, -1033, -674,  -3612, -1524, 31,    -1656, -1010, 656,   -211,
    -391,  945,   629,   -13,   823,   866,   57,    407,   683,   -67,   -110,
    316,   -236,  -614,  -105,  -343,  -1251, -869,  507,   1237};

VoiceFilter::VoiceFilter() { init(); }

void VoiceFilter::init() {
  memset(safe_history, 0, sizeof(safe_history));
  last_index = 0;
}

void VoiceFilter::put(int16_t input) {
  safe_history[(last_index++) & 127] = input;
}

int16_t VoiceFilter::get() {
  int64_t acc =
      0; // Use 64-bit to prevent overflow (65 taps * 16bit * 16bit > 32bit)
  int index = last_index - 1; // Start from most recent

  // Convolve
  for (int i = 0; i < VOICEFILTER_TAP_NUM; ++i) {
    acc += (int64_t)safe_history[(index--) & 127] * voice_filter_taps1[i];
  }

  // Voter2: return acc >> 16
  // Gain compensation? Voter2 says "This routine attenuates... needs extra 1.5x
  // amplifier" In Voter2 code: out[i] = ((int32_t)VoiceFilter_get1(&vf)*3)/2;
  // We will return raw result here and apply gain in DSPProcessor if needed,
  // OR just apply it here to match the "get" expectation.
  // Let's implement the pure filter get here (>> 16).

  return (int16_t)(acc >> 16);
}
