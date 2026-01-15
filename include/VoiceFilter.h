#ifndef VOICEFILTER_H
#define VOICEFILTER_H

#include <stdint.h>

#define VOICEFILTER_TAP_NUM 65

class VoiceFilter {
public:
  VoiceFilter();
  void init();
  void put(int16_t input);
  int16_t get();

private:
  int16_t history[64]; // Power of 2 buffer len for easy masking (size 64 is
                       // barely not enough for 65 taps? wait)
  // T-Filter code uses a circular buffer.
  // Voter2 code: f->history[(index--) & 63]
  // 64 history elements. 65 taps.
  // Wait, the T-filter code in Voter2 has 65 taps but mask is & 63.
  // Let's re-read Voter2 source carefully.
  // "f->history[(f->last_index++) & 63] = input;"
  // "acc += (int32_t)f->history[(index--) & 63] * voice_filter_taps1[i];"
  // coefficients size is 65.
  // loop i goes 0 to 64.
  // If we only have 64 history items, we can't store 65 samples history.
  // This implies the T-Filter code might be relying on something subtle or the
  // 65th tap is barely used or it logic wraps. Actually, T-Filter generator
  // usually generates N taps. The buffer size must be >= N. 63 mask implies 64
  // size. 65 taps > 64 size. This looks like a bug in Voter2 reference or
  // acceptable loss? Let's use 128 size mask & 127 to be safe and correct.

  int16_t safe_history[128];
  unsigned int last_index;
};

#endif // VOICEFILTER_H
