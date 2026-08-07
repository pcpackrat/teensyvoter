#ifndef JITTER_BUFFER_H
#define JITTER_BUFFER_H

#include <Arduino.h>

// 8kHz * 1 byte (uLaw) = 8000 bytes/sec
// 500ms = 4000 bytes
// Let's allocate enough for ~2 seconds to be safe
#define JITTER_BUF_SIZE 16384

class JitterBuffer {
public:
  JitterBuffer();

  // Clears the buffer and resets state
  void reset();

  // Push data into the buffer (Packet Receiver)
  // Returns true if success, false if buffer full
  bool put(const uint8_t *data, size_t len);

  // Pull data from the buffer (Audio Playback)
  // Returns number of bytes actually read (may be 0 if buffering or empty)
  size_t get(uint8_t *out, size_t len);

  // Returns number of bytes available to read
  size_t available();

  // Returns number of bytes free to write
  size_t space();

  // Status
  bool isBuffering() { return _buffering; }
  float getBufferUsage(); // 0.0 to 1.0

  // Tuning
  void setLatency(uint16_t ms);

public:
  uint8_t _buffer[JITTER_BUF_SIZE];
  volatile size_t _head; // Write index
  volatile size_t _tail; // Read index

private:

  // Jitter Handling
  bool _buffering;     // True if we are filling up, False if playing
  size_t _targetLevel; // Bytes to accumulate before playing (Start Threshold)

  // Diagnostics - isolating whether remaining audio jitter is from real
  // buffer underruns (network-side gaps) or something further downstream
  // in playback. Rate-limited summary rather than per-event, since a
  // partial shortfall can legitimately happen often on a real network.
  uint32_t _underrunCount;        // avail==0, full re-buffer triggered
  uint32_t _partialShortfallCount; // toRead < len, silence-padded but no re-buffer
  uint32_t _lastReportTime;
  void _reportIfDue();
};

#endif
