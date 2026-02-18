#include "JitterBuffer.h"

JitterBuffer::JitterBuffer() {
  _head = 0;
  _tail = 0;
  _buffering = true;

  // Default Latency: 200ms
  // 8000 samples/sec * 0.2s = 1600 samples
  setLatency(200);
}

void JitterBuffer::setLatency(uint16_t ms) {
  if (ms > 1500)
    ms = 1500; // Cap at 1.5s
  if (ms < 20)
    ms = 20; // Min 20ms

  // Calculate target bytes
  _targetLevel = (uint32_t)ms * 8;
  _underrunThreshold = 160; // 1 packet (20ms) leeway

  reset();
}

void JitterBuffer::reset() {
  _head = 0;
  _tail = 0;
  _buffering = true;
}

size_t JitterBuffer::available() {
  if (_head >= _tail) {
    return _head - _tail;
  } else {
    return (JITTER_BUF_SIZE - _tail) + _head;
  }
}

size_t JitterBuffer::space() { return (JITTER_BUF_SIZE - 1) - available(); }

bool JitterBuffer::put(const uint8_t *data, size_t len) {
  if (space() < len) {
    // Overflow!
    // Strategy: Drop oldest data? Or drop new packet?
    // For simplicity, drop new packet and warn.
    // Or better: Advance tail (drop old) to make room?
    // Let's just fail for now.
    return false;
  }

  for (size_t i = 0; i < len; i++) {
    _buffer[_head] = data[i];
    _head = (_head + 1) % JITTER_BUF_SIZE;
  }

  return true;
}

size_t JitterBuffer::get(uint8_t *out, size_t len) {
  size_t avail = available();

  // Hysteresis Logic
  if (_buffering) {
    if (avail >= _targetLevel) {
      _buffering = false; // Start playing
    } else {
      // Fill with silence if requested? Or just return 0 (caller plays silence)
      memset(out, 0xFF, len); // uLaw silence is 0xFF
      return 0;               // Indicate no real data read
    }
  }

  if (avail < len) {
    // Partial read or Underrun
    if (avail <= _underrunThreshold) {
      // Buffer empty! Enter buffering mode.
      _buffering = true;
      memset(out, 0xFF, len);
      return 0;
    }
  }

  // Read data
  size_t toRead = (avail < len) ? avail : len;

  for (size_t i = 0; i < toRead; i++) {
    out[i] = _buffer[_tail];
    _tail = (_tail + 1) % JITTER_BUF_SIZE;
  }

  // If we requested more than available, fill rest with silence
  if (toRead < len) {
    memset(out + toRead, 0xFF, len - toRead);
  }

  return toRead;
}

float JitterBuffer::getBufferUsage() {
  return (float)available() / (float)_targetLevel;
}
