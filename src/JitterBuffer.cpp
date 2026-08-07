#include "JitterBuffer.h"

JitterBuffer::JitterBuffer() {
  _head = 0;
  _tail = 0;
  _buffering = true;
  _underrunCount = 0;
  _partialShortfallCount = 0;
  _lastReportTime = 0;

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

  if (avail == 0) {
    // Truly empty - only now re-enter buffering mode. Previously this
    // fired via _underrunThreshold == len (one packet), so ANY shortfall
    // below a single full frame - even one packet arriving a few ms late,
    // which happens constantly on a real network - forced a full 200ms
    // re-buffer instead of a single ~20ms silence-padded gap. That made
    // ordinary network jitter sound like constant choppiness. Now only a
    // fully-drained buffer re-triggers buffering; a partial shortfall just
    // plays what's real and pads the rest with silence for this one frame.
    _buffering = true;
    _underrunCount++;
    memset(out, 0xFF, len);
    _reportIfDue();
    return 0;
  }

  // Read data
  size_t toRead = (avail < len) ? avail : len;

  for (size_t i = 0; i < toRead; i++) {
    out[i] = _buffer[_tail];
    _tail = (_tail + 1) % JITTER_BUF_SIZE;
  }

  // If we requested more than available, fill rest with silence
  if (toRead < len) {
    _partialShortfallCount++;
    memset(out + toRead, 0xFF, len - toRead);
  }

  _reportIfDue();
  return len; // out is always fully populated (real data + any silence pad)
}

void JitterBuffer::_reportIfDue() {
  uint32_t now = millis();
  if (now - _lastReportTime < 5000) return;
  if (_lastReportTime != 0 && (_underrunCount > 0 || _partialShortfallCount > 0)) {
    Serial.printf("[JitterBuf] underruns(full rebuffer)=%lu partial(padded)=%lu since last report\r\n",
                  (unsigned long)_underrunCount, (unsigned long)_partialShortfallCount);
  }
  _lastReportTime = now;
  _underrunCount = 0;
  _partialShortfallCount = 0;
}

float JitterBuffer::getBufferUsage() {
  return (float)available() / (float)_targetLevel;
}
