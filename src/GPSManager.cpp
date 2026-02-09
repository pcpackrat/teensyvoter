#include "GPSManager.h"
#include <TimeLib.h> // Teensy Time library

GPSManager *GPSManager::_instance = nullptr;

GPSManager::GPSManager() {
  _gpsSerial = nullptr;
  _lastPpsMicros = 0;
  _ppsTriggered = false;
  _currentEpoch = 0;
  _validTime = false;
  _ppsPeriod = 1000000;
  _stableStatus = GPS_NO_FIX;
  _pendingStatus = GPS_NO_FIX;
  _pendingStartTime = 0;
  _lastLoggedStatus = GPS_NO_FIX;
  _instance = this;
}

void GPSManager::_ppsISR() {
  if (_instance) {
    _instance->_handlePPS();
  }
}

void GPSManager::_handlePPS() {
  uint32_t now = micros();
  uint32_t delta = now - _lastPpsMicros;

  // Simple debouncing: ignore if less than 900ms has passed
  if (delta > 900000) {
    _ppsPeriod = delta;

    // CRITICAL FIX: Increment epoch BEFORE updating _lastPpsMicros
    // This way, _currentEpoch represents the second that just started
    // and _lastPpsMicros marks the beginning of that second
    if (_validTime) {
      _currentEpoch++;
    }

    _lastPpsMicros = now;
    _ppsTriggered = true;
  }
}

void GPSManager::begin(Stream *serialPort, uint8_t ppsPin) {
  _gpsSerial = serialPort;
  _ppsPin = ppsPin;

  pinMode(_ppsPin, INPUT);
  // Trigger on RISING edge (standard for PPS)
  attachInterrupt(digitalPinToInterrupt(_ppsPin), _ppsISR, RISING);
}

void GPSManager::update() {
  uint32_t now = micros();
  // 1. Parse Serial Data
  while (_gpsSerial && _gpsSerial->available() > 0) {
    _gpsParser.encode(_gpsSerial->read());
  }

  // 2. Check for newly updated NMEA time
  if (_gpsParser.time.isUpdated() && _gpsParser.date.isUpdated()) {
    if (_gpsParser.location.isValid()) {
      // Convert TinyGPS constituents to Epoch
      tmElements_t tm;
      tm.Second = _gpsParser.time.second();
      tm.Minute = _gpsParser.time.minute();
      tm.Hour = _gpsParser.time.hour();
      tm.Day = _gpsParser.date.day();
      tm.Month = _gpsParser.date.month();
      tm.Year =
          _gpsParser.date.year() - 1970; // TimeLib expects offset from 1970

      // Note: NMEA usually comes a few hundred ms *after* PPS.
      // So this time likely belongs to the PPS that just happened.
      // We set the base _currentEpoch.
      // The PPS interrupt handles incrementing it for the *next* second.

      // Safety check: Don't jump backward massively if we are already locked
      time_t gpsTime = makeTime(tm);
      if (!_validTime || abs((int64_t)gpsTime - (int64_t)_currentEpoch) >= 1) {
        _currentEpoch = gpsTime;
        _validTime = true;

        // Sync Teensy RTC as well
        Teensy3Clock.set(gpsTime);
      }
    }
  }

  // 3. Track Status Transitions with 500ms Hysteresis
  GPSLockStatus currentRaw = _getRawLockStatus();

  if (currentRaw != _pendingStatus) {
    _pendingStatus = currentRaw;
    _pendingStartTime = now;
  } else {
    // Current raw matches pending - has it been stable for 500ms?
    if (millis() - _pendingStartTime > 500) {
      _stableStatus = currentRaw;
    }
  }

  if (_stableStatus != _lastLoggedStatus) {
    const char *oldStr = "UNKNOWN";
    const char *newStr = "UNKNOWN";

    auto statusToStr = [](GPSLockStatus s) {
      switch (s) {
      case GPS_NO_FIX:
        return "NO_FIX";
      case GPS_LOCKED:
        return "LOCKED";
      case GPS_LOST_PPS:
        return "LOST_PPS";
      case GPS_LOST_SERIAL:
        return "LOST_SERIAL";
      default:
        return "UNKNOWN";
      }
    };

    oldStr = statusToStr(_lastLoggedStatus);
    newStr = statusToStr(_stableStatus);

    char ts[20];
    getTimestamp(ts);

    Serial.printf("%s [GPS] Status Change: %s -> %s\r\n", ts, oldStr, newStr);

    // Provide specific context for why it dropped out
    uint32_t ppsAge = (now >= _lastPpsMicros) ? (now - _lastPpsMicros) : 0;
    if (_stableStatus == GPS_LOST_PPS) {
      Serial.printf(
          "%s [GPS] Context: PPS Timeout. Age: %u us (Limit: 5000000)\r\n", ts,
          ppsAge);
    } else if (_stableStatus == GPS_LOST_SERIAL) {
      uint32_t age = _gpsParser.time.age();
      Serial.printf(
          "%s [GPS] Context: Serial Timeout. Age: %u ms (Limit: 10000)\r\n", ts,
          age);
    } else if (_stableStatus == GPS_LOCKED) {
      if (_ppsPeriod < 5000000) {
        Serial.printf(
            "%s [GPS] Context: Fix Restored. Sats: %u, PPS Period: %u us\r\n",
            ts, getSatellites(), _ppsPeriod);
      } else {
        Serial.printf("%s [GPS] Context: Fix Restored. Sats: %u\r\n", ts,
                      getSatellites());
      }
    }

    _lastLoggedStatus = _stableStatus;
  }
}

void GPSManager::getTimestamp(char *buf) {
  if (!buf)
    return;

  VTIME vt;
  getNetworkTime(&vt, false); // Get raw high-res time

  time_t t = (time_t)vt.vtime_sec;
  tmElements_t tm;
  breakTime(t, tm);

  uint32_t millisPart = (vt.vtime_nsec / 1000000);

  snprintf(buf, 20, "[%02d:%02d:%02d.%03u]", tm.Hour, tm.Minute, tm.Second,
           millisPart);
}

bool GPSManager::isLocked() { return _stableStatus == GPS_LOCKED; }

GPSManager::GPSLockStatus GPSManager::getLockStatus() { return _stableStatus; }

GPSManager::GPSLockStatus GPSManager::_getRawLockStatus() {
  if (!_validTime) {
    return GPS_NO_FIX;
  }

  uint32_t now = micros();
  uint32_t ppsAge = (now >= _lastPpsMicros) ? (now - _lastPpsMicros) : 0;

  bool ppsActive = ppsAge < 5000000;
  if (!ppsActive) {
    return GPS_LOST_PPS;
  }

  bool serialActive = _gpsParser.time.age() < 10000;
  if (!serialActive) {
    return GPS_LOST_SERIAL;
  }

  // REQUIRE STABLE LOCK: At least 4 satellites for connection
  if (_gpsParser.satellites.value() < 4) {
    return GPS_NO_FIX;
  }

  return GPS_LOCKED;
}

bool GPSManager::isTimeSet() { return _validTime; }

uint32_t GPSManager::getPpsJitter() {
  if (_ppsPeriod > 1000000) {
    return _ppsPeriod - 1000000;
  } else {
    return 1000000 - _ppsPeriod;
  }
}

void GPSManager::getNetworkTime(VTIME *t, bool quantize) {
  if (!t)
    return;

  noInterrupts();
  uint32_t lastPps = _lastPpsMicros;
  uint32_t epoch = _currentEpoch;
  interrupts();

  uint32_t now = micros();
  uint32_t deltaMicros = now - lastPps;

  // If we are way past 1 second (lost PPS?), just rely on math
  if (deltaMicros >= 1000000) {
    // We are into the next second but interrupt hasn't fired or was missed
    // Handle gracefully by creating a virtual second
    uint32_t secondsOver = deltaMicros / 1000000;
    epoch += secondsOver;
    deltaMicros %= 1000000;
  }

  if (quantize) {
    // SNAP to nearest 20ms grid (20000 micros)
    // Add half-step (10000) for rounding
    uint32_t slots = (deltaMicros + 10000) / 20000;
    deltaMicros = slots * 20000;
  }

  // Handle potential overflow if rounding pushed us to 1000000
  if (deltaMicros >= 1000000) {
    epoch++;
    deltaMicros -= 1000000;
  }

  t->vtime_sec = epoch;
  t->vtime_nsec = deltaMicros * 1000; // Convert us to ns
}

uint32_t GPSManager::getEpoch() {
  noInterrupts();
  uint32_t e = _currentEpoch;
  interrupts();
  return e;
}

uint32_t GPSManager::getSatellites() { return _gpsParser.satellites.value(); }

void GPSManager::getGPSStrings(char *lat, char *lon, char *elev) {
  if (lat) {
    // Format: DDMM.mmN
    double rawLat = _gpsParser.location.lat();
    char ns = (rawLat >= 0) ? 'N' : 'S';
    rawLat = fabs(rawLat);
    int deg = (int)rawLat;
    if (deg > 90)
      deg = 90; // Clamp for safety/warnings
    double mins = (rawLat - deg) * 60.0;
    snprintf(lat, 9, "%02d%05.2f%c", deg, mins, ns);
  }
  if (lon) {
    // Format: DDDMM.mmW
    double rawLon = _gpsParser.location.lng();
    char ew = (rawLon >= 0) ? 'E' : 'W';
    rawLon = fabs(rawLon);
    int deg = (int)rawLon;
    if (deg > 180)
      deg = 180; // Clamp for safety/warnings
    double mins = (rawLon - deg) * 60.0;
    snprintf(lon, 10, "%03d%05.2f%c", deg, mins, ew);
  }
  if (elev) {
    // Format: EEEEE.e (Meters)
    double alt = _gpsParser.altitude.meters();
    snprintf(elev, 7, "%05.1f", alt);
  }
}
