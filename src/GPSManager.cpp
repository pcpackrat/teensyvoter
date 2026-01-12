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
      if (!_validTime || abs((int64_t)gpsTime - (int64_t)_currentEpoch) > 2) {
        _currentEpoch = gpsTime;
        _validTime = true;

        // Sync Teensy RTC as well
        Teensy3Clock.set(gpsTime);
      }
    }
  }
}

bool GPSManager::isLocked() {
  // We consider it locked if we have valid time and PPS is active (seen
  // recently)
  bool ppsActive = (micros() - _lastPpsMicros) < 1100000;
  return _validTime && ppsActive;
}

bool GPSManager::isTimeSet() { return _validTime; }

uint32_t GPSManager::getPpsJitter() {
  if (_ppsPeriod > 1000000) {
    return _ppsPeriod - 1000000;
  } else {
    return 1000000 - _ppsPeriod;
  }
}

void GPSManager::getNetworkTime(VTIME *t) {
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

  t->vtime_sec = epoch;
  t->vtime_nsec = deltaMicros * 1000; // Convert us to ns
}

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
