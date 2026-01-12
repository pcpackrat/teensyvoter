#ifndef GPS_MANAGER_H
#define GPS_MANAGER_H

#include "VoterProtocol.h"
#include <Arduino.h>
#include <TinyGPSPlus.h>

class GPSManager {
public:
  GPSManager();

  // Init
  void begin(Stream *serialPort, uint8_t ppsPin);

  // Main loop update
  void update();

  // Status
  bool isLocked();  // True if GPS has fix and PPS is active
  bool isTimeSet(); // True if we have valid UTC time

  // Time Retrieval
  // Fill the VTIME struct with the exact current network time
  void getNetworkTime(VTIME *t);

  // Debugging / Tuning
  uint32_t getPpsJitter(); // Returns jitter in micros from last second

  // Location
  void getGPSStrings(char *lat, char *lon, char *elev);

private:
  Stream *_gpsSerial;
  TinyGPSPlus _gpsParser;
  uint8_t _ppsPin;

  // PPS State
  volatile uint32_t _lastPpsMicros;
  volatile bool _ppsTriggered;
  uint32_t _ppsPeriod; // Measured duration between PPS

  // Time State
  uint32_t _currentEpoch; // UTC Seconds
  bool _validTime;

  // Static ISR wrapper
  static void _ppsISR();
  static GPSManager *_instance; // Singleton pointer for ISR

  void _handlePPS();
};

#endif
