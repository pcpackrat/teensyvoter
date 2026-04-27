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
  enum GPSLockStatus {
    GPS_NO_FIX,      // No valid time/location
    GPS_LOCKED,      // Time valid, PPS active, Serial active
    GPS_LOST_PPS,    // Time valid, but PPS missing (>1.1s)
    GPS_LOST_SERIAL, // Time valid, PPS active, but Serial missing (>2s)
  };

  bool isLocked(); // True if GPS_LOCKED
  GPSLockStatus getLockStatus();
  bool isTimeSet(); // True if we have valid UTC time
  bool checkPPS();  // Returns true if PPS fired since last check (clears flag)

  // Time Retrieval
  // Fill the VTIME struct with the exact current network time
  // quantize: If true, snaps timestamp to nearest 20ms grid relative to PPS
  // (removes jitter)
  void getNetworkTime(VTIME *t, bool quantize = true);

  // Return the current atomic epoch (safely reads _currentEpoch)
  uint32_t getEpoch();

  // Debugging / Tuning
  uint32_t getPpsJitter(); // Returns jitter in micros from last second

  // Location
  void getGPSStrings(char *lat, char *lon, char *elev);
  uint32_t getSatellites();
  void getTimestamp(char *buf); // Returns "[HH:MM:SS.mmm]"

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

  GPSLockStatus _stableStatus;
  GPSLockStatus _pendingStatus;
  uint32_t _pendingStartTime;
  GPSLockStatus _lastLoggedStatus;

  GPSLockStatus _getRawLockStatus();
  void _handlePPS();
};

#endif
