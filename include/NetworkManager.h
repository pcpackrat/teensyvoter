#ifndef NETWORK_MANAGER_H
#define NETWORK_MANAGER_H

#include "NetworkDriver.h"
#include <Arduino.h>

class NetworkManager {
public:
  NetworkManager();

  // Init
  void begin(NetworkDriver *driver);

  // Passthrough
  void update();
  bool isConnected();
  IPAddress getLocalIP();
  DriverType getType();

  void setYieldCallback(void (*cb)()) { if(_driver) _driver->setYieldCallback(cb); }

  // Voter Protocol specific
  void setTarget(IPAddress ip, uint16_t port);
  void sendPacket(const uint8_t *data, uint16_t length);
  void sendPacketTo(IPAddress ip, uint16_t port, const uint8_t *data,
                    uint16_t length);
  int parsePacket();
  int read(uint8_t *buffer, size_t maxLen);
  IPAddress resolveHostname(const char *hostname);

private:
  NetworkDriver *_driver;
};

#endif
