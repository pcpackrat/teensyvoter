#ifndef NETWORK_MANAGER_H
#define NETWORK_MANAGER_H

#include "NetworkDriver.h"
#include <Arduino.h>
// #include "EspSpiDriver.h" // We'll include concrete types in main

class NetworkManager {
public:
  NetworkManager();

  // Init
  void begin(NetworkDriver *driver, uint8_t *mac_addr);

  // Passthrough
  void update();
  bool isConnected();
  IPAddress getLocalIP();
  IPAddress getSubnetMask();
  IPAddress getGateway();
  IPAddress getDNS();
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
  uint8_t *_mac;
};

#endif
