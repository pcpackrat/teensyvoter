#ifndef NETWORK_DRIVER_H
#define NETWORK_DRIVER_H

#include <Arduino.h>
#include <IPAddress.h>

enum DriverType {
  DRIVER_NONE,
  DRIVER_ESP32_SPI
};

class NetworkDriver {
public:
  virtual ~NetworkDriver() {}

  // Init
  virtual bool begin() = 0;
  virtual void update() = 0;

  // Status
  virtual bool isConnected() = 0;
  virtual IPAddress getLocalIP() = 0;
  virtual DriverType getType() = 0;

  // Data - Simplified for UDP Voter Protocol
  virtual void setTarget(IPAddress ip, uint16_t port) = 0;
  virtual void sendPacket(const uint8_t *data, uint16_t len) = 0;
  virtual void sendPacketTo(IPAddress ip, uint16_t port, const uint8_t *data,
                            uint16_t len) = 0;
  virtual int parsePacket() = 0;
  virtual int read(uint8_t *buffer, size_t maxLen) = 0;
  virtual IPAddress resolveHostname(const char *hostname) = 0;
  virtual void setYieldCallback(void (*cb)()) { _yieldCb = cb; }

protected:
  void (*_yieldCb)() = nullptr;
  void _yield() { if (_yieldCb) _yieldCb(); }
};

#endif
