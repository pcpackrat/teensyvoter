#ifndef ESP_SPI_DRIVER_H
#define ESP_SPI_DRIVER_H

#include "NetworkDriver.h"
#include "SpiProtocol.h"
#include <Arduino.h>
#include <SPI.h>

class EspSpiDriver : public NetworkDriver {
public:
  EspSpiDriver(uint8_t csPin, uint8_t readyPin, uint8_t resetPin);

  // NetworkDriver Implementation
  bool begin(uint8_t *mac) override;
  void update() override;
  bool isConnected() override;
  IPAddress getLocalIP() override;
  IPAddress getSubnetMask() override;
  IPAddress getGateway() override;
  IPAddress getDNS() override;
  DriverType getType() override { return DRIVER_WIFI_SPI; }

  void setTarget(IPAddress ip, uint16_t port) override;
  void sendPacket(const uint8_t *data, uint16_t len) override;
  void sendPacketTo(IPAddress ip, uint16_t port, const uint8_t *data,
                    uint16_t len) override;
  int parsePacket() override;
  int read(uint8_t *buffer, size_t maxLen) override;

  // WiFi Specific
  void setCredentials(const char *ssid, const char *pass);
  IPAddress resolveHostname(const char *hostname) override;
  IPAddress getDNSServer();
  void setStaticDNS(IPAddress dns) { _staticDNS = dns; }

  // Config Management (for ESP32 web server)
  void pushConfig(const void *configData, uint16_t configLen);
  bool hasConfigCmd();
  int readConfigCmd(uint8_t *buffer, size_t maxLen);

private:
  uint8_t _cs, _ready, _reset;
  IPAddress _targetIP;
  uint16_t _targetPort;
  IPAddress _cachedIP; // IP Cache
  IPAddress _staticDNS;

  // Buffers
  uint8_t _rxBuffer[512];
  int _rxLen;

  // Config command buffer (from ESP32 web server)
  uint8_t _cfgBuffer[256];
  int _cfgLen;

  // SPI Helpers
  bool _waitReady(uint32_t timeoutMs);
  uint8_t _readStatus();
};

#endif
