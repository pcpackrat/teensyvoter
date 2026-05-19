#ifndef ETHERNET_DRIVER_H
#define ETHERNET_DRIVER_H

#include "NetworkDriver.h"
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>

class EthernetDriver : public NetworkDriver {
public:
  EthernetDriver();
  virtual ~EthernetDriver() {}

  // Init
  virtual bool begin(uint8_t *mac) override; // DHCP mode
  bool begin(uint8_t *mac, IPAddress ip, IPAddress subnet, IPAddress gateway,
             IPAddress dns); // Static IP mode
  virtual void update() override;

  // Status
  virtual bool isConnected() override;
  virtual IPAddress getLocalIP() override;
  virtual IPAddress getSubnetMask() override;
  virtual IPAddress getGateway() override;
  virtual IPAddress getDNS() override;
  virtual DriverType getType() override { return DRIVER_ETHERNET; }

  // Data
  virtual void setTarget(IPAddress ip, uint16_t port) override;
  virtual void sendPacket(const uint8_t *data, uint16_t len) override;
  virtual void sendPacketTo(IPAddress ip, uint16_t port, const uint8_t *data,
                            uint16_t len) override;
  virtual int parsePacket() override;
  virtual int read(uint8_t *buffer, size_t maxLen) override;

  // DNS Resolution
  virtual IPAddress resolveHostname(const char *hostname) override;

private:
  EthernetUDP _udp;
  IPAddress _targetIP;
  uint16_t _targetPort;
  bool _linkStatus;
  uint8_t *_mac;
};

#endif
