#ifndef ETHERNET_DRIVER_H
#define ETHERNET_DRIVER_H

#include "NetworkDriver.h"
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>

class EthernetDriver : public NetworkDriver {
public:
    bool begin(uint8_t* mac) override {
        if (Ethernet.begin(mac) == 0) return false;
        _udp.begin(0);
        return true;
    }
    
    void update() override {
        Ethernet.maintain();
    }

    bool isConnected() override {
        return Ethernet.linkStatus() != LinkOFF;
    }

    IPAddress getLocalIP() override {
        return Ethernet.localIP();
    }

    DriverType getType() override { return DRIVER_ETHERNET; }

    void setTarget(IPAddress ip, uint16_t port) override {
        _targetIP = ip;
        _targetPort = port;
    }

    void sendPacket(const uint8_t* data, uint16_t len) override {
        _udp.beginPacket(_targetIP, _targetPort);
        _udp.write(data, len);
        _udp.endPacket();
    }

    int parsePacket() override {
        return _udp.parsePacket();
    }

    int read(uint8_t* buffer, size_t maxLen) override {
        return _udp.read(buffer, maxLen);
    }

private:
    EthernetUDP _udp;
    IPAddress _targetIP;
    uint16_t _targetPort;
};

#endif
