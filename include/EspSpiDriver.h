#ifndef ESP_SPI_DRIVER_H
#define ESP_SPI_DRIVER_H

#include <Arduino.h>
#include <SPI.h>
#include "NetworkDriver.h"

// Simple Protocol Constants
#define SPI_CMD_SEND_UDP 0x10
#define SPI_CMD_SET_SSID 0x20
#define SPI_CMD_GET_STATUS 0x30

class EspSpiDriver : public NetworkDriver {
public:
    EspSpiDriver(uint8_t csPin, uint8_t readyPin, uint8_t resetPin);
    
    // NetworkDriver Implementation
    bool begin(uint8_t* mac) override;
    void update() override;
    bool isConnected() override;
    IPAddress getLocalIP() override;
    DriverType getType() override { return DRIVER_WIFI_SPI; }

    void setTarget(IPAddress ip, uint16_t port) override;
    void sendPacket(const uint8_t* data, uint16_t len) override;
    int parsePacket() override;
    int read(uint8_t* buffer, size_t maxLen) override;

    // WiFi Specific
    void setCredentials(const char* ssid, const char* pass);

private:
    uint8_t _cs, _ready, _reset;
    IPAddress _targetIP;
    uint16_t _targetPort;
    
    // Buffers
    uint8_t _rxBuffer[512];
    int _rxLen;

    // SPI Helpers
    void _spiTransfer(uint8_t* data, size_t len);
    uint8_t _readStatus();
};

#endif
