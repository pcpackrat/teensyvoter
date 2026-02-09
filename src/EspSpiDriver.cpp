#include "EspSpiDriver.h"
#include "SpiProtocol.h"

EspSpiDriver::EspSpiDriver(uint8_t csPin, uint8_t readyPin, uint8_t resetPin) {
  _cs = csPin;
  _ready = readyPin;
  _reset = resetPin;
  _rxLen = 0;
  _cachedIP = IPAddress(0, 0, 0, 0);
}

bool EspSpiDriver::begin(uint8_t *mac) {
  pinMode(_cs, OUTPUT);
  digitalWrite(_cs, HIGH);

  pinMode(_reset, OUTPUT);
  digitalWrite(_reset, LOW);
  delay(100);
  digitalWrite(_reset, HIGH);

  pinMode(_ready, INPUT);

  SPI.begin();
  // Default to generic SPI speed
  return true;
}

void EspSpiDriver::update() {
  // Check READY pin from ESP if we aren't using interrupts
  // If HIGH, it means ESP has data for us.
  if (digitalRead(_ready) == HIGH) {
    // Prepare to read?
    // For simplicity, we just let parsePacket handle it when called.
    // Or we could buffer here.
  }
}

void EspSpiDriver::sendPacket(const uint8_t *data, uint16_t len) {
  // Format: [CMD] [LEN_HI] [LEN_LO] [IP...4] [PORT...2] [DATA...]

  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
  digitalWrite(_cs, LOW);

  // Header
  uint8_t header[9];
  header[0] = CMD_SEND_UDP;
  header[1] = (len >> 8) & 0xFF;
  header[2] = (len & 0xFF);
  header[3] = _targetIP[0];
  header[4] = _targetIP[1];
  header[5] = _targetIP[2];
  header[6] = _targetIP[3];
  header[7] = (_targetPort >> 8) & 0xFF;
  header[8] = (_targetPort & 0xFF);

  SPI.transfer(header, 9);

  // Data
  for (uint16_t i = 0; i < len; i++) {
    SPI.transfer(data[i]);
  }

  // Padding bytes to ensure FIFO flushes and last byte is latched (Fix for
  // dropped bytes)
  for (int k = 0; k < 4; k++) {
    SPI.transfer(0x00);
    delayMicroseconds(5);
  }

  // Wait before CS HIGH
  delayMicroseconds(50);

  digitalWrite(_cs, HIGH);
  SPI.endTransaction();
}

void EspSpiDriver::setCredentials(const char *ssid, const char *pass) {
  uint8_t ssidLen = strlen(ssid);
  uint8_t passLen = strlen(pass);

  SPI.beginTransaction(
      SPISettings(1000000, MSBFIRST, SPI_MODE0)); // Slow for config
  digitalWrite(_cs, LOW);

  SPI.transfer(CMD_SET_CONFIG);
  SPI.transfer(ssidLen);
  for (int i = 0; i < ssidLen; i++)
    SPI.transfer(ssid[i]);

  SPI.transfer(passLen);
  for (int i = 0; i < passLen; i++) {
    SPI.transfer(pass[i]);
    delayMicroseconds(5);
  }

  // Padding bytes to ensure FIFO flushes and last byte is latched
  for (int k = 0; k < 4; k++) {
    SPI.transfer(0x00);
    delayMicroseconds(5);
  }

  // Wait before CS HIGH to ensure physical transmission matches FIFO
  delayMicroseconds(50);

  digitalWrite(_cs, HIGH);
  SPI.endTransaction();
}

IPAddress EspSpiDriver::resolveHostname(const char *hostname) {
  if (!hostname || hostname[0] == '\0') {
    return IPAddress(0, 0, 0, 0); // Invalid hostname
  }

  uint8_t hostnameLen = strlen(hostname);
  if (hostnameLen > 63) {
    Serial.println("[DNS] Hostname too long");
    return IPAddress(0, 0, 0, 0);
  }

  Serial.printf("[DNS] Resolving: %s\n", hostname);

  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(_cs, LOW);

  // Send DNS lookup command
  SPI.transfer(CMD_DNS_LOOKUP);
  SPI.transfer(hostnameLen);
  for (int i = 0; i < hostnameLen; i++) {
    SPI.transfer(hostname[i]);
  }

  // Padding to flush FIFO
  for (int k = 0; k < 4; k++) {
    SPI.transfer(0x00);
    delayMicroseconds(5);
  }

  delayMicroseconds(50);
  digitalWrite(_cs, HIGH);
  SPI.endTransaction();

  // Wait for ESP32 to perform DNS lookup (can take 1-5 seconds)
  Serial.println("[DNS] Waiting for response...");
  uint32_t startTime = millis();
  while (millis() - startTime < 10000) { // 10 second timeout
    if (digitalRead(_ready) == HIGH) {
      delayMicroseconds(500);

      SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
      digitalWrite(_cs, LOW);

      uint8_t status = SPI.transfer(0x00);
      if (status & STATUS_HAS_DATA) {
        uint8_t lenHi = SPI.transfer(0x00);
        uint8_t lenLo = SPI.transfer(0x00);
        uint16_t len = (lenHi << 8) | lenLo;

        if (len == 4) { // IP address is 4 bytes
          uint8_t ip[4];
          for (int i = 0; i < 4; i++) {
            ip[i] = SPI.transfer(0x00);
          }

          digitalWrite(_cs, HIGH);
          SPI.endTransaction();

          IPAddress result(ip[0], ip[1], ip[2], ip[3]);
          if (result == IPAddress(0, 0, 0, 0)) {
            Serial.println("[DNS] Resolution failed (0.0.0.0)");
          } else {
            Serial.printf("[DNS] Resolved to: %d.%d.%d.%d\n", ip[0], ip[1],
                          ip[2], ip[3]);
          }
          return result;
        }
      }

      digitalWrite(_cs, HIGH);
      SPI.endTransaction();
    }
    delay(100); // Poll every 100ms
  }

  Serial.println("[DNS] Timeout");
  return IPAddress(0, 0, 0, 0);
}

IPAddress EspSpiDriver::getDNSServer() {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(_cs, LOW);

  SPI.transfer(CMD_GET_DNS);

  // Padding to flush FIFO
  for (int k = 0; k < 4; k++) {
    SPI.transfer(0x00);
    delayMicroseconds(5);
  }

  delayMicroseconds(50);
  digitalWrite(_cs, HIGH);
  SPI.endTransaction();

  // Wait for ESP32 response
  uint32_t startTime = millis();
  while (millis() - startTime < 2000) { // 2 second timeout
    if (digitalRead(_ready) == HIGH) {
      delayMicroseconds(500);

      SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
      digitalWrite(_cs, LOW);

      uint8_t status = SPI.transfer(0x00);
      if (status & STATUS_HAS_DATA) {
        uint8_t lenHi = SPI.transfer(0x00);
        uint8_t lenLo = SPI.transfer(0x00);
        uint16_t len = (lenHi << 8) | lenLo;

        if (len == 4) { // IP address is 4 bytes
          uint8_t ip[4];
          for (int i = 0; i < 4; i++) {
            ip[i] = SPI.transfer(0x00);
          }

          digitalWrite(_cs, HIGH);
          SPI.endTransaction();

          IPAddress result(ip[0], ip[1], ip[2], ip[3]);
          Serial.printf("[DNS] Server: %d.%d.%d.%d\n", ip[0], ip[1], ip[2],
                        ip[3]);
          return result;
        }
      }

      digitalWrite(_cs, HIGH);
      SPI.endTransaction();
    }
    delay(50);
  }

  Serial.println("[DNS] Timeout getting DNS server");
  return IPAddress(0, 0, 0, 0);
}

int EspSpiDriver::parsePacket() {
  // If ESP says "Ready", we read.
  if (digitalRead(_ready) == HIGH) {
    // Race Condition Fix: Give ESP32 time to enter spi_slave_transmit loop
    // after setting READY pin high.
    delayMicroseconds(500);

    // Use 1MHz for consistency with Config
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    digitalWrite(_cs, LOW);

    // We expect ESP to clock out: [STATUS] [LEN_HI] [LEN_LO] [DATA...]
    // Since ESP is Slave, we must send Dummy bytes to shift data in.

    uint8_t status = SPI.transfer(0x00);

    if (status & STATUS_HAS_DATA) {
      uint8_t lenHi = SPI.transfer(0x00);
      uint8_t lenLo = SPI.transfer(0x00);
      uint16_t len = (lenHi << 8) | lenLo;

      if (len > 0 && len < 512) {
        // Read Payload
        for (int i = 0; i < len; i++) {
          _rxBuffer[i] = SPI.transfer(0x00);
        }
        _rxLen = len;
      } else {
        // If len is 0 or invalid, this is weird.
        Serial.printf("[SPI Debug] Invalid Packet Len=%d (Status=0x%02X)\r\n",
                      len, status);
        _rxLen = 0;
      }
    } else {

      _rxLen = 0;
    }

    // Padding for Read (Consistency)
    for (int k = 0; k < 4; k++)
      SPI.transfer(0x00);
    delayMicroseconds(50);

    digitalWrite(_cs, HIGH);
    SPI.endTransaction();

    return _rxLen;
  }
  return 0;
}

int EspSpiDriver::read(uint8_t *buffer, size_t maxLen) {
  if (_rxLen > 0) {
    size_t copyLen = (_rxLen < (int)maxLen) ? _rxLen : maxLen;
    memcpy(buffer, _rxBuffer, copyLen);
    _rxLen = 0; // Clear buffer
    return copyLen;
  }
  return 0;
}

// Stubs
bool EspSpiDriver::isConnected() { return true; }

IPAddress EspSpiDriver::getLocalIP() {
  // Return Cached IP if valid (not 0.0.0.0)
  if (_cachedIP != IPAddress(0, 0, 0, 0)) {
    return _cachedIP;
  }

  // Manual Command Send
  // Match setCredentials speed/padding
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(_cs, LOW);
  SPI.transfer(CMD_GET_IP);

  // Padding bytes to ensure ESP32 sees the command
  for (int k = 0; k < 4; k++) {
    SPI.transfer(0x00);
    delayMicroseconds(5);
  }
  delayMicroseconds(50);

  digitalWrite(_cs, HIGH);
  SPI.endTransaction();

  // Wait for Response (Poll Ready) with Queue Draining
  uint32_t start = millis();
  while (millis() - start < 500) { // 500ms Timeout to drain queue
    if (digitalRead(_ready) == HIGH) {
      // Serial.println("[SPI] Ready Pin HIGH. Parsing...");

      // Read whatever is in the buffer
      int len = parsePacket();

      if (len == 4) {
        _cachedIP =
            IPAddress(_rxBuffer[0], _rxBuffer[1], _rxBuffer[2], _rxBuffer[3]);
        return _cachedIP;
      } else {
        // Silently retry during WiFi connection
        // Serial.printf("[SPI Debug] IP Rx Fail. Len=%d\r\n", len);
      }
    }
    delay(5);
  }
  return IPAddress(0, 0, 0, 0);
}

IPAddress EspSpiDriver::getSubnetMask() {
  return IPAddress(0, 0, 0, 0); // Not supported over SPI yet
}

IPAddress EspSpiDriver::getGateway() {
  return IPAddress(0, 0, 0, 0); // Not supported over SPI yet
}

IPAddress EspSpiDriver::getDNS() { return getDNSServer(); }

void EspSpiDriver::setTarget(IPAddress ip, uint16_t port) {
  _targetIP = ip;
  _targetPort = port;
}
