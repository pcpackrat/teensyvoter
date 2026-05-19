#include "EspSpiDriver.h"
#include "SpiProtocol.h"

EspSpiDriver::EspSpiDriver(uint8_t csPin, uint8_t readyPin, uint8_t resetPin) {
  _cs = csPin;
  _ready = readyPin;
  _reset = resetPin;
  _rxLen = 0;
  _cfgLen = 0;
  _cachedIP = IPAddress(0, 0, 0, 0);
  _staticDNS = IPAddress(0, 0, 0, 0);
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
  return true;
}

void EspSpiDriver::update() {
  // Stub - parsePacket handles reads.
}

void EspSpiDriver::sendPacket(const uint8_t *data, uint16_t len) {
  sendPacketTo(_targetIP, _targetPort, data, len);
}

void EspSpiDriver::sendPacketTo(IPAddress ip, uint16_t port, const uint8_t *data, uint16_t len) {
  static bool isSending = false;
  if (isSending) return; // Prevent recursive calls during SPI transactions
  isSending = true;

  // Collision evasion: Read pending data if READY is HIGH
  if (digitalRead(_ready) == HIGH) {
    parsePacket();
  }
  // Give the ESP32 more time to re-arm its SPI slave driver between transactions
  delayMicroseconds(2000); 

  // 4MHz is the maximum stable speed for long-wire SPI slave on ESP32
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
  digitalWrite(_cs, LOW);
  delayMicroseconds(100); // More time for the ESP32 to detect the CS edge

  // Buffer for the entire 512-byte transaction to use efficient block transfer
  uint8_t buffer[MAX_SPI_BUF];
  memset(buffer, 0, MAX_SPI_BUF);

  // Header: [CMD] [LEN_HI] [LEN_LO] [IP...4] [PORT...2]
  buffer[0] = CMD_SEND_UDP;
  buffer[1] = (len >> 8) & 0xFF;
  buffer[2] = len & 0xFF;
  for (int i = 0; i < 4; i++) buffer[3 + i] = ip[i];
  buffer[7] = (port >> 8) & 0xFF;
  buffer[8] = port & 0xFF;

  // Payload
  uint16_t payloadLen = (len > (MAX_SPI_BUF - 9)) ? (MAX_SPI_BUF - 9) : len;
  memcpy(&buffer[9], data, payloadLen);

  // Efficient block transfer
  SPI.transfer(buffer, MAX_SPI_BUF);

  digitalWrite(_cs, HIGH);
  SPI.endTransaction();
  isSending = false; // Reset reentrancy guard
}

void EspSpiDriver::setCredentials(const char *ssid, const char *pass) {
  uint8_t ssidLen = strlen(ssid);
  uint8_t passLen = strlen(pass);

  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(_cs, LOW);
  delayMicroseconds(50);

  SPI.transfer(CMD_SET_CONFIG);
  SPI.transfer(ssidLen);
  for (int i = 0; i < ssidLen; i++) SPI.transfer(ssid[i]);
  
  SPI.transfer(passLen);
  for (int i = 0; i < passLen; i++) SPI.transfer(pass[i]);

  // Pad to 512
  int sent = 3 + ssidLen + passLen;
  while (sent < MAX_SPI_BUF) {
    SPI.transfer(0x00);
    sent++;
  }

  digitalWrite(_cs, HIGH);
  SPI.endTransaction();
}

IPAddress EspSpiDriver::resolveHostname(const char *hostname) {
  if (!hostname || hostname[0] == '\0') return IPAddress(0, 0, 0, 0);
  uint8_t hLen = strlen(hostname);

  // Aggressively clear any pending data
  for (int i = 0; i < 3; i++) {
    if (digitalRead(_ready) == HIGH) {
      parsePacket();
      delay(50);
    }
  }
  
  delay(200); // Settle time

  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(_cs, LOW);
  delayMicroseconds(50);

  SPI.transfer(CMD_DNS_LOOKUP);
  SPI.transfer(hLen);
  for (int i = 0; i < hLen; i++) SPI.transfer(hostname[i]);

  // Pad to 512
  for (int i = hLen + 2; i < MAX_SPI_BUF; i++) SPI.transfer(0x00);

  digitalWrite(_cs, HIGH);
  SPI.endTransaction();

  // Wait for resolution (up to 10s)
  uint32_t start = millis();
  while (millis() - start < 10000) {
    if (digitalRead(_ready) == HIGH) {
      delayMicroseconds(500);
      SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
      digitalWrite(_cs, LOW);
      delayMicroseconds(50);

      uint8_t status = SPI.transfer(0x00);
      uint8_t lenHi  = SPI.transfer(0x00);
      uint8_t lenLo  = SPI.transfer(0x00);
      uint16_t len   = (lenHi << 8) | lenLo;

      uint8_t ip[4] = {0, 0, 0, 0};
      if ((status & STATUS_HAS_DATA) && len == 4) {
        for (int i = 0; i < 4; i++) ip[i] = SPI.transfer(0x00);
      }

      // Finish the 512 byte transfer
      int consumed = 3 + ((status & STATUS_HAS_DATA) ? 4 : 0);
      while (consumed < MAX_SPI_BUF) {
          SPI.transfer(0x00);
          consumed++;
      }

      digitalWrite(_cs, HIGH);
      SPI.endTransaction();
      
      if (status & STATUS_HAS_DATA) {
        if (len == 4) {
          IPAddress result(ip[0], ip[1], ip[2], ip[3]);
          Serial.printf("[DNS] SPI RX: %d.%d.%d.%d (Status: 0x%02X, Len: %d)\r\n", 
                        ip[0], ip[1], ip[2], ip[3], status, len);
          
          // CRITICAL: If the result matches our local IP, it's almost certainly stale data
          // from a previous getLocalIP() call that was still in the ESP32's buffer.
          if (result != IPAddress(0,0,0,0) && result != _cachedIP) {
            return result;
          } else if (result == _cachedIP) {
            Serial.println("[DNS] Ignoring stale Local IP in SPI buffer, waiting for real result...");
          }
        } else {
          Serial.printf("[DNS] SPI RX Other: Status 0x%02X, Len %d\r\n", status, len);
        }
      }
    }
    delay(100);
  }

  return IPAddress(0, 0, 0, 0);
}

IPAddress EspSpiDriver::getDNSServer() {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(_cs, LOW);
  delayMicroseconds(50);

  SPI.transfer(CMD_GET_DNS);
  for (int k = 1; k < MAX_SPI_BUF; k++) SPI.transfer(0x00);

  digitalWrite(_cs, HIGH);
  SPI.endTransaction();

  delay(100);

  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(_cs, LOW);
  delayMicroseconds(50);

  uint8_t status = SPI.transfer(0x00);
  uint8_t lenHi  = SPI.transfer(0x00);
  uint8_t lenLo  = SPI.transfer(0x00);
  uint16_t len   = (lenHi << 8) | lenLo;

  uint8_t ip[4] = {0, 0, 0, 0};
  if ((status & STATUS_HAS_DATA) && len == 4) {
    for (int i = 0; i < 4; i++) ip[i] = SPI.transfer(0x00);
  }

  digitalWrite(_cs, HIGH);
  SPI.endTransaction();

  return IPAddress(ip[0], ip[1], ip[2], ip[3]);
}

int EspSpiDriver::parsePacket() {
  if (digitalRead(_ready) == HIGH) {
    delayMicroseconds(500);

    SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
    digitalWrite(_cs, LOW);
    delayMicroseconds(50);

    uint8_t status = SPI.transfer(0x00);

    if (status == STATUS_CONFIG_CMD) {
      uint8_t lenHi = SPI.transfer(0x00);
      uint8_t lenLo = SPI.transfer(0x00);
      uint16_t len = (lenHi << 8) | lenLo;

      if (len > 0 && len < sizeof(_cfgBuffer)) {
        for (int i = 0; i < len; i++) {
          _cfgBuffer[i] = SPI.transfer(0x00);
        }
        _cfgLen = len;
      }
      // Pad to 512
      int consumed = 3 + (len < sizeof(_cfgBuffer) ? len : 0);
      while (consumed < MAX_SPI_BUF) {
          SPI.transfer(0x00);
          consumed++;
      }
      digitalWrite(_cs, HIGH);
      SPI.endTransaction();
      return 0;
    } else if (status & STATUS_HAS_DATA) {
      uint8_t lenHi = SPI.transfer(0x00);
      uint8_t lenLo = SPI.transfer(0x00);
      uint16_t len = (lenHi << 8) | lenLo;

      if (len > 0 && len < 512) {
        for (int i = 0; i < len; i++) {
          _rxBuffer[i] = SPI.transfer(0x00);
        }
        _rxLen = len;
      } else {
        _rxLen = 0;
      }

      // Pad to 512
      int consumed = 3 + (len > 0 && len < 512 ? len : 0);
      while (consumed < MAX_SPI_BUF) {
          SPI.transfer(0x00);
          consumed++;
      }
      digitalWrite(_cs, HIGH);
      SPI.endTransaction();
      return _rxLen;
    } else {
      // Pad to 512 for idle status
      for (int k = 1; k < MAX_SPI_BUF; k++) SPI.transfer(0x00);
      _rxLen = 0;
    }

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
    _rxLen = 0;
    return copyLen;
  }
  return 0;
}

bool EspSpiDriver::isConnected() { return true; }

IPAddress EspSpiDriver::getLocalIP() {
  if (_cachedIP != IPAddress(0, 0, 0, 0)) return _cachedIP;

  // Clear any pending unsolicited data before sending a command
  while (digitalRead(_ready) == HIGH) {
    parsePacket();
    delay(5);
  }

  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(_cs, LOW);
  delayMicroseconds(50);

  SPI.transfer(CMD_GET_IP);
  for (int k = 1; k < MAX_SPI_BUF; k++) SPI.transfer(0x00);
  digitalWrite(_cs, HIGH);
  SPI.endTransaction();

  // Master uses 100ms delay for IP retrieval
  delay(100);

  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(_cs, LOW);
  delayMicroseconds(50);

  uint8_t status = SPI.transfer(0x00);
  uint8_t lenHi  = SPI.transfer(0x00);
  uint8_t lenLo  = SPI.transfer(0x00);
  uint16_t len   = (lenHi << 8) | lenLo;

  uint8_t ip[4] = {0, 0, 0, 0};
  if ((status & STATUS_HAS_DATA) && len == 4) {
    for (int i = 0; i < 4; i++) ip[i] = SPI.transfer(0x00);
  }

  // Pad to 512
  int consumed = 3 + (len == 4 ? 4 : 0);
  while (consumed < MAX_SPI_BUF) {
      SPI.transfer(0x00);
      consumed++;
  }

  digitalWrite(_cs, HIGH);
  SPI.endTransaction();

  IPAddress result(ip[0], ip[1], ip[2], ip[3]);
  if (result != IPAddress(0, 0, 0, 0)) _cachedIP = result;
  return result;
}

IPAddress EspSpiDriver::getSubnetMask() { return IPAddress(0, 0, 0, 0); }
IPAddress EspSpiDriver::getGateway() { return IPAddress(0, 0, 0, 0); }
IPAddress EspSpiDriver::getDNS() { return getDNSServer(); }

void EspSpiDriver::setTarget(IPAddress ip, uint16_t port) {
  _targetIP = ip;
  _targetPort = port;
}

void EspSpiDriver::pushConfig(const void *configData, uint16_t configLen) {
  if (configLen > 500) return;
  delay(5);

  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(_cs, LOW);
  delayMicroseconds(50);

  uint8_t status = SPI.transfer(CMD_PUSH_CONFIG);
  if (status & STATUS_HAS_DATA) {
      digitalWrite(_cs, HIGH);
      SPI.endTransaction();
      return;
  }

  SPI.transfer((configLen >> 8) & 0xFF);
  SPI.transfer(configLen & 0xFF);

  const uint8_t *data = (const uint8_t *)configData;
  for (uint16_t i = 0; i < configLen; i++) {
    SPI.transfer(data[i]);
  }

  // Pad to 512
  int sent = 3 + configLen;
  while (sent < MAX_SPI_BUF) {
      SPI.transfer(0x00);
      sent++;
  }

  digitalWrite(_cs, HIGH);
  SPI.endTransaction();
}

bool EspSpiDriver::hasConfigCmd() { return _cfgLen > 0; }

int EspSpiDriver::readConfigCmd(uint8_t *buffer, size_t maxLen) {
  if (_cfgLen > 0) {
    size_t copyLen = (_cfgLen < (int)maxLen) ? _cfgLen : maxLen;
    memcpy(buffer, _cfgBuffer, copyLen);
    _cfgLen = 0;
    return copyLen;
  }
  return 0;
}
