#include "EspSpiDriver.h"
#include "SpiProtocol.h"

EspSpiDriver::EspSpiDriver(uint8_t csPin, uint8_t readyPin, uint8_t resetPin) {
  _cs = csPin;
  _ready = readyPin;
  _reset = resetPin;
  _rxLen = 0;
  _cfgLen = 0;
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

  // SPI Collision Evasion Maneuver: 
  // If the ESP32 is already asserting READY, a UDP packet is sitting identically in the DMA buffer.
  // If we initiate a TX command right now, the DMA will physically blast the UDP packet at us while we
  // clock out the TX command! To prevent silent packet loss, we explicitly read it first into _rxBuffer!
  if (digitalRead(_ready) == HIGH) {
      parsePacket();
  }

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

    if (status == STATUS_CONFIG_CMD) {
      // Config command from ESP32 web server — route to separate buffer
      uint8_t lenHi = SPI.transfer(0x00);
      uint8_t lenLo = SPI.transfer(0x00);
      uint16_t len = (lenHi << 8) | lenLo;

      if (len > 0 && len < sizeof(_cfgBuffer)) {
        for (int i = 0; i < len; i++) {
          _cfgBuffer[i] = SPI.transfer(0x00);
        }
        _cfgLen = len;
      }

      // Padding
      for (int k = 0; k < 4; k++) SPI.transfer(0x00);
      delayMicroseconds(50);

      digitalWrite(_cs, HIGH);
      SPI.endTransaction();
      return 0; // No UDP data — config command stored separately
    } else if (status & STATUS_HAS_DATA) {
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

  // Clear any stale buffer state
  _rxLen = 0;

  // --- Phase 1: Send CMD_GET_IP ---
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(_cs, LOW);
  SPI.transfer(CMD_GET_IP);
  for (int k = 0; k < 4; k++) {
    SPI.transfer(0x00);
    delayMicroseconds(5);
  }
  delayMicroseconds(50);
  digitalWrite(_cs, HIGH);
  SPI.endTransaction();

  // --- Phase 2: Wait for ESP32 to process and re-arm ---
  // The ESP32 needs time to: complete spi_slave_transmit, process CMD_GET_IP,
  // prepare the response in sendbuf, and re-arm spi_slave_transmit.
  delay(100);

  // --- Phase 3: Blind read (bypass READY pin) ---
  // Directly clock out the response. The ESP32's DMA buffer should contain
  // [STATUS] [LEN_HI] [LEN_LO] [IP0] [IP1] [IP2] [IP3]
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(_cs, LOW);

  uint8_t status = SPI.transfer(0x00);
  uint8_t lenHi  = SPI.transfer(0x00);
  uint8_t lenLo  = SPI.transfer(0x00);
  uint16_t len   = (lenHi << 8) | lenLo;

  uint8_t ip[4] = {0, 0, 0, 0};
  if ((status & STATUS_HAS_DATA) && len == 4) {
    for (int i = 0; i < 4; i++) {
      ip[i] = SPI.transfer(0x00);
    }
  }

  // Padding for consistency
  for (int k = 0; k < 4; k++) SPI.transfer(0x00);
  delayMicroseconds(50);

  digitalWrite(_cs, HIGH);
  SPI.endTransaction();



  IPAddress result(ip[0], ip[1], ip[2], ip[3]);
  if (result != IPAddress(0, 0, 0, 0)) {
    _cachedIP = result;
  }
  return result;
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

// --- Config Management (for ESP32 Web Server) ---

void EspSpiDriver::pushConfig(const void *configData, uint16_t configLen) {
  // Send the raw SysConfig struct to the ESP32 via CMD_PUSH_CONFIG
  // Format: [CMD_PUSH_CONFIG] [LEN_HI] [LEN_LO] [config bytes...]
  if (configLen > 500) return; // Safety check

  // Pace before starting a new transaction
  delay(5);

  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(_cs, LOW);
  delayMicroseconds(50);

  // Check status byte first to see if we're colliding with a UDP packet
  uint8_t status = SPI.transfer(CMD_PUSH_CONFIG);
  if (status & STATUS_HAS_DATA) {
      // Collision! ESP32 has data for us. Abort push to prevent corruption.
      digitalWrite(_cs, HIGH);
      SPI.endTransaction();
      Serial.println("[SPI] Config push aborted: UDP data pending on ESP32");
      return;
  }

  SPI.transfer((configLen >> 8) & 0xFF);
  SPI.transfer(configLen & 0xFF);

  const uint8_t *data = (const uint8_t *)configData;
  for (uint16_t i = 0; i < configLen; i++) {
    SPI.transfer(data[i]);
    // Pace the transfer to avoid overrunning the ESP32's DMA
    if (i % 64 == 63) delayMicroseconds(10);
  }

  // Padding
  for (int k = 0; k < 4; k++) SPI.transfer(0x00);
  delayMicroseconds(50);

  digitalWrite(_cs, HIGH);
  SPI.endTransaction();

  Serial.printf("[SPI] Pushed config to ESP32 (%d bytes)\r\n", configLen);
}

bool EspSpiDriver::hasConfigCmd() {
  return _cfgLen > 0;
}

int EspSpiDriver::readConfigCmd(uint8_t *buffer, size_t maxLen) {
  if (_cfgLen > 0) {
    size_t copyLen = (_cfgLen < (int)maxLen) ? _cfgLen : maxLen;
    memcpy(buffer, _cfgBuffer, copyLen);
    _cfgLen = 0; // Clear buffer
    return copyLen;
  }
  return 0;
}
