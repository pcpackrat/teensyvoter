#include "EspSpiDriver.h"
#include "SpiProtocol.h"

EspSpiDriver::EspSpiDriver(uint8_t csPin, uint8_t readyPin, uint8_t resetPin) {
  _cs = csPin;
  _ready = readyPin;
  _reset = resetPin;
  _rxLen = 0;
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

  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
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

int EspSpiDriver::parsePacket() {
  // If ESP says "Ready", we read.
  if (digitalRead(_ready) == HIGH) {
    SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
    digitalWrite(_cs, LOW);

    // We expect ESP to clock out: [STATUS] [LEN_HI] [LEN_LO] [DATA...]
    // Since ESP is Slave, we must send Dummy bytes to shift data in.

    uint8_t status = SPI.transfer(0x00);

    if (status == STATUS_HAS_DATA) {
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
        _rxLen = 0;
      }
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
    _rxLen = 0; // Clear buffer
    return copyLen;
  }
  return 0;
}

// Stubs
bool EspSpiDriver::isConnected() { return true; }

IPAddress EspSpiDriver::getLocalIP() {
  // Send CMD_GET_IP
  uint8_t cmd[1] = {CMD_GET_IP};
  sendPacket(cmd, 0); // Need to handle 0 length or just send 1 byte command?
  // sendPacket assumes UDP packet structure which adds headers.
  // We need a raw "sendCommand" or just manually do it here.

  // Manual Command Send
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(_cs, LOW);
  SPI.transfer(CMD_GET_IP);
  digitalWrite(_cs, HIGH);
  SPI.endTransaction();

  // Wait for Response (Poll Ready)
  uint32_t start = millis();
  while (millis() - start < 200) { // 200ms Timeout
    if (digitalRead(_ready) == HIGH) {
      // Read it!
      int len = parsePacket(); // This reads into _rxBuffer
      if (len == 4) {
        return IPAddress(_rxBuffer[0], _rxBuffer[1], _rxBuffer[2],
                         _rxBuffer[3]);
      }
    }
  }

  return IPAddress(0, 0, 0, 0);
}
void EspSpiDriver::setTarget(IPAddress ip, uint16_t port) {
  _targetIP = ip;
  _targetPort = port;
}
