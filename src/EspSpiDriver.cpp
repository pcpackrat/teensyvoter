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
  _pollAsyncTxn();
  _reportRxStats();
}

void EspSpiDriver::_reportRxStats() {
  uint32_t now = millis();
  if (_rxStatsWindowStart == 0) {
    _rxStatsWindowStart = now;
    return;
  }
  if (now - _rxStatsWindowStart < 5000) return;

  uint32_t windowMs = now - _rxStatsWindowStart;
  float avgUs = _rxCallCount ? (float)_rxTotalUs / _rxCallCount : 0;
  Serial.printf(
      "[RX-STATS] window=%lums blockingCalls=%lu avgUs=%.0f totalBlockedUs=%lu\r\n",
      windowMs, _rxCallCount, avgUs, _rxTotalUs);

  _rxStatsWindowStart = now;
  _rxCallCount = 0;
  _rxTotalUs = 0;
}

void EspSpiDriver::sendPacket(const uint8_t *data, uint16_t len) {
  sendPacketTo(_targetIP, _targetPort, data, len);
}

void EspSpiDriver::_pollAsyncTxn() {
  if (_txnInFlight == SpiTxnType::NONE) return;
  if (_spiEvent) { // EventResponder::operator bool() - true once DMA completes
    digitalWrite(_cs, HIGH);
    SPI.endTransaction();
    _spiEvent.clearEvent();
    _txnInFlight = SpiTxnType::NONE;
  }
}

void EspSpiDriver::sendPacketTo(IPAddress ip, uint16_t port, const uint8_t *data, uint16_t len) {
  // Root cause of an audio-path buzz, found via isolated A/B testing
  // against a proven-silent pre-Phase-0 baseline (see
  // docs/esp32_network_migration_plan.md): it was the transfer-length
  // right-sizing, not blocking-vs-async on either the Teensy or ESP32
  // side, not the reduced delay, not the clock speed. Best explanation:
  // the original full MAX_SPI_BUF transfer is mostly zero-padding after
  // the real ~194-byte payload, so MOSI sits idle (constant) for most of
  // the transfer while only SCLK keeps ticking. Right-sizing eliminated
  // that idle tail, making every byte of the (shorter) transfer real,
  // actively-toggling data - apparently higher bit-transition density
  // despite the shorter total duration, which is what coupled audibly
  // into the audio path. Fix: always transfer the full MAX_SPI_BUF again
  // (restoring the zero-padding), keeping everything else that was
  // proven innocent during the bisection: async DMA transfer (CPU freed
  // during the ~2ms full-length transfer instead of blocked solid),
  // 200us pad (down from the original 2000us), and 2MHz clock.
  //
  // If a previous async send hasn't completed yet, finish it first
  // rather than silently dropping this one - should be rare in practice
  // but a bounded wait beats a dropped audio frame. The 5ms safety valve
  // aborts a stuck transaction rather than hanging the audio pipeline
  // indefinitely.
  uint32_t waitStart = micros();
  while (_txnInFlight != SpiTxnType::NONE) {
    _pollAsyncTxn();
    if (_txnInFlight != SpiTxnType::NONE && (micros() - waitStart > 5000)) {
      digitalWrite(_cs, HIGH);
      SPI.endTransaction();
      _spiEvent.clearEvent();
      _txnInFlight = SpiTxnType::NONE;
      break;
    }
  }

  delayMicroseconds(200);

  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
  digitalWrite(_cs, LOW);
  delayMicroseconds(100); // Time for the ESP32 to detect the CS edge

  memset(_txBuffer, 0, MAX_SPI_BUF);

  // Header: [CMD] [LEN_HI] [LEN_LO] [IP...4] [PORT...2]
  _txBuffer[0] = CMD_SEND_UDP;
  _txBuffer[1] = (len >> 8) & 0xFF;
  _txBuffer[2] = len & 0xFF;
  for (int i = 0; i < 4; i++) _txBuffer[3 + i] = ip[i];
  _txBuffer[7] = (port >> 8) & 0xFF;
  _txBuffer[8] = port & 0xFF;

  // Payload
  uint16_t payloadLen = (len > (MAX_SPI_BUF - 9)) ? (MAX_SPI_BUF - 9) : len;
  memcpy(&_txBuffer[9], data, payloadLen);

  // Always the full buffer now - see the note at the top of this
  // function for why. This also means the ESP32 always gets a
  // full-length transaction regardless of whether it had anything
  // queued for us, so there's no truncation/data-loss risk from that
  // angle either - but we still drain via a real parsePacket() call
  // below when READY is high, since this async send's own RX side
  // (_txRxScratch) is discarded/never parsed, so the bytes would
  // otherwise never reach _rxBuffer/_handlePacket().
  uint16_t transferLen = MAX_SPI_BUF;

  if (digitalRead(_ready) == HIGH) {
    parsePacket();
  }

  _spiEvent.clearEvent();
  bool queued = SPI.transfer(_txBuffer, _txRxScratch, transferLen, _spiEvent);
  if (queued) {
    _txnInFlight = SpiTxnType::SEND;
    // CS/endTransaction happen later in _pollAsyncTxn() once the DMA
    // completes - do NOT close them here, the transfer is still in flight.
  } else {
    // Queueing failed (e.g. DMA channel busy) - fall back to closing out
    // immediately rather than leaving CS asserted with nothing pending.
    digitalWrite(_cs, HIGH);
    SPI.endTransaction();
  }
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
      _drainOnePendingBlocking();
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

void EspSpiDriver::_drainOnePendingBlocking() {
  // Kept as the shared "make sure the bus is free, then do one blocking
  // receive" helper used by resolveHostname()/getLocalIP()'s drain loops.
  // Just calls parsePacket() now that the receive path is synchronous
  // again (see REVERTED note on parsePacket() below).
  parsePacket();
}

int EspSpiDriver::parsePacket() {
  // REVERTED (2026-07-12): tried making this async (DMA + EventResponder,
  // same pattern as sendPacketTo(), full padded transfer length kept this
  // time to avoid repeating the earlier auth-breaking mistake). Built and
  // ran on real hardware, but: (a) didn't fix the audio buzz this whole
  // investigation is chasing - unchanged, still present; (b) introduced a
  // new regression - SPI-STATS showed entire 5-second windows with zero
  // audio sends, not seen in any earlier test, most likely from send/
  // receive now contending over the shared bus/CS line in a way that
  // occasionally stalled audio entirely; (c) the [RX-STATS] avgUs number
  // didn't even drop (~2734us average, same as the blocking version),
  // meaning the change wasn't delivering the intended benefit either.
  // Net negative - reverted to the simple, proven-correct blocking
  // version below. If the receive path's blocking time needs addressing
  // again later, do it as its own careful investigation, not bundled
  // into the buzz chase.
  //
  // Wait for any in-flight async SEND (see sendPacketTo()) to finish
  // before starting our own transaction - they share the same bus/CS
  // line. Same bounded-wait-with-safety-valve pattern as sendPacketTo().
  uint32_t sendWaitStart = micros();
  while (_txnInFlight != SpiTxnType::NONE) {
    _pollAsyncTxn();
    if (_txnInFlight != SpiTxnType::NONE && (micros() - sendWaitStart > 5000)) {
      digitalWrite(_cs, HIGH);
      SPI.endTransaction();
      _spiEvent.clearEvent();
      _txnInFlight = SpiTxnType::NONE;
      break;
    }
  }

  if (digitalRead(_ready) == HIGH) {
    uint32_t rxStart = micros();
    delayMicroseconds(500);

    SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
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
      int consumed = 3 + (len < sizeof(_cfgBuffer) ? len : 0);
      while (consumed < MAX_SPI_BUF) {
        SPI.transfer(0x00);
        consumed++;
      }
      digitalWrite(_cs, HIGH);
      SPI.endTransaction();
      _rxCallCount++;
      _rxTotalUs += (micros() - rxStart);
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

      int consumed = 3 + (len > 0 && len < 512 ? len : 0);
      while (consumed < MAX_SPI_BUF) {
        SPI.transfer(0x00);
        consumed++;
      }
      digitalWrite(_cs, HIGH);
      SPI.endTransaction();
      _rxCallCount++;
      _rxTotalUs += (micros() - rxStart);
      return _rxLen;
    } else {
      for (int k = 1; k < MAX_SPI_BUF; k++) SPI.transfer(0x00);
      _rxLen = 0;
    }

    digitalWrite(_cs, HIGH);
    SPI.endTransaction();
    _rxCallCount++;
    _rxTotalUs += (micros() - rxStart);

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
    _drainOnePendingBlocking();
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
