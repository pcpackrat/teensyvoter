#include "EspSpiDriver.h"
#include "SpiProtocol.h"

EspSpiDriver::EspSpiDriver(uint8_t csPin, uint8_t readyPin, uint8_t resetPin) {
  _cs = csPin;
  _ready = readyPin;
  _reset = resetPin;
  _rxLen = 0;
  _cachedIP = IPAddress(0, 0, 0, 0);
  _staticDNS = IPAddress(0, 0, 0, 0);
}

bool EspSpiDriver::begin() {
  pinMode(_cs, OUTPUT);
  digitalWrite(_cs, HIGH);

  // Must configure the hardware SPI pins (drives SCK to a known idle-low
  // state) before releasing the ESP32 from reset below. SCK (Teensy pin
  // 13) drives the ESP32's IO12, which is a boot strapping pin
  // (VDD_SDIO flash voltage select) - if it's left floating at the
  // reset edge it can be sampled high, selecting the wrong flash
  // voltage and causing an intermittent boot failure. Reproduced on
  // hardware: boot success depended on power-up timing before this fix.
  SPI.begin();

  pinMode(_reset, OUTPUT);
  digitalWrite(_reset, LOW);
  delay(100);
  digitalWrite(_reset, HIGH);

  pinMode(_ready, INPUT);

  return true;
}

void EspSpiDriver::update() {
  _pollAsyncTxn();
}

void EspSpiDriver::sendPacket(const uint8_t *data, uint16_t len) {
  sendPacketTo(_targetIP, _targetPort, data, len);
}

void EspSpiDriver::_pollAsyncTxn() {
  if (_txnInFlight == SpiTxnType::NONE) return;
  if (_spiEvent) { // EventResponder::operator bool() - true once DMA completes
    // _txRxScratch now holds this transaction's full response - inspect it
    // before closing out, instead of discarding it. See the queue's
    // declaration comment in EspSpiDriver.h for why this matters.
    _handleAsyncRxScratch();
    digitalWrite(_cs, HIGH);
    SPI.endTransaction();
    _spiEvent.clearEvent();
    _txnInFlight = SpiTxnType::NONE;
  }

  static uint32_t lastAsyncRxReport = 0;
  static uint32_t lastReportedRecovered = 0;
  static uint32_t lastReportedDropped = 0;
  static uint32_t lastReportedCksumBad = 0;
  if (millis() - lastAsyncRxReport > 5000) {
    lastAsyncRxReport = millis();
    // All three counters are cumulative-since-boot (never reset), so
    // gating purely on ">0" reprinted the same totals every 5s forever
    // after the first ever event - only print when something changed.
    if (_asyncRxRecoveredCount != lastReportedRecovered ||
        _asyncRxQueueDroppedCount != lastReportedDropped ||
        _downlinkCksumMismatchCount != lastReportedCksumBad) {
      Serial.printf("[SPI-RX] Async-recovered downlink packets: %lu total (%lu dropped due to queue full, %lu checksum-bad)\r\n",
                    (unsigned long)_asyncRxRecoveredCount, (unsigned long)_asyncRxQueueDroppedCount,
                    (unsigned long)_downlinkCksumMismatchCount);
      lastReportedRecovered = _asyncRxRecoveredCount;
      lastReportedDropped = _asyncRxQueueDroppedCount;
      lastReportedCksumBad = _downlinkCksumMismatchCount;
    }
  }
}

// See _lastDownlinkSeq's declaration comment. Called from both places a
// downlink STATUS_HAS_DATA payload gets read (parsePacket()'s live poll,
// _handleAsyncRxScratch()'s recovered path) - loss can happen via either.
void EspSpiDriver::_checkDownlinkSeq(uint16_t seq) {
  if (_downlinkSeqInit) {
    uint16_t expected = (uint16_t)(_lastDownlinkSeq + 1);
    if (seq != expected) {
      uint16_t gap = (uint16_t)(seq - expected);
      _downlinkSeqDropCount += gap;
      Serial.printf("[SEQ] Gap in ESP32->Teensy SPI downlink: expected %u got %u (missing %u, total missing %lu)\r\n",
                    expected, seq, gap, (unsigned long)_downlinkSeqDropCount);
    }
  }
  _lastDownlinkSeq = seq;
  _downlinkSeqInit = true;
}

// Parses a completed async send's response side (_txRxScratch) exactly
// like parsePacket()'s live-read branches do, but operating on the
// already-fully-received in-memory buffer instead of doing fresh
// SPI.transfer(0x00) calls - the DMA already clocked the whole thing in.
void EspSpiDriver::_handleAsyncRxScratch() {
  uint8_t status = _txRxScratch[0];
  uint16_t len = ((uint16_t)_txRxScratch[1] << 8) | _txRxScratch[2];

  if (status == STATUS_CONFIG_CMD) {
    if (len > 0 && len < sizeof(_cfgQueue[0]) && (3 + len) <= MAX_SPI_BUF) {
      _queueRxConfigCmd(&_txRxScratch[3], len);
    }
  } else if (status & STATUS_HAS_DATA) {
    // [3][4] = CKSUM, [5][6] = SEQ, payload from [7] - matches the ESP32's
    // downlink poll staging format (see Spirit.cpp's "safe window" handler).
    uint16_t recvCksum = ((uint16_t)_txRxScratch[3] << 8) | _txRxScratch[4];
    uint16_t recvSeq = ((uint16_t)_txRxScratch[5] << 8) | _txRxScratch[6];
    if (len > 0 && len < 512 && (7 + len) <= MAX_SPI_BUF) {
      _checkDownlinkSeq(recvSeq);
      uint16_t calcCksum = 0;
      for (uint16_t i = 0; i < len; i++) calcCksum += _txRxScratch[7 + i];
      if (calcCksum != recvCksum) {
        _downlinkCksumMismatchCount++;
      }
      if (_asyncRxQueueCount >= ASYNC_RX_QUEUE_SIZE) {
        _asyncRxQueueDroppedCount++;
      } else {
        memcpy(_asyncRxQueue[_asyncRxQueueTail], &_txRxScratch[7], len);
        _asyncRxQueueLen[_asyncRxQueueTail] = len;
        _asyncRxQueueTail = (_asyncRxQueueTail + 1) % ASYNC_RX_QUEUE_SIZE;
        _asyncRxQueueCount++;
        _asyncRxRecoveredCount++;
      }
    }
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
      // Forced abort of a still-in-flight transfer - previously silent.
      // The prior transaction's DMA data may not have been fully clocked
      // out to the ESP32 when we yank CS high here, so this is a real,
      // if rare, potential frame corruption/loss point right at the
      // Teensy->ESP32 boundary. Counting it so it's visible instead of
      // invisible when chasing dropped/choppy audio.
      _forcedAbortCount++;
      Serial.printf("[SPI-TX] Forced abort of stuck async send (#%lu so far)\r\n",
                    (unsigned long)_forcedAbortCount);
      digitalWrite(_cs, HIGH);
      SPI.endTransaction();
      _spiEvent.clearEvent();
      _txnInFlight = SpiTxnType::NONE;
      break;
    }
  }

  // Drain any pending downlink data BEFORE opening our own transaction
  // below - parsePacket() runs a fully self-contained beginTransaction()/
  // CS-low/.../CS-high/endTransaction() cycle of its own. Calling it after
  // our CS was already asserted (as this used to do, checked further
  // down) left CS high (parsePacket()'s own closing edge) with nothing
  // re-asserting it before the async SPI.transfer() below - that transfer
  // then ran with CS deasserted, invisible to the ESP32 slave, a silent
  // single-frame loss. Confirmed as the source of residual isolated
  // "missing 1" SPI sequence gaps that persisted after the re-arm-window
  // fixes (which addressed a different, ESP32-side loss mechanism).
  if (digitalRead(_ready) == HIGH) {
    parsePacket();
  }

  // Pad before reasserting CS - must clear the ESP32 slave's measured
  // dequeue-process-requeue re-arm window (MaxRearmUs in the ESP32 HB log,
  // observed up to ~427us on hardware) or this transaction starts before
  // the slave is listening again and is silently lost. Was 200us here -
  // less than the observed ceiling - while parsePacket()'s equivalent pad
  // below was already 500us; that asymmetry matched the evidence exactly
  // (SPI seq gaps were overwhelmingly on the send side). 600us keeps
  // margin above the measured max.
  delayMicroseconds(600);

  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
  digitalWrite(_cs, LOW);
  delayMicroseconds(100); // Time for the ESP32 to detect the CS edge

  memset(_txBuffer, 0, MAX_SPI_BUF);

  // Header: [CMD] [SEQ_HI] [SEQ_LO] [LEN_HI] [LEN_LO] [IP...4] [PORT...2] [CKSUM_HI] [CKSUM_LO]
  // SEQ is a free-running per-transaction counter, purely at this SPI
  // transport layer (independent of anything inside the Voter protocol
  // payload) - lets the ESP32 directly detect a dropped transaction on
  // this leg (a gap in SEQ) instead of only ever inferring loss/
  // corruption indirectly from app-layer symptoms like the challenge-
  // field validity check. SEQ alone only proves a transaction arrived,
  // not that its content is intact - added CKSUM (simple 16-bit additive
  // sum over the payload) after two rounds of transport fixes still left
  // reports of corrupted-sounding transmitted audio at the server with
  // clean SEQ/loss numbers throughout, which pointed at exactly this
  // blind spot: bit errors within a frame that don't happen to land on
  // the SEQ bytes are completely invisible to a sequence-only check.
  _txBuffer[0] = CMD_SEND_UDP;
  _txBuffer[1] = (_txSeq >> 8) & 0xFF;
  _txBuffer[2] = _txSeq & 0xFF;
  _txSeq++;
  _txBuffer[3] = (len >> 8) & 0xFF;
  _txBuffer[4] = len & 0xFF;
  for (int i = 0; i < 4; i++) _txBuffer[5 + i] = ip[i];
  _txBuffer[9] = (port >> 8) & 0xFF;
  _txBuffer[10] = port & 0xFF;

  // Payload
  uint16_t payloadLen = (len > (MAX_SPI_BUF - 13)) ? (MAX_SPI_BUF - 13) : len;
  uint16_t cksum = 0;
  for (uint16_t i = 0; i < payloadLen; i++) cksum += data[i];
  _txBuffer[11] = (cksum >> 8) & 0xFF;
  _txBuffer[12] = cksum & 0xFF;
  memcpy(&_txBuffer[13], data, payloadLen);

  // Always the full buffer now - see the note at the top of this
  // function for why. This also means the ESP32 always gets a
  // full-length transaction regardless of whether it had anything
  // queued for us, so there's no truncation/data-loss risk from that
  // angle either - but we still drain via a real parsePacket() call
  // above (before our own CS assertion) when READY is high, since this
  // async send's own RX side (_txRxScratch) is discarded/never parsed, so
  // the bytes would otherwise never reach _rxBuffer/_handlePacket().
  uint16_t transferLen = MAX_SPI_BUF;

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

// Appends a received config command to the RX queue (FIFO, drained by
// readConfigCmd()). See the queue's declaration comment in EspSpiDriver.h
// for why this needs to be a real queue and not a single slot.
void EspSpiDriver::_queueRxConfigCmd(const uint8_t *data, uint16_t len) {
  if (len == 0 || len >= sizeof(_cfgQueue[0])) return;
  if (_cfgQueueCount >= CFG_RX_QUEUE_SIZE) {
    Serial.println("[CFG-RX] Queue full, dropping incoming config command!");
    return;
  }
  memcpy(_cfgQueue[_cfgQueueTail], data, len);
  _cfgQueueLen[_cfgQueueTail] = len;
  _cfgQueueTail = (_cfgQueueTail + 1) % CFG_RX_QUEUE_SIZE;
  _cfgQueueCount++;
}

// Reads a "status + 2-byte len [+ payload]" response frame from an
// already-open transaction (CS already low) and pads the rest out to
// MAX_SPI_BUF. Routes a STATUS_CONFIG_CMD response into the RX queue -
// every response-reading site needs this, not just parsePacket(), or a
// queued config command that happens to land in the same transaction as
// (say) a getLocalIP() poll gets silently discarded instead of ever
// reaching hasConfigCmd()/readConfigCmd(). Confirmed on hardware: a
// WiFi SSID change made through the AP setup page was lost this way -
// getLocalIP() is polled every 5s by the Teensy's own reconnect-check
// loop, and one of those polls ate the queued PARAM_WIFI_SSID command
// instead of the config-dispatch path ever seeing it.
//
// outBuf/maxOutLen are for a STATUS_HAS_DATA payload (e.g. a 4-byte IP);
// pass maxOutLen=0 if the caller doesn't expect one. Returns the raw
// status byte; *outLen is set to the STATUS_HAS_DATA payload length (0
// if none, or if it was a config command instead).
uint8_t EspSpiDriver::_readResponseFrame(uint8_t *outBuf, uint8_t maxOutLen, uint16_t *outLen) {
  uint8_t status = SPI.transfer(0x00);
  uint8_t lenHi = SPI.transfer(0x00);
  uint8_t lenLo = SPI.transfer(0x00);
  uint16_t len = (lenHi << 8) | lenLo;
  int consumed = 3;
  *outLen = 0;

  if (status == STATUS_CONFIG_CMD && len > 0 && len < sizeof(_cfgQueue[0])) {
    uint8_t cmdBuf[256];
    for (int i = 0; i < len; i++) cmdBuf[i] = SPI.transfer(0x00);
    _queueRxConfigCmd(cmdBuf, len);
    consumed += len;
  } else if ((status & STATUS_HAS_DATA) && len > 0 && len <= maxOutLen) {
    for (int i = 0; i < len; i++) outBuf[i] = SPI.transfer(0x00);
    *outLen = len;
    consumed += len;
  }

  while (consumed < MAX_SPI_BUF) {
    SPI.transfer(0x00);
    consumed++;
  }
  return status;
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

      uint8_t ip[4] = {0, 0, 0, 0};
      uint16_t len = 0;
      uint8_t status = _readResponseFrame(ip, sizeof(ip), &len);

      digitalWrite(_cs, HIGH);
      SPI.endTransaction();

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
      } else if (status & STATUS_HAS_DATA) {
        Serial.printf("[DNS] SPI RX Other: Status 0x%02X\r\n", status);
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

  uint8_t ip[4] = {0, 0, 0, 0};
  uint16_t len = 0;
  _readResponseFrame(ip, sizeof(ip), &len);

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
  // Serve already-recovered downlink data first (see EspSpiDriver.h's
  // _asyncRxQueue comment) - no live SPI activity needed, and this must
  // win over a fresh poll below so an older recovered packet can't sit
  // behind a newer live one and go stale.
  if (_asyncRxQueueCount > 0) {
    uint16_t len = _asyncRxQueueLen[_asyncRxQueueHead];
    memcpy(_rxBuffer, _asyncRxQueue[_asyncRxQueueHead], len);
    _rxLen = len;
    _asyncRxQueueHead = (_asyncRxQueueHead + 1) % ASYNC_RX_QUEUE_SIZE;
    _asyncRxQueueCount--;
    return _rxLen;
  }

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
    // Was 500us - barely above the observed ESP32 re-arm ceiling
    // (MaxRearmUs, up to ~455-464us on hardware), same insufficient-
    // margin situation sendPacketTo()'s original 200us pad was in before
    // it got bumped to 600us. That fix (plus the READY-timing fix on the
    // ESP32 side) cut downlink loss from ~36% to ~15% but didn't finish
    // the job - this is the second half, matching the same pattern.
    delayMicroseconds(700);

    SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
    digitalWrite(_cs, LOW);
    delayMicroseconds(50);

    uint8_t status = SPI.transfer(0x00);

    if (status == STATUS_CONFIG_CMD) {
      uint8_t lenHi = SPI.transfer(0x00);
      uint8_t lenLo = SPI.transfer(0x00);
      uint16_t len = (lenHi << 8) | lenLo;

      int consumed = 3;
      if (len > 0 && len < sizeof(_cfgQueue[0])) {
        uint8_t cmdBuf[256];
        for (int i = 0; i < len; i++) {
          cmdBuf[i] = SPI.transfer(0x00);
        }
        _queueRxConfigCmd(cmdBuf, len);
        consumed += len;
      }
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
      uint8_t cksumHi = SPI.transfer(0x00);
      uint8_t cksumLo = SPI.transfer(0x00);
      uint16_t recvCksum = (cksumHi << 8) | cksumLo;
      uint8_t seqHi = SPI.transfer(0x00);
      uint8_t seqLo = SPI.transfer(0x00);
      uint16_t recvSeq = (seqHi << 8) | seqLo;

      if (len > 0 && len < 512) {
        _checkDownlinkSeq(recvSeq);
        uint16_t calcCksum = 0;
        for (int i = 0; i < len; i++) {
          _rxBuffer[i] = SPI.transfer(0x00);
          calcCksum += _rxBuffer[i];
        }
        _rxLen = len;
        if (calcCksum != recvCksum) {
          _downlinkCksumMismatchCount++;
        }
      } else {
        _rxLen = 0;
      }

      int consumed = 7 + (len > 0 && len < 512 ? len : 0);
      while (consumed < MAX_SPI_BUF) {
        SPI.transfer(0x00);
        consumed++;
      }
      digitalWrite(_cs, HIGH);
      SPI.endTransaction();
      return _rxLen;
    } else {
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
  if (_cachedIP != IPAddress(0, 0, 0, 0) && (millis() - _cachedIPTime < CACHED_IP_TTL_MS)) {
    return _cachedIP;
  }

  // Clear any pending unsolicited data before sending a command.
  //
  // REVERTED (matching resolveHostname()'s 50ms/200ms drain-settle
  // pattern, tried as a fix for a rare bogus-IP SPI glitch): this
  // function is called from the main loop path every 5s, not on-demand
  // like resolveHostname() - when there's a burst of queued items to
  // drain (routine, not just during AP setup), the slower pattern could
  // block the main loop for the better part of a second in one call.
  // Teensy is single-threaded, so that starves gps.update() from running
  // often enough and was observed causing real GPS lock loss - a far
  // worse cost than the rare cosmetic bug it was meant to fix. Back to
  // the original fast drain; the underlying SPI corruption question
  // (possibly CS on the input-only IO35 pin) is still open but needs a
  // fix that doesn't block the audio/GPS-critical main loop.
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

  uint8_t ip[4] = {0, 0, 0, 0};
  uint16_t len = 0;
  _readResponseFrame(ip, sizeof(ip), &len);

  digitalWrite(_cs, HIGH);
  SPI.endTransaction();

  IPAddress result(ip[0], ip[1], ip[2], ip[3]);
  if (result != IPAddress(0, 0, 0, 0)) {
    _cachedIP = result;
    _cachedIPTime = millis();
  }
  return result;
}

void EspSpiDriver::setTarget(IPAddress ip, uint16_t port) {
  _targetIP = ip;
  _targetPort = port;
}

void EspSpiDriver::_pushBlob(uint8_t cmd, const void *blobData, uint16_t blobLen) {
  if (blobLen > 500) return;

  // The ESP32 aborts (STATUS_HAS_DATA in the first status byte) if it
  // already has something else queued to send - originally fine since
  // pushConfig() only ever ran once at boot when there's little other
  // traffic. GPS status now shares this same helper but pushes every 5s
  // *during* active voter/audio traffic, when the ESP32 very often has
  // something queued - a single attempt was landing on a busy ESP32
  // often enough that it could go indefinitely without ever getting a
  // push through. Retry a few times, a beat apart, rather than requiring
  // pure luck.
  for (int attempt = 0; attempt < 5; attempt++) {
    delay(5);

    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    digitalWrite(_cs, LOW);
    delayMicroseconds(50);

    uint8_t status = SPI.transfer(cmd);
    if (status & STATUS_HAS_DATA) {
        digitalWrite(_cs, HIGH);
        SPI.endTransaction();
        continue; // busy - try again
    }

    SPI.transfer((blobLen >> 8) & 0xFF);
    SPI.transfer(blobLen & 0xFF);

    const uint8_t *data = (const uint8_t *)blobData;
    for (uint16_t i = 0; i < blobLen; i++) {
      SPI.transfer(data[i]);
    }

    // Pad to 512
    int sent = 3 + blobLen;
    while (sent < MAX_SPI_BUF) {
        SPI.transfer(0x00);
        sent++;
    }

    digitalWrite(_cs, HIGH);
    SPI.endTransaction();
    return;
  }
}

void EspSpiDriver::pushConfig(const void *configData, uint16_t configLen) {
  _pushBlob(CMD_PUSH_CONFIG, configData, configLen);
}

void EspSpiDriver::pushGpsStatus(const GpsStatus *status) {
  _pushBlob(CMD_PUSH_GPS_STATUS, status, sizeof(GpsStatus));
}

bool EspSpiDriver::hasConfigCmd() { return _cfgQueueCount > 0; }

int EspSpiDriver::readConfigCmd(uint8_t *buffer, size_t maxLen) {
  if (_cfgQueueCount == 0) return 0;
  uint8_t srcLen = _cfgQueueLen[_cfgQueueHead];
  size_t copyLen = (srcLen < (uint8_t)maxLen) ? srcLen : maxLen;
  memcpy(buffer, _cfgQueue[_cfgQueueHead], copyLen);
  _cfgQueueHead = (_cfgQueueHead + 1) % CFG_RX_QUEUE_SIZE;
  _cfgQueueCount--;
  return copyLen;
}
