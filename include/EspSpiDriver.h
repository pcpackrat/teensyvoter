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
  bool begin() override;
  void update() override;
  bool isConnected() override;
  IPAddress getLocalIP() override;
  DriverType getType() override { return DRIVER_ESP32_SPI; }

  void setTarget(IPAddress ip, uint16_t port) override;
  void sendPacket(const uint8_t *data, uint16_t len) override;
  void sendPacketTo(IPAddress ip, uint16_t port, const uint8_t *data,
                    uint16_t len) override;
  int parsePacket() override;
  int read(uint8_t *buffer, size_t maxLen) override;

  // WiFi Specific
  IPAddress resolveHostname(const char *hostname) override;
  IPAddress getDNSServer();
  void setStaticDNS(IPAddress dns) { _staticDNS = dns; }

  // Config Management (for ESP32 web server)
  void pushConfig(const void *configData, uint16_t configLen);
  void pushGpsStatus(const GpsStatus *status);
  bool hasConfigCmd();
  int readConfigCmd(uint8_t *buffer, size_t maxLen);

private:
  uint8_t _cs, _ready, _reset;
  IPAddress _targetIP;
  uint16_t _targetPort;
  // IP cache: avoids a fresh ~100ms+ blocking SPI round-trip on every
  // call (getLocalIP() is called from serial menu renders, which would
  // otherwise stall real-time audio well past the ~20ms budget on every
  // menu view once connected). Time-bounded, not permanent - a
  // permanent cache meant one corrupted read that happened to produce a
  // non-zero garbage IP would latch forever with no way to recover
  // short of a reboot, and it could never reflect the ESP32 legitimately
  // changing address (e.g. ETH/WiFi/AP failover). Observed on hardware:
  // a bogus "IP: 0.0.0.63" got cached and believed for the rest of a
  // session.
  IPAddress _cachedIP;
  uint32_t _cachedIPTime = 0;
  static const uint32_t CACHED_IP_TTL_MS = 30000;
  IPAddress _staticDNS;

  // Buffers
  uint8_t _rxBuffer[512];
  int _rxLen;

  // Config command queue (from ESP32 web server). Multiple commands can
  // arrive back-to-back over separate SPI transactions - one per web-
  // form field, then a final SAVE_REBOOT - and getLocalIP()'s drain loop
  // can pull several of them in a row without ever yielding back to the
  // main loop() in between. A single slot here let a later command
  // (usually SAVE_REBOOT) silently overwrite an earlier one before
  // readConfigCmd() ever got called - same bug class as the ESP32-side
  // queueConfigCmd() single-slot bug already fixed, just on this end of
  // the same pipe. Confirmed on hardware: WiFi SSID/password changes via
  // the AP setup page kept being lost even after the ESP32-side fix,
  // because this side had the identical problem independently.
  static const uint8_t CFG_RX_QUEUE_SIZE = 8;
  uint8_t _cfgQueue[CFG_RX_QUEUE_SIZE][256];
  uint8_t _cfgQueueLen[CFG_RX_QUEUE_SIZE];
  uint8_t _cfgQueueHead = 0;
  uint8_t _cfgQueueTail = 0;
  uint8_t _cfgQueueCount = 0;
  void _queueRxConfigCmd(const uint8_t *data, uint16_t len);

  // SPI Helpers
  bool _waitReady(uint32_t timeoutMs);
  uint8_t _readStatus();
  void _pushBlob(uint8_t cmd, const void *blobData, uint16_t blobLen);
  uint8_t _readResponseFrame(uint8_t *outBuf, uint8_t maxOutLen, uint16_t *outLen);

  // Async (DMA) send state. sendPacketTo() used to busy-wait the CPU for
  // the full transfer duration (~700us+), which was stalling long enough,
  // with perfect 20ms regularity, to audibly disrupt the Teensy's own
  // real-time audio pipeline. This queues the transfer via DMA and returns
  // immediately; _pollAsyncTxn() (called from update()) closes out CS/
  // the SPI transaction once the EventResponder reports completion.
  // parsePacket() (the receive path) also waits on this before starting
  // its own blocking transaction, since both directions share the same
  // bus/CS line - see parsePacket()'s comment for why receive itself
  // isn't async (tried it, reverted: didn't fix the buzz this was meant
  // to address, and introduced a new intermittent-audio-dropout
  // regression with no measured benefit).
  //
  // _txBuffer must be a persistent (not stack-local) buffer since the DMA
  // engine reads/writes it after sendPacketTo() has already returned.
  // _txRxScratch is a SEPARATE buffer for the async transfer's incoming
  // side - using the same buffer for tx and rx caused a real bug: the RX
  // DMA channel writing incoming (mostly-zero) bytes into a shared buffer
  // could race ahead of the TX channel finishing that same address,
  // clobbering the tail of the outgoing message with zeros right before
  // it went out (observed: syslog messages arriving with their last ~2
  // bytes replaced by NUL). Separate buffers eliminate the race.
  enum class SpiTxnType { NONE, SEND };
  EventResponder _spiEvent;
  SpiTxnType _txnInFlight = SpiTxnType::NONE;
  uint8_t _txBuffer[MAX_SPI_BUF];
  uint8_t _txRxScratch[MAX_SPI_BUF];
  uint32_t _forcedAbortCount = 0;
  uint16_t _txSeq = 0; // free-running CMD_SEND_UDP transport sequence, see sendPacketTo()
  void _pollAsyncTxn();

  // Downlink data recovered from an async send's own response side
  // (_txRxScratch), which used to be discarded outright. The ESP32 can
  // stage a STATUS_HAS_DATA downlink payload onto ANY transaction's
  // response, not just ones parsePacket() explicitly polled for - the
  // digitalRead(_ready) check before an async send is a check-then-act
  // race (READY can go high in the gap between that check and the send
  // actually going out), and when it loses that race the downlink packet
  // was riding on this exact transaction's response, previously thrown
  // away unread. Same "single slot gets clobbered by a second writer"
  // risk as the config-cmd queue below, so it gets the same fix: a real
  // queue, drained by parsePacket() before it does anything else, not a
  // single slot that a fresh READY-triggered read could overwrite before
  // this one's ever consumed.
  static const uint8_t ASYNC_RX_QUEUE_SIZE = 4;
  uint8_t _asyncRxQueue[ASYNC_RX_QUEUE_SIZE][512];
  uint16_t _asyncRxQueueLen[ASYNC_RX_QUEUE_SIZE];
  uint8_t _asyncRxQueueHead = 0;
  uint8_t _asyncRxQueueTail = 0;
  uint8_t _asyncRxQueueCount = 0;
  uint32_t _asyncRxRecoveredCount = 0; // how much this fix is actually catching - diagnostic
  uint32_t _asyncRxQueueDroppedCount = 0;
  void _handleAsyncRxScratch();

  // resolveHostname()/getLocalIP() loop on digitalRead(_ready) to drain
  // stale pending data before issuing a fresh request via parsePacket().
  void _drainOnePendingBlocking();
};

#endif
