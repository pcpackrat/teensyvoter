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
  bool begin(uint8_t *mac) override;
  void update() override;
  bool isConnected() override;
  IPAddress getLocalIP() override;
  IPAddress getSubnetMask() override;
  IPAddress getGateway() override;
  IPAddress getDNS() override;
  DriverType getType() override { return DRIVER_WIFI_SPI; }

  void setTarget(IPAddress ip, uint16_t port) override;
  void sendPacket(const uint8_t *data, uint16_t len) override;
  void sendPacketTo(IPAddress ip, uint16_t port, const uint8_t *data,
                    uint16_t len) override;
  int parsePacket() override;
  int read(uint8_t *buffer, size_t maxLen) override;

  // WiFi Specific
  void setCredentials(const char *ssid, const char *pass);
  IPAddress resolveHostname(const char *hostname) override;
  IPAddress getDNSServer();
  void setStaticDNS(IPAddress dns) { _staticDNS = dns; }

  // Config Management (for ESP32 web server)
  void pushConfig(const void *configData, uint16_t configLen);
  bool hasConfigCmd();
  int readConfigCmd(uint8_t *buffer, size_t maxLen);

private:
  uint8_t _cs, _ready, _reset;
  IPAddress _targetIP;
  uint16_t _targetPort;
  IPAddress _cachedIP; // IP Cache
  IPAddress _staticDNS;

  // Buffers
  uint8_t _rxBuffer[512];
  int _rxLen;

  // Config command buffer (from ESP32 web server)
  uint8_t _cfgBuffer[256];
  int _cfgLen;

  // SPI Helpers
  bool _waitReady(uint32_t timeoutMs);
  uint8_t _readStatus();

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
  void _pollAsyncTxn();

  // resolveHostname()/getLocalIP() loop on digitalRead(_ready) to drain
  // stale pending data before issuing a fresh request via parsePacket().
  void _drainOnePendingBlocking();

  // Receive-path instrumentation, reported every 5s from update(). Made
  // visible because parsePacket() was previously fully blocking and
  // totally uninstrumented - in the idle-status case it was clocking a
  // full ~2ms dummy transfer for no reason. Confirmed via these counters:
  // firing ~1x/sec at ~2.7ms/call, the largest single blocking stall in
  // the driver - tried making it async to address that, reverted (see
  // parsePacket()), so this still reflects real per-call blocking time.
  uint32_t _rxCallCount = 0;
  uint32_t _rxTotalUs = 0;
  uint32_t _rxStatsWindowStart = 0;
  void _reportRxStats();
};

#endif
