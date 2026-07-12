#ifndef VOTER_CLIENT_H
#define VOTER_CLIENT_H

#include "GPSManager.h"
#include "JitterBuffer.h"
#include "NetworkManager.h"
#include "VoterProtocol.h"
#include <Arduino.h>

// State Machine
enum VoterState {
  VOTER_DISCONNECTED = 0,
  VOTER_AUTHENTICATING = 1,
  VOTER_CONNECTED = 2,
  VOTER_AUTH_ERROR = 3
};

enum VoterAuthError {
  AUTH_ERR_NONE = 0,
  AUTH_ERR_NO_RESPONSE = 1,
  AUTH_ERR_HOST_MISMATCH = 2,
  AUTH_ERR_CLIENT_REJECTED = 3
};

class VoterClient {
public:
  VoterClient();

  // Init
  void begin(NetworkManager *net, GPSManager *gps, IPAddress host,
             uint16_t port, const char *clientPwd, const char *hostPwd);

  // Main Loop
  void update();

  // Audio Input (called by Audio ISR or polling)
  void processAudioFrame(uint8_t *ulawData, uint8_t rssi, VTIME frameTime);

  // Sends the audio frame staged by processAudioFrame(), if any. Call once,
  // after the caller's own audio-queue-draining work for this pass is done
  // (see main.cpp) - deliberately NOT sent synchronously from within
  // processAudioFrame() itself, because that runs inside the record-queue
  // drain loop and the SPI-bridged send's ~700us blocking cost was
  // perturbing that loop's own draining cadence just enough to be audible
  // (a perfectly periodic ~50Hz artifact, since Phase 0 made this send
  // reliable/regular for the first time - confirmed via A/B against
  // Ethernet, which doesn't exhibit it, and isn't RF: WiFi TX power
  // experiments didn't explain it either).
  void flushPendingAudio();

  // Audio Output (TX)
  JitterBuffer
      jitterBuffer; // Public so main loop can feed it logic/read from it

  // Status
  bool isConnected() { return _state == VOTER_CONNECTED; }
  VoterState getState() { return _state; }
  VoterAuthError getAuthError() { return _authError; }
  void setTimingOffset(int16_t ms) { _timingOffsetMs = ms; }
  uint32_t getLastHostAudioTime() { return _lastHostAudioTime; }

private:
  // Core Dependencies
  NetworkManager *_net;
  GPSManager *_gps;

  // Config
  IPAddress _hostIP;
  uint16_t _hostPort;
  const char *_clientPwd;
  const char *_hostPwd;

  // State
  VoterState _state;
  VoterAuthError _authError;
  uint32_t _lastAttemptTime;

  // Protocol State
  char _myChallenge[VOTER_CHALLENGE_LEN + 1];
  char _serverChallenge[VOTER_CHALLENGE_LEN + 1];
  uint32_t _serverDigest; // The digest we expect FROM the server
  uint32_t _myDigest;     // The digest we send TO the server

  // Helpers
  uint32_t _crc32(const uint8_t *buf1, const uint8_t *buf2);
  void _sendAuthPacket();
  void _handlePacket(const uint8_t *data, int len);
  void _handleProxyAudioPacket(const uint8_t *data, int len);
  void _generateChallenge();
  void _sendGPSPacket();
  uint32_t _lastGPSSend;
  uint32_t _gpsLostTime;
  uint32_t _authAttempts;
  uint32_t _lastRxTime;
  bool _hasWarnedAuth;
  bool _hasWarnedGps;

  // Protocol Sequencing
  uint32_t _packetCounter; // Free-running 20ms counter
  int32_t _seqOffset;      // Phase offset to align sequence with PPS

  // Transmission State and Delay Buffer
  bool _isTransmitting;         // True if we are currently sending audio
  int16_t _timingOffsetMs;      // User-defined offset (+ = newer, - = older)
  uint32_t _lastCallTime;       // Last audio frame processing time

  // Authority Alignment: Continuous Frame Counting
  uint32_t _anchorSec;
  uint32_t _anchorNsec;
  uint32_t _burstPacketCount;
  bool _anchorActive;
  uint32_t _gpSeq;              // GP Sequence (+1 per non-audio packet)
  uint32_t _lastHostAudioTime; // Timestamp of last audio packet from server

  // Phase 0: SPI send instrumentation (audio path only - this is the
  // time-critical 20ms-cadence send). Logged/reset every 5s by _reportSpiStats().
  uint32_t _spiStatsWindowStart;
  uint32_t _audioSendCount;
  uint32_t _audioSendMaxUs;
  uint64_t _audioSendTotalUs;
  uint32_t _audioSendOver15msCount;
  void _reportSpiStats();

  // Deferred audio send staging - see flushPendingAudio().
  PROXY_AUDIO_PACKET _pendingAudioPkt;
  bool _pendingAudioSend;

public:
  uint32_t getPacketCounter() { return _packetCounter; }
  void incrementPacketCounter() { _packetCounter++; }
  void alignTimestampPhase(); // Called on PPS to align sequence logic
};

#endif
