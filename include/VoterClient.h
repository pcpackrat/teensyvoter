#ifndef VOTER_CLIENT_H
#define VOTER_CLIENT_H

#include "GPSManager.h"
#include "NetworkManager.h"
#include "VoterProtocol.h"
#include <Arduino.h>

// State Machine
enum VoterState {
  VOTER_DISCONNECTED = 0,
  VOTER_AUTHENTICATING = 1,
  VOTER_CONNECTED = 2
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

  // Status
  bool isConnected() { return _state == VOTER_CONNECTED; }

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
  void _generateChallenge();
  void _sendGPSPacket();
  uint32_t _lastGPSSend;
};

#endif
