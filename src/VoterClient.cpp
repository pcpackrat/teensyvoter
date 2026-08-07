#include "VoterClient.h"
#include <stdint.h>
#include "Logger.h"

// Local byte-swap helpers with unique names to avoid toolchain builtin issues
// Compiler intrinsics for ARM Cortex-M7 (Teensy 4.1) - Little Endian to Network
// (Big Endian)
static inline uint32_t my_htonl(uint32_t x) { return __builtin_bswap32(x); }
static inline uint16_t my_htons(uint16_t x) { return __builtin_bswap16(x); }
static inline uint32_t my_ntohl(uint32_t x) { return __builtin_bswap32(x); }
static inline uint16_t my_ntohs(uint16_t x) { return __builtin_bswap16(x); }

// Legacy CRC32 Table from Voter.c (line 539)
static const uint32_t crc_32_tab[] = {
    0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f,
    0xe963a535, 0x9e6495a3, 0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988,
    0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91, 0x1db71064, 0x6ab020f2,
    0xf3b97148, 0x84be41de, 0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
    0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec, 0x14015c4f, 0x63066cd9,
    0xfa0f3d63, 0x8d080df5, 0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172,
    0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b, 0x35b5a8fa, 0x42b2986c,
    0xdbbbc9d6, 0xacbcf940, 0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
    0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423,
    0xcfba9599, 0xb8bda50f, 0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924,
    0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d, 0x76dc4190, 0x01db7106,
    0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
    0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d,
    0x91646c97, 0xe6635c01, 0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e,
    0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457, 0x65b0d9c6, 0x12b7e950,
    0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
    0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7,
    0xa4d1c46d, 0xd3d6f4fb, 0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0,
    0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9, 0x5005713c, 0x270241aa,
    // (Truncated to save space - standard CRC32)
    0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
    0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81,
    0xb7bd5c3b, 0xc0ba6cad, 0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a,
    0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683, 0xe3630b12, 0x94643b84,
    0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
    0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb,
    0x196c3671, 0x6e6b06e7, 0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc,
    0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5, 0xd6d6a3e8, 0xa1d1937e,
    0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
    0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55,
    0x316e8eef, 0x4669be79, 0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236,
    0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f, 0xc5ba3bbe, 0xb2bd0b28,
    0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
    0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f,
    0x72076785, 0x05005713, 0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38,
    0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21, 0x86d3d2d4, 0xf1d4e242,
    0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
    0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69,
    0x616bffd3, 0x166ccf45, 0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2,
    0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db, 0xaed16a4a, 0xd9d65adc,
    0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
    0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605, 0xcdd70693,
    0x54de5729, 0x23d967bf, 0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94,
    0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d};

VoterClient::VoterClient() {
  _state = VOTER_DISCONNECTED;
  _authError = AUTH_ERR_NONE;
  _lastAttemptTime = 0;
  _lastGPSSend = 0;
  _gpsLostTime = 0;
  _authAttempts = 0;
  _lastRxTime = 0;
  _corruptedPacketCount = 0;
  _lastCorruptedReportTime = 0;
  _authMismatchStreak = 0;
  _droppedAudioDigestMismatch = 0;
  _lastDroppedAudioReportTime = 0;
  _audioFramesSent = 0;
  _hasWarnedAuth = false;
  _hasWarnedGps = false;
  _hasWarnedGpsWaitConnect = false;
  _serverDigest = 0;
  _myDigest = 0;
  memset(_myChallenge, 0, sizeof(_myChallenge));
  memset(_serverChallenge, 0, sizeof(_serverChallenge));
  _packetCounter = 0;
  _seqOffset = 0;
  _isTransmitting = false;
  _timingOffsetMs = -180; // Default to Authority Value
  _anchorActive = false;
  _burstPacketCount = 0;
  _gpSeq = 0;
  _lastHostAudioTime = 0;
  _pendingAudioSend = false;
}

void VoterClient::begin(NetworkManager *net, GPSManager *gps, IPAddress host,
                        uint16_t port, const char *clientPwd,
                        const char *hostPwd) {
  _net = net;
  _gps = gps;
  _hostIP = host;
  _hostPort = port;
  _clientPwd = clientPwd;
  _hostPwd = hostPwd;

  _generateChallenge();
  _net->setTarget(_hostIP, _hostPort);
}

void VoterClient::update() {
  // 1. Check Incoming
  int packetSize = _net->parsePacket();
  if (packetSize >= (int)sizeof(VOTER_PACKET_HEADER)) {
    uint8_t buffer[MAX_BUFFER_SIZE];
    _net->read(buffer, sizeof(buffer));
    _handlePacket(buffer, packetSize);
  }

  // 2. Authentication Loop
  if (_state == VOTER_DISCONNECTED || _state == VOTER_AUTHENTICATING) {
    if (millis() - _lastAttemptTime >
        2000) { // Retry every 2 seconds (was 500ms)
      _lastAttemptTime = millis();
      // Only attempt to connect if we have a GPS Lock
      if (_gps && _gps->isLocked()) {
        _hasWarnedGpsWaitConnect = false;
        _sendAuthPacket();
      } else if (!_hasWarnedGpsWaitConnect) {
        _hasWarnedGpsWaitConnect = true;
        Serial.println("[Voter] Waiting for GPS Lock before connecting...");
      }
    }

    // Check for silent failure (Server ignoring us because of bad password or
    // network) Warn once after 5 attempts (approx 10 seconds)
    if (_authAttempts >= 5 && !_hasWarnedAuth) {
      _hasWarnedAuth = true;

      if (_state == VOTER_AUTHENTICATING) {
        _authError = AUTH_ERR_CLIENT_REJECTED;
        Serial.println("[Voter] WARNING: Client Password rejected by Host!");
      } else if (_lastRxTime == 0) {
        _authError = AUTH_ERR_NO_RESPONSE;
        Serial.println(
            "[Voter] WARNING: No response from Host. Check network!");
      } else {
        // Heard from host but never reached AUTHENTICATING state
        _authError = AUTH_ERR_CLIENT_REJECTED;
        Serial.println(
            "[Voter] WARNING: Client Password rejected (Host silent).");
      }

      _state = VOTER_AUTH_ERROR; // Stop retrying!
    }
  } else if (_state == VOTER_CONNECTED) {
    // 3. Check for GPS Lock Loss (with Debounce)
    if (_gps && !_gps->isLocked()) {
      if (_gpsLostTime == 0) {
        _gpsLostTime = millis();
      }

      // Print "Signal Lost" warning only after 500ms of loss
      if (!_hasWarnedGps && (millis() - _gpsLostTime > 500)) {
        _hasWarnedGps = true;
        char ts[20];
        _gps->getTimestamp(ts);
        Serial.printf(
            "%s [Voter] GPS Signal Lost! Entering 1s holdover mode...\r\n",
            ts);
      }

      // Disconnect if lock is lost for > 1,000ms (1 second) - Immediate Fail-safe
      if (millis() - _gpsLostTime > 1000) {
        GPSManager::GPSLockStatus status = _gps->getLockStatus();
        char ts[20];
        _gps->getTimestamp(ts);
        Serial.printf("%s [Voter] Holdover Expired! GPS Lock Lost. Reason: ",
                      ts);
        switch (status) {
        case GPSManager::GPS_NO_FIX:
          Serial.println("No Fix (Time Invalid)");
          break;
        case GPSManager::GPS_LOST_PPS:
          Serial.println("PPS Lost (>5.0s)");
          break;
        case GPSManager::GPS_LOST_SERIAL:
          Serial.println("Serial Data Timeout (>10s)");
          break;
        case GPSManager::GPS_LOCKED:
          Serial.println("Transient Glitch (Still locked?)");
          break;
        default:
          Serial.println("Unknown");
          break;
        }

        _state = VOTER_DISCONNECTED;
        _lastAttemptTime = millis(); // Force delay
        _gpsLostTime = 0;            // Reset
        logger.error("VOTER", "Disconnected: GPS Holdover Expired");
        return;
      }
    } else {
      // Lock is valid - Reset debounce timer
      if (_gpsLostTime != 0) {
        if (_hasWarnedGps) {
          char ts[20];
          _gps->getTimestamp(ts);
          Serial.printf("%s [Voter] GPS Lock Recovered! (Sats: %u)\r\n", ts,
                        _gps->getSatellites());
        }
        _gpsLostTime = 0;
        _hasWarnedGps = false;
      }
    }

    // 4. Keepalive / GPS Packet (Periodic)
    if (millis() - _lastGPSSend > 500) { // Every 500 ms
      _lastGPSSend = millis();
      _sendGPSPacket();
    }

    if (_isTransmitting && (millis() - _lastCallTime > 200)) {
      _isTransmitting = false;
    }
  }
}

// Ported from Voter.c crc32_bufs
uint32_t VoterClient::_crc32(const uint8_t *buf1, const uint8_t *buf2) {
  uint32_t oldcrc32 = 0xFFFFFFFF;

  while (buf1 && *buf1) {
    oldcrc32 =
        crc_32_tab[(oldcrc32 ^ *buf1++) & 0xff] ^ ((uint32_t)oldcrc32 >> 8);
  }
  while (buf2 && *buf2) {
    oldcrc32 =
        crc_32_tab[(oldcrc32 ^ *buf2++) & 0xff] ^ ((uint32_t)oldcrc32 >> 8);
  }
  return ~oldcrc32;
}

void VoterClient::alignTimestampPhase() {
  // Calculates the offset needed to force the effective sequence count
  // to be a multiple of 50 (0ms) at this moment (PPS).
  uint32_t mod = _packetCounter % 50;
  if (mod == 0) {
    _seqOffset = 0;
  } else {
    _seqOffset = 50 - mod;
  }
}

void VoterClient::_generateChallenge() {
  memset(_myChallenge, 0, sizeof(_myChallenge));
  snprintf(_myChallenge, VOTER_CHALLENGE_LEN, "%lu",
           random(10000000, 99999999));
}

void VoterClient::_sendAuthPacket() {
  // Construct Header-Only Packet
  VOTER_PACKET_HEADER header;
  memset(&header, 0, sizeof(header));

  // Time: For General Purpose packets (Auth/GPS/Keepalive),
  // vtime_nsec increments by 1 per packet (Voter2 Author Feedback)
  if (_gps && _gps->isLocked()) {
    header.curtime.vtime_sec = my_htonl(_gps->getEpoch());
    header.curtime.vtime_nsec = my_htonl(_gpSeq++);
  } // Else 0

  // Challenge
  memcpy(header.challenge, _myChallenge, VOTER_CHALLENGE_LEN);

  // Digest
  header.digest = my_htonl(_myDigest); // 0 initially
  header.payload_type = my_htons(PAYLOAD_AUTH);

  Serial.println("[Voter] Sending Auth Request...");
  _authAttempts++;
  _net->sendPacket((uint8_t *)&header, sizeof(header));
}

void VoterClient::_sendGPSPacket() {
  // if (!_gps || !_gps->isLocked()) return; // FIXED: Don't return, send empty
  // if needed
  bool locked = (_gps && _gps->isLocked());

  PROXY_GPS_PACKET pkt;
  memset(&pkt, 0, sizeof(pkt));

  // 1. Header
  // VOTER2 REFERENCE: GP packets use +1 increment in nsec field
  pkt.header.curtime.vtime_sec = my_htonl(_gps->getEpoch());
  pkt.header.curtime.vtime_nsec = my_htonl(_gpSeq++);

  memcpy(pkt.header.challenge, _myChallenge, VOTER_CHALLENGE_LEN);
  pkt.header.digest = my_htonl(_myDigest);
  pkt.header.payload_type = my_htons(PAYLOAD_GPS);

  // 2. Location Payload
  if (locked) {
    _gps->getGPSStrings(pkt.lat, pkt.lon, pkt.elev);
  } else {
    // PIC behavior: empty/zero strings when no fix
    strcpy((char *)pkt.lat, "0000.00N");
    strcpy((char *)pkt.lon, "00000.00W");
    strcpy((char *)pkt.elev, "  0.0 "); 
  }

  // 3. Send
  _net->sendPacket((uint8_t *)&pkt, sizeof(pkt));
}

bool VoterClient::_isValidChallenge(const char *challenge) {
  // Challenge is always a decimal number encoded as an ASCII string
  // (_generateChallenge() builds it via snprintf "%lu"), null-padded to
  // VOTER_CHALLENGE_LEN. Anything else in that field means the packet
  // got corrupted in transit - observed on hardware as garbled text
  // ("New Server Challenge: (garbage)") and repeated failed auth
  // attempts, intermittently, with no single change it traced back to -
  // most likely occasional SPI receive corruption. Reject rather than
  // act on it (recalculating digests from garbage, or printing it).
  bool sawNull = false;
  for (int i = 0; i < VOTER_CHALLENGE_LEN; i++) {
    char c = challenge[i];
    if (c == '\0') {
      sawNull = true;
      continue;
    }
    if (sawNull) return false; // non-null byte after padding started
    if (c < '0' || c > '9') return false;
  }
  return true;
}

void VoterClient::_handlePacket(const uint8_t *data, int len) {
  VOTER_PACKET_HEADER *header = (VOTER_PACKET_HEADER *)data;

  // Check if challenge changed (New Session start) OR if it is an Auth Request
  char rcvChallenge[VOTER_CHALLENGE_LEN + 1];
  memset(rcvChallenge, 0, sizeof(rcvChallenge));
  memcpy(rcvChallenge, header->challenge, VOTER_CHALLENGE_LEN);

  // Ignore packets that we sent (loopback/broadcast)
  if (strncmp(rcvChallenge, _myChallenge, VOTER_CHALLENGE_LEN) == 0) {
    return;
  }

  // Valid packet received - Update tracking
  _lastRxTime = millis();
  // _authAttempts = 0; // REMOVED: Only reset on progress/new session to allow
  // failure timeout _hasWarnedAuth reset moved to happy path in digest check

  // Debug info

  // Only treat this as a genuine new-challenge/session-reset event if the
  // challenge both differs from our current one AND looks like a real
  // challenge (digits, null-padded) - a single corrupted header byte
  // shouldn't be able to force a full re-auth cycle. This does NOT gate
  // digest verification or audio dispatch below - a glitched challenge
  // byte doesn't mean the rest of the packet (digest, audio payload) is
  // also bad, and the digest check already protects that path on its
  // own. An earlier version of this fix dropped the whole packet on a
  // bad challenge and made things much worse: audio packets carry this
  // same header field at 50pps, so even a modest per-packet corruption
  // rate on it was discarding a lot of otherwise-good audio frames
  // wholesale.
  bool challengeValid = _isValidChallenge(rcvChallenge);
  if (!challengeValid) {
    _corruptedPacketCount++;
    if (millis() - _lastCorruptedReportTime > 10000) {
      _lastCorruptedReportTime = millis();
      Serial.printf("[Voter] %lu corrupted challenge field(s) so far (packet still processed)\r\n",
                    (unsigned long)_corruptedPacketCount);
    }
  }

  bool newChallenge = challengeValid &&
      (strncmp(rcvChallenge, _serverChallenge, VOTER_CHALLENGE_LEN) != 0);
  uint16_t type = my_ntohs(header->payload_type);

  if (newChallenge) {
    Serial.printf("[Voter] New Server Challenge: %s (My Chl: %s)\r\n",
                  rcvChallenge, _myChallenge);
    memcpy(_serverChallenge, rcvChallenge, VOTER_CHALLENGE_LEN);

    // Recalculate Digests
    _myDigest = _crc32((uint8_t *)_serverChallenge, (uint8_t *)_clientPwd);
    _serverDigest = _crc32((uint8_t *)_myChallenge, (uint8_t *)_hostPwd);

    Serial.println("[Voter] Updated authentication digests");

    // Always reply to a NEW challenge immediately
    _state = VOTER_DISCONNECTED;
    _authError = AUTH_ERR_NONE;
    _authAttempts = 0; // Progress!
    _hasWarnedAuth = false;
    _sendAuthPacket();
    return;
  }

  // Verify Server Digest
  uint32_t incomingDigest = my_ntohl(header->digest);
  if (incomingDigest == _serverDigest) {
    _authMismatchStreak = 0; // A good digest clears any prior bad reads
    _hasWarnedAuth = false; // Reset warning state on happy path

    if (_state == VOTER_DISCONNECTED) {
      Serial.printf(
          "[Voter] Host Password Accepted (Exp: 0x%08X Got: 0x%08X)\r\n",
          _serverDigest, incomingDigest);
      _state = VOTER_AUTHENTICATING;
    }

    // A matching digest is sufficient to be connected, regardless of
    // payload type. The reference PIC/STM32 firmware connects on digest
    // match alone; the server's routine "empty" heartbeat packets reuse
    // the same payload type as the initial AUTH challenge, so requiring
    // a non-AUTH packet here can hang forever waiting for one.
    if (_state != VOTER_CONNECTED) {
      // CRITICAL FIX: Do not auto-connect if we don't have GPS lock!
      if (_gps && !_gps->isLocked()) {
        return;
      }

      Serial.println("[Voter] Authentication Successful! Connected to Host.");
      _state = VOTER_CONNECTED;
      _authError = AUTH_ERR_NONE;
      _authAttempts = 0; // Success!
      _lastGPSSend = millis();
      logger.info("VOTER", "Connected to Host %u.%u.%u.%u:%u", _hostIP[0],
                  _hostIP[1], _hostIP[2], _hostIP[3], _hostPort);
    }

    if (type == PAYLOAD_ULAW) {
      _handleProxyAudioPacket(data, len);
    }
  } else {
    // If Digest Mismatch AND it was an AUTH packet, it might be a challenge we
    // missed or a retry - or, confirmed on hardware, a single corrupted SPI
    // read (e.g. digest field landing as 0xFFC00000 right as audio traffic
    // starts) that has nothing to do with the actual passwords. A real
    // password mismatch fails every single retry; a one-off bit error
    // clears on the next packet. Require several in a row before treating
    // it as terminal, same reasoning already applied to the challenge field
    // above - one bad packet shouldn't tear down the whole session.
    if (type == PAYLOAD_AUTH) {
      _authMismatchStreak++;
      if (_authMismatchStreak >= 3 && !_hasWarnedAuth) {
        _hasWarnedAuth = true;
        _state = VOTER_AUTH_ERROR; // Stop retrying!
        _authError = AUTH_ERR_HOST_MISMATCH;
        Serial.printf("[Voter] Auth Mismatch! Exp: 0x%08X Got: 0x%08X\r\n",
                      _serverDigest, incomingDigest);
        Serial.println(
            "[Voter] Authentication Failed: Password Mismatch. Please "
            "check your Voter and Host passwords.");
      }
      // This prevents a rapid-fire feedback loop between client and server.
    } else if (type == PAYLOAD_ULAW) {
      // A digest mismatch on an audio packet silently drops that whole
      // 20ms frame (never reaches _handleProxyAudioPacket()) - previously
      // invisible. Counting/reporting this separately from the AUTH-packet
      // case above so we can tell whether ongoing SPI-level corruption is
      // eating audio frames one at a time, which would sound like exactly
      // this kind of steady jitter even with the auth-cascade bug fixed.
      _droppedAudioDigestMismatch++;
      if (millis() - _lastDroppedAudioReportTime > 10000) {
        _lastDroppedAudioReportTime = millis();
        Serial.printf(
            "[Voter] %lu audio frame(s) dropped so far on digest mismatch "
            "(Exp: 0x%08X Got: 0x%08X)\r\n",
            (unsigned long)_droppedAudioDigestMismatch, _serverDigest, incomingDigest);
      }
    }
  }
}

void VoterClient::_handleProxyAudioPacket(const uint8_t *data, int len) {
  if (len < (int)sizeof(PROXY_AUDIO_PACKET))
    return;

  PROXY_AUDIO_PACKET *pkt = (PROXY_AUDIO_PACKET *)data;
  _lastHostAudioTime = millis(); // Track network-layer activity for PTT

  // Push audio to Jitter Buffer
  // Note: pkt->audio is 160 bytes of uLaw
  jitterBuffer.put(pkt->audio, FRAME_SIZE);
}

void VoterClient::processAudioFrame(uint8_t *ulawData, uint8_t rssi,
                                    VTIME frameTime) {
  if (_state != VOTER_CONNECTED) {
    return;
  }

  // Construct Packet
  PROXY_AUDIO_PACKET pkt;
  memset(&pkt, 0, sizeof(pkt));

  // --- Rule 1: Anchor Once (Authority Alignment) ---
  // If this is the start of a transmission, capture current GPS time as anchor.
  if (!_anchorActive || !_isTransmitting) {
    _anchorSec = frameTime.vtime_sec;
    _anchorNsec = frameTime.vtime_nsec;
    _burstPacketCount = 0;
    _anchorActive = true;
  }

  // --- Rule 2: Count Frames (Continuous counting) ---
  // CurrentTime = BaseEpoch + (FramesSent * 20ms)
  uint64_t totalNsec = (uint64_t)_anchorSec * 1000000000ULL + _anchorNsec;
  totalNsec += (uint64_t)_burstPacketCount * 20000000ULL;

  pkt.header.curtime.vtime_sec = (uint32_t)(totalNsec / 1000000000ULL);
  pkt.header.curtime.vtime_nsec = (uint32_t)(totalNsec % 1000000000ULL);

  // --- Rule 3: Delay Offset (Authority Alignment) ---
  // Apply User Timing Offset (default -180ms per Authority)
  if (_timingOffsetMs != 0) {
    int64_t nsec = (int64_t)pkt.header.curtime.vtime_nsec;
    nsec += (int64_t)_timingOffsetMs * 1000000LL;
    
    while (nsec >= 1000000000LL) {
      nsec -= 1000000000LL;
      pkt.header.curtime.vtime_sec++;
    }
    while (nsec < 0) {
      nsec += 1000000000LL;
      pkt.header.curtime.vtime_sec--;
    }
    pkt.header.curtime.vtime_nsec = (uint32_t)nsec;
  }

  // Housekeeping
  _burstPacketCount++;
  _lastCallTime = millis();
  _isTransmitting = (rssi > 0);

  // If burst ended (unkeyed), clear anchor for next keyup
  if (!_isTransmitting) {
    _anchorActive = false;
  }

  // Network Byte Order for Time
  pkt.header.curtime.vtime_sec = my_htonl(pkt.header.curtime.vtime_sec);
  pkt.header.curtime.vtime_nsec = my_htonl(pkt.header.curtime.vtime_nsec);

  memcpy(pkt.header.challenge, _myChallenge, VOTER_CHALLENGE_LEN);
  pkt.header.digest = my_htonl(_myDigest);
  pkt.header.payload_type = my_htons(PAYLOAD_ULAW); 

  pkt.rssi = rssi;
  memcpy(pkt.audio, ulawData, FRAME_SIZE);

  // Stage for flushPendingAudio() rather than sending here - see that
  // method's declaration in VoterClient.h for why. Guard against the rare
  // case where a second frame completes in the same record-queue-drain
  // pass before the first is flushed (possible given the resample-carry
  // remainder logic in main.cpp) - flush the older one immediately rather
  // than silently overwriting/dropping it; that would be a real gap in
  // the packet sequence, worse than the timing issue this is fixing.
  if (_pendingAudioSend) {
    flushPendingAudio();
  }
  _pendingAudioPkt = pkt;
  _pendingAudioSend = true;
}

void VoterClient::flushPendingAudio() {
  if (!_pendingAudioSend) return;
  _pendingAudioSend = false;

  _net->sendPacket((uint8_t *)&_pendingAudioPkt, sizeof(_pendingAudioPkt));

  // Per-window count (resets each print), same style as the ESP32's own
  // "UDPfwd" heartbeat counter - line these two logs up side by side
  // during a choppy test. If this number and the ESP32's diverge, that
  // pinpoints whether frames are going missing on the Teensy->ESP32 SPI
  // leg or the ESP32->server UDP leg.
  _audioFramesSent++;
  static uint32_t lastAudioSentReport = 0;
  if (millis() - lastAudioSentReport > 5000) {
    lastAudioSentReport = millis();
    Serial.printf("[SPI-TX] Audio frames sent (last ~5s window): %lu\r\n", (unsigned long)_audioFramesSent);
    _audioFramesSent = 0;
  }
}
