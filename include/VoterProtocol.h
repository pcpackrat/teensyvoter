#ifndef VOTER_PROTOCOL_H
#define VOTER_PROTOCOL_H

#include <Arduino.h>

// --- Protocol Constants ---
#define VOTER_CHALLENGE_LEN 10
#define FRAME_SIZE 160      // 20ms of 8kHz audio
#define MAX_BUFFER_SIZE 512 // Enough for header + payload

// --- Payload Types ---
#define PAYLOAD_AUTH 0
#define PAYLOAD_ULAW 1
#define PAYLOAD_GPS 2
#define PAYLOAD_ADPCM 3 // Not planned for initial port
#define PAYLOAD_PING 5

// --- Structures ---
// Ensure strict packing to match the wire format of the original PIC firmware
#pragma pack(push, 1)

typedef struct {
  uint32_t vtime_sec;
  uint32_t vtime_nsec;
} VTIME;

typedef struct {
  VTIME curtime;
  uint8_t challenge[VOTER_CHALLENGE_LEN];
  uint32_t digest;
  uint16_t payload_type;
} VOTER_PACKET_HEADER;

// RSSI + Audio Payload Wrapper
typedef struct {
  VOTER_PACKET_HEADER header;
  uint8_t rssi;
  uint8_t audio[FRAME_SIZE]; // Exact 160 bytes for ULAW (Total packet 185)
} PROXY_AUDIO_PACKET;

// GPS Payload Wrapper
typedef struct {
  VOTER_PACKET_HEADER header;
  char lat[9];
  char lon[10];
  char elev[7];
} PROXY_GPS_PACKET;

#pragma pack(pop)

#endif
