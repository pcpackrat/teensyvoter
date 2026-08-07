#ifndef SPI_PROTOCOL_H
#define SPI_PROTOCOL_H

#include <stdint.h>

#define MAX_SPI_BUF 512

// Commands (Teensy -> ESP)
#define CMD_NOP 0x00
#define CMD_GET_STATUS 0x01
#define CMD_SET_CONFIG 0x02
#define CMD_GET_IP 0x03
#define CMD_DNS_LOOKUP 0x04
#define CMD_GET_DNS 0x05
#define CMD_PUSH_CONFIG 0x06
#define CMD_PUSH_GPS_STATUS 0x07
#define CMD_SEND_UDP 0x10

// Responses/Status Flags (ESP -> Teensy)
#define STATUS_IDLE 0x00
#define STATUS_HAS_DATA 0x01
#define STATUS_CONFIG_CMD 0x40
#define STATUS_ERROR 0x80

// Config Sub-commands (ESP32 -> Teensy)
#define CFG_CMD_SET_PARAM 0x07
#define CFG_CMD_SAVE_REBOOT 0x08
#define CFG_CMD_REQUEST_CONFIG 0x09

// Parameter IDs (for CFG_CMD_SET_PARAM)
#define PARAM_HOST_IP 0x01
#define PARAM_HOST_PORT 0x02
#define PARAM_HOSTNAME 0x03
#define PARAM_CLIENT_PWD 0x04
#define PARAM_HOST_PWD 0x05
#define PARAM_WIFI_SSID 0x06
#define PARAM_WIFI_PASS 0x07
#define PARAM_RX_GAIN 0x10
#define PARAM_TX_GAIN_PCT 0x11
#define PARAM_COS_MODE 0x12
#define PARAM_COS_INVERT 0x13
#define PARAM_DSP_SQUELCH 0x14
#define PARAM_USE_HW_RSSI 0x15
#define PARAM_RSSI_MIN 0x16
#define PARAM_RSSI_MAX 0x17
#define PARAM_PL_FILTER 0x18
#define PARAM_DEEMP 0x19
#define PARAM_PTT_INVERT 0x1A
#define PARAM_PTT_TAIL_MS 0x1B
#define PARAM_DSP_CALIB 0x1C
#define PARAM_INPUT_SOURCE 0x1D
#define PARAM_USE_SYSLOG 0x20
#define PARAM_SYSLOG_IP 0x21
#define PARAM_SYSLOG_PORT 0x22
#define PARAM_SYSLOG_HOSTNAME 0x23
#define PARAM_RX_DIGITAL_GAIN_PCT 0x24
#define PARAM_DNS_SERVER_IP 0x25
#define PARAM_USE_STATIC_IP 0x26
#define PARAM_STATIC_IP 0x27
#define PARAM_SUBNET_MASK 0x28
#define PARAM_GATEWAY 0x29
#define PARAM_STATIC_DNS 0x2A

// GPS telemetry, pushed periodically Teensy -> ESP32 (CMD_PUSH_GPS_STATUS).
// Not part of SysConfig - this is live status, not persisted config, and
// changes too often to piggyback on the config-push path. Defined once
// here (rather than hand-duplicated like SysConfig/SysConfigMirror) so
// there's no risk of the two sides drifting out of byte-alignment.
#pragma pack(push, 1)
struct GpsStatus {
  uint32_t satellites;
  uint32_t ppsJitterUs;
  bool locked;
  bool timeSet;
  char lat[16];
  char lon[16];
  char elev[32];
};
#pragma pack(pop)

#endif
