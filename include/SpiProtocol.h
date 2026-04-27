#ifndef SPI_PROTOCOL_H
#define SPI_PROTOCOL_H

#include <stdint.h>

// Commands (Teensy -> ESP)
#define CMD_NOP 0x00
#define CMD_GET_STATUS 0x01
#define CMD_SET_CONFIG 0x02 // SSID, PASS
#define CMD_GET_IP 0x03     // Returns 4 bytes IPv4
#define CMD_DNS_LOOKUP 0x04 // DNS resolution
#define CMD_GET_DNS 0x05    // Get DNS server IP
#define CMD_PUSH_CONFIG 0x06 // Push full SysConfig struct to ESP32
#define CMD_SEND_UDP 0x10   // + Target Info

// Responses/Status Flags (ESP -> Teensy, first byte of sendbuf)
#define STATUS_IDLE 0x00
#define STATUS_HAS_DATA 0x01       // ESP has UDP data for Teensy
#define STATUS_WIFI_CONN 0x02      // WiFi is Connected
#define STATUS_CONFIG_CMD 0x40     // ESP32 web server config command
#define STATUS_ERROR 0x80

// Config Sub-Commands (ESP32 -> Teensy, payload byte[0] when STATUS_CONFIG_CMD)
#define CFG_CMD_SET_PARAM 0x07     // Set a single config parameter
#define CFG_CMD_SAVE_REBOOT 0x08   // Save EEPROM and reboot
#define CFG_CMD_REQUEST_CONFIG 0x09 // Request fresh config push

// Config Parameter IDs (for CFG_CMD_SET_PARAM)
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

// Command Packets Structures (Conceptual)

// CMD_SEND_UDP Format:
// [CMD (1)] [LEN_HI] [LEN_LO] [IP1] [IP2] [IP3] [IP4] [PORT_HI] [PORT_LO]
// [DATA...]

// CMD_SET_CONFIG Format:
// [CMD (1)] [SSID_LEN (1)] [SSID...] [PASS_LEN (1)] [PASS...]

// CMD_PUSH_CONFIG Format:
// [CMD (1)] [LEN_HI] [LEN_LO] [SysConfig raw bytes...]

// STATUS_CONFIG_CMD Format (ESP32 -> Teensy):
// [STATUS=0x40] [LEN_HI] [LEN_LO] [CFG_SUB_CMD] [DATA...]
// Sub-cmd SET_PARAM: [0x07] [PARAM_ID] [VALUE_LEN] [VALUE...]
// Sub-cmd SAVE_REBOOT: [0x08]
// Sub-cmd REQUEST_CONFIG: [0x09]

// Reading Data (Teensy Reads)
// Send [0x00] continuously to clock out STATUS byte.
// If STATUS_HAS_DATA bit is set:
// Continue clocking to get: [LEN_HI] [LEN_LO] [DATA...]

#endif

