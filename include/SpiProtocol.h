#ifndef SPI_PROTOCOL_H
#define SPI_PROTOCOL_H

#include <stdint.h>

// Commands (Teensy -> ESP)
#define CMD_NOP 0x00
#define CMD_GET_STATUS 0x01
#define CMD_SET_CONFIG 0x02 // SSID, PASS
#define CMD_GET_IP 0x03     // Returns 4 bytes IPv4
#define CMD_SEND_UDP 0x10   // + Target Info

// Responses/Status Flags
#define STATUS_IDLE 0x00
#define STATUS_HAS_DATA 0x01  // ESP has received data for Teensy
#define STATUS_WIFI_CONN 0x02 // WiFi is Connected
#define STATUS_ERROR 0x80

// Command Packets Structures (Conceptual)

// CMD_SEND_UDP Format:
// [CMD (1)] [LEN_HI] [LEN_LO] [IP1] [IP2] [IP3] [IP4] [PORT_HI] [PORT_LO]
// [DATA...]

// CMD_SET_CONFIG Format:
// [CMD (1)] [SSID_LEN (1)] [SSID...] [PASS_LEN (1)] [PASS...]

// Reading Data (Teensy Reads)
// Send [0x00] continuously to clock out STATUS byte.
// If STATUS_HAS_DATA bit is set:
// Continue clocking to get: [LEN_HI] [LEN_LO] [DATA...]

#endif
