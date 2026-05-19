# Data Structures

## Voter Protocol (Network)

Defined in [VoterProtocol.h](file:///C:/Users/mikec/Documents/Projects/VOTER/TeensyVoter/include/VoterProtocol.h). Packets are strictly packed and transmit values in Network Byte Order (Big Endian).

### GPS Time Struct
```cpp
typedef struct {
    uint32_t   vtime_sec;     // Seconds since Epoch
    uint32_t   vtime_nsec;    // Nanoseconds remainder / Sequence counter
} VTIME;
```

### Packet Header
Standard header for all Voter protocol packets.
```cpp
typedef struct {
    VTIME      curtime;       // GPS Timestamp (Sec, NSec)
    uint8_t    challenge[10]; // ASCII numeric string challenge from server
    uint32_t   digest;        // CRC32 digest of (Challenge + Password)
    uint16_t   payload_type;  // 0=Auth, 1=uLaw Audio, 2=GPS, 5=Ping
} VOTER_PACKET_HEADER;
```

### Audio Payload (Type 1 - uLaw Audio)
Sent every 20ms during active transmissions.
```cpp
typedef struct {
    VOTER_PACKET_HEADER header;
    uint8_t   rssi;           // RSSI value (bit 7: COR active/inactive, bits 0-6: signal level)
    uint8_t   audio[160];     // 160 samples of G.711 u-law compressed audio (20ms)
} PROXY_AUDIO_PACKET;
```

### GPS Location Payload (Type 2 - Keepalive)
Sent periodically (every 500ms) to update location details.
```cpp
typedef struct {
    VOTER_PACKET_HEADER header;
    char      lat[9];         // NMEA Latitude (e.g., "0000.00N")
    char      lon[10];        // NMEA Longitude (e.g., "00000.00W")
    char      elev[7];        // Elevation string (e.g., "  0.0 ")
} PROXY_GPS_PACKET;
```

---

## Configuration (EEPROM)

Managed by [ConfigManager](file:///C:/Users/mikec/Documents/Projects/VOTER/TeensyVoter/include/ConfigManager.h) and loaded/saved directly to Teensy EEPROM using byte-packing `#pragma pack(push, 1)`.

```cpp
struct SysConfig {
  // 32-bit fields
  uint32_t magic;                 // Magic marker (0x564F5452 - "VOTR")
  uint32_t version;               // Config version check
  uint32_t hostIP;                // Server IP address
  uint32_t staticIP;              // Teensy static IP fallback (Ethernet)
  uint32_t subnetMask;            // Subnet mask (Ethernet)
  uint32_t gateway;               // Gateway IP (Ethernet)
  uint32_t staticDNS;             // DNS IP fallback (Ethernet)
  uint32_t dnsServerIP;           // DNS Server IP (WiFi)
  float    dspCalib;              // Tuning factor for DSP RSSI
  uint32_t syslogIP;              // Syslog destination IP

  // 16-bit fields
  uint16_t hostPort;              // Server port (default 1667)
  uint16_t rssiMin;               // ADC value representing 0% signal strength
  uint16_t rssiMax;               // ADC value representing 100% signal strength
  int16_t  timingOffsetMs;        // Delay adjustment offset (default -180ms)
  uint16_t pttTailMs;             // Transmitter tail time (default 500ms)
  uint16_t syslogPort;            // Syslog port (default 514)

  // 8-bit / bool fields
  uint8_t  mac[6];                // Device MAC address (custom override)
  uint8_t  cosMode;               // COS Squelch Mode: 0=Always On, 1=HW, 2=DSP
  uint8_t  dspSquelchThresh;      // DSP squelch threshold level (0-255)
  uint8_t  radioRxAnalogGain;     // SGTL5000 LineIn analog level (0-15)
  uint8_t  radioRxDigitalGainPct; // Software boost percentage (100% = unity)
  uint8_t  radioTxMasterGainPct;  // Master output level scaling (0-100%)
  uint8_t  inputSource;           // 0=Line In, 1=Mic In
  bool     useStaticIP;           // Use Static IP instead of DHCP (Ethernet)
  bool     useHwRSSI;             // true=Hardware ADC RSSI, false=DSP Noise RSSI
  bool     cosInvert;             // Invert COS active polarity (true=Active High)
  bool     enablePLFilter;        // Enable 300Hz-2400Hz voice bandpass filter
  bool     enableDeemp;           // Enable 75µs de-emphasis filtering
  bool     pttInvert;             // Invert PTT output polarity (true=Active Low)
  bool     useSyslog;             // Enable logging to syslog server

  // Strings (byte arrays)
  char     hostname[64];          // Client hostname
  char     syslogHostname[64];    // FQDN or IP for syslog
  char     clientPwd[20];         // Authentication client password
  char     hostPwd[20];           // Authentication host password
  char     wifiSSID[32];          // WiFi network name
  char     wifiPass[64];          // WiFi network password
};
```
