# Data Structures

## Voter Protocol (Network)

### Packet Header
Standard header for all Voter/Cisco protocol packets. Big Endian (Network Byte Order).
```cpp
typedef struct {
    uint16_t   payload_type;  // 0x0000=Audio/Auth, 0x0001=GPS? Varies by implementation.
                              // Check VoterProtocol.h for specific mappings.
    uint16_t   seq;           // Sequence Number (Optional/Unused in simple clients)
    VTIME      curtime;       // GPS Timestamp (Sec, NSec)
    uint32_t   digest;        // CRC32(Challenge + Password)
    uint8_t    challenge[10]; // ASCII Numeric string challenge from server
} VOTER_PACKET_HEADER;
```

### Audio Payload (Type 0)
```cpp
typedef struct {
    VOTER_PACKET_HEADER header;
    uint8_t   rssi;           // 0-255 (0=Squelched, 255=Strong)
    uint8_t   audio[160];     // uLaw encoded audio (20ms frame)
} PROXY_AUDIO_PACKET;
```

## Configuration (EEPROM)

Managed by `ConfigManager`. Serialized struct:
```cpp
struct ConfigData {
    uint8_t  version;           // Config Version check
    
    // Network Target
    uint32_t hostIP;            // IPAddress stored as u32
    uint16_t hostPort;          // Default 1667
    
    // Auth
    char     clientPwd[20];
    char     hostPwd[20];
    
    // Audio / Radio
    uint8_t  rxGain;            // 0-15 (SGTL5000 LineIn Level)
    uint8_t  inputSource;       // 0=Line, 1=Mic
    uint16_t rssiMin;           // ADC value for 0% Signal
    uint16_t rssiMax;           // ADC value for 100% Signal
    
    // Features
    bool     useHwRSSI;         // true=ADC, false=DSP Noise
    uint8_t  cosMode;           // 0=Always, 1=HW, 2=DSP
    uint8_t  dspSquelchThresh;  // 0-255
    
    // Filters
    bool     enablePLFilter;
    bool     enableDeemp;
    float    dspCalib;          // Tuning factor for DSP RSSI
};
```
