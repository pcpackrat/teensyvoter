#ifndef CONFIG_MANAGER_H
#define CONFIG_MANAGER_H

#include <Arduino.h>
#include <EEPROM.h>
#include <NativeEthernet.h>

// Magic Header to detect valid config
#define CONFIG_MAGIC 0x564F5452 // "VOTR"
#define CONFIG_VERSION 22       // Natural alignment version

// COS/Squelch Modes
#define COS_MODE_ALWAYS_ON 0 // Always send RSSI (testing/no squelch)
#define COS_MODE_HARDWARE 1  // Use GPIO pin for COS
#define COS_MODE_DSP 2       // Use DSP noise detection

struct SysConfig {
  // 32-bit fields
  uint32_t magic;
  uint32_t version;
  uint32_t hostIP;
  uint32_t staticIP;
  uint32_t subnetMask;
  uint32_t gateway;
  uint32_t staticDNS;
  uint32_t dnsServerIP;
  float dspCalib;

  // 16-bit fields
  uint16_t hostPort;
  uint16_t rssiMin;
  uint16_t rssiMax;
  int16_t timingOffsetMs;
  uint16_t pttTailMs;

  // 8-bit / bool fields
  uint8_t mac[6];
  uint8_t cosMode;
  uint8_t dspSquelchThresh;
  uint8_t radioRxAnalogGain;
  uint8_t radioRxDigitalGainPct;
  uint8_t radioTxMasterGainPct;
  uint8_t inputSource;
  bool useStaticIP;
  bool useHwRSSI;
  bool cosInvert;
  bool enablePLFilter;
  bool enableDeemp;
  bool pttInvert;

  // Strings (byte arrays)
  char hostname[64];
  char clientPwd[20];
  char hostPwd[20];
  char wifiSSID[32];
  char wifiPass[64];
};

class ConfigManager {
public:
  ConfigManager();
  void begin();
  void load();
  void save();
  void resetDefaults();

  // Helpers
  IPAddress getHostIP();
  void setHostIP(IPAddress ip);
  const char *getHostname();
  void setHostname(const char *hostname);

  SysConfig data;
};

#endif
