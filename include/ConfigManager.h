#ifndef CONFIG_MANAGER_H
#define CONFIG_MANAGER_H

#include <Arduino.h>
#include <EEPROM.h>
#include <NativeEthernet.h>

// Magic Header to detect valid config
#define CONFIG_MAGIC 0xCAFEBABE
#define CONFIG_VERSION 8

// COS/Squelch Modes
#define COS_MODE_ALWAYS_ON 0 // Always send RSSI (testing/no squelch)
#define COS_MODE_HARDWARE 1  // Use GPIO pin for COS
#define COS_MODE_DSP 2       // Use DSP noise detection

struct SysConfig {
  uint32_t magic;
  uint32_t version;

  // Identity
  uint8_t mac[6];

  // Voter Host
  uint32_t hostIP; // Stored as uint32 to be generic (Network byte order?)
  uint16_t hostPort;

  // Authentication
  char clientPwd[20];
  char hostPwd[20];

  // Options
  bool useHwRSSI;
  uint8_t cosMode;          // COS_MODE_* constant
  uint8_t dspSquelchThresh; // 0-255, threshold for DSP squelch
  uint8_t rxGain;           // 0-15, SGTL5000 Line In Level
  uint8_t inputSource;      // AUDIO_INPUT_LINEIN or AUDIO_INPUT_MIC

  // RSSI Calibration (Raw ADC 0-1023)
  uint16_t rssiMin;
  uint16_t rssiMax;

  // DSP Calibration
  float dspCalib; // Default 13.0f

  // Audio Filtering
  bool enablePLFilter; // 300Hz HPF (Block PL)
  bool enableDeemp;    // De-emphasis LPF
};

class ConfigManager {
public:
  SysConfig data;

  ConfigManager();
  void begin();

  void load();
  void save();
  void resetDefaults();

  // Helper to get formatted IP
  IPAddress getHostIP();
  void setHostIP(IPAddress ip);
};

#endif
