#include "ConfigManager.h"
#include <Audio.h>

ConfigManager::ConfigManager() {
  // Constructor
}

void ConfigManager::begin() { load(); }

void ConfigManager::load() {
  EEPROM.get(0, data);
  if (data.magic != CONFIG_MAGIC || data.version != CONFIG_VERSION) {
    Serial.printf("[Config] Invalid Magic/Version (Got v%u, Exp v%u) - "
                  "Resetting Defaults\n",
                  data.version, CONFIG_VERSION);
    resetDefaults();
  } else {
    Serial.println("[Config] Settings Loaded from EEPROM");
  }
}

void ConfigManager::save() {
  data.magic = CONFIG_MAGIC;
  data.version = CONFIG_VERSION;
  EEPROM.put(0, data);
  Serial.println("[Config] Saved to EEPROM");
}

void ConfigManager::resetDefaults() {
  memset(&data, 0, sizeof(data));

  // Default MAC (Teensy 4.1 reads real MAC from OCOTP usually, but we will
  // store override) Actually, NativeEthernet reads the hardware MAC. We only
  // need to store it if we want to spoof it. Let's set a dummy default.
  data.mac[0] = 0xDE;
  data.mac[1] = 0xAD;
  data.mac[2] = 0xBE;
  data.mac[3] = 0xEF;
  data.mac[4] = 0xFE;
  data.mac[5] = 0xED;

  // Default Host: 10.10.10.42 : 667
  data.hostIP = (uint32_t)IPAddress(10, 10, 10, 42);
  data.hostPort = 1667;

  strcpy(data.clientPwd, "teensyvoter");
  strcpy(data.hostPwd, "K5LMA146980");

  data.useHwRSSI = true; // Default to Hardware RSSI (ADC checked good)
  data.inputSource = 0;  // Line In

  data.rssiMax = 1000;
  // DSP Calibration: 50.0f allows for more noise before RSSI drops to 0.
  data.dspCalib = 50.0f; // Was 13.0f

  data.dspSquelchThresh = 30; // Default DSP squelch threshold
  data.rxGain = 6;            // Default Gain (User found 5-6 good)
  data.inputSource = 0;       // Default to Line In (AUDIO_INPUT_LINEIN = 0)

  // DSP Filters (CRITICAL - were missing!)
  data.enablePLFilter =
      true; // Enable 300Hz HPF (blocks PL tones & low-freq noise)
  data.enableDeemp = true; // Enable de-emphasis (reduces high-freq noise)

  save();
  Serial.println("[Config] Reset to Defaults");
}

IPAddress ConfigManager::getHostIP() { return IPAddress(data.hostIP); }

void ConfigManager::setHostIP(IPAddress ip) { data.hostIP = (uint32_t)ip; }
