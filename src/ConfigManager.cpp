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
  data.hostname[0] = '\0'; // Empty hostname by default
  data.hostIP = (uint32_t)IPAddress(10, 10, 10, 40);
  data.hostPort = 1667;

  strcpy(data.clientPwd, "teensy123");
  strcpy(data.hostPwd, "K5LMA146980");
  strcpy(data.wifiSSID, "ImWatchinYou");
  strcpy(data.wifiPass, "n0Password");
  data.dnsServerIP = 0; // Use DHCP default DNS

  // Static IP Configuration (Ethernet)
  data.useStaticIP = false;                             // Default to DHCP
  data.staticIP = (uint32_t)IPAddress(10, 10, 50, 200); // Fallback IP
  data.subnetMask = (uint32_t)IPAddress(255, 255, 255, 0);
  data.gateway = (uint32_t)IPAddress(10, 10, 50, 1);
  data.staticDNS = 0; // Use gateway as DNS

  data.useHwRSSI = true;  // Default to Hardware RSSI (ADC checked good)
  data.cosInvert = false; // Default: Active Low (Standard for Open Collector)
  data.cosMode = COS_MODE_HARDWARE; // Default to Hardware COS
  data.inputSource = 0;             // Line In

  data.rssiMax = 1000;
  // DSP Calibration: 50.0f allows for more noise before RSSI drops to 0.
  data.dspCalib = 50.0f; // Was 13.0f

  data.dspSquelchThresh = 30; // Default DSP squelch threshold

  // Directional Defaults
  data.radioRxAnalogGain = 6;    // Default Gain (Pre-amp)
  data.radioRxDigitalGainPct = 100; // 100% = Unity
  data.radioTxMasterGainPct = 50;  // 50% = Nominal

  data.inputSource = 0;       // Default to Line In (AUDIO_INPUT_LINEIN = 0)

  // DSP Filters (CRITICAL - were missing!)
  data.enablePLFilter =
      true; // Enable 300Hz HPF (blocks PL tones & low-freq noise)
  data.enableDeemp = true; // Enable de-emphasis (reduces high-freq noise)
  data.timingOffsetMs = 0; // Default to actual sample time

  // PTT Defaults
  data.pttInvert = true; // Default: Active Low (Industry Standard)
  data.pttTailMs = 500;  // Default: 500ms tail (Eliminates flickering/popping)

  save();
  Serial.println("[Config] Reset to Defaults\r");
}

IPAddress ConfigManager::getHostIP() { return IPAddress(data.hostIP); }

void ConfigManager::setHostIP(IPAddress ip) { data.hostIP = (uint32_t)ip; }

const char *ConfigManager::getHostname() { return data.hostname; }

void ConfigManager::setHostname(const char *hostname) {
  if (hostname) {
    strncpy(data.hostname, hostname, sizeof(data.hostname) - 1);
    data.hostname[sizeof(data.hostname) - 1] = '\0'; // Ensure null termination
  } else {
    data.hostname[0] = '\0';
  }
}
