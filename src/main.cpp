/*
  TeensyVoter - Voter Receiver/Transmitter Firmware
  Platform: Teensy 4.1 + Audio Shield
  Network: ESP32 (SPI) or Native Ethernet

  Dependencies:
  - FNET (NativeEthernet)
  - Audio, SPI, Wire
  - TinyGPSPlus
*/

#include <Arduino.h>
#include <Audio.h>
#include <NativeEthernet.h>
#include <SPI.h>
#include <Wire.h>
#include <arm_math.h> // CMSIS DSP Library for FIR decimator

#include "ConfigManager.h"
#include "DSPProcessor.h"
#include "EspSpiDriver.h"
#include "EthernetDriver.h"
#include "GPSManager.h"
#include "NetworkManager.h"
#include "VoterClient.h"
#include "VoterProtocol.h"
#include "WebServer.h"

#define RSSI_PIN A14 // Connect to voltage divider output (0-3.3V)
#define COS_PIN 41   // Hardware COS input (active HIGH/LOW depending on radio)
#define PIN_DEBUG_TX 3 // Debug / oscilloscope pin
#define PPS_PIN 2      // GPS PPS Input
// #define WIFI_SERIAL Serial5 // REMOVED (Conflicted with GPS)
#define GPS_SERIAL Serial1 // GPS Module RX/TX (Pins 0/1)

// --- Global State ---
float g_headphoneVol = 0.5f;
uint16_t g_digitalGainPct = 100; // Default 100% (Unity)
// bool g_noSignalMode = false;    // Removed
bool g_testToneMode = false; // If true, send 1kHz test tone
int g_forcedRSSI = -1;       // -1 = Disabled, 0-255 = Force Value
float g_testTonePhase = 0.0f;
// bool g_dspSilenceMode = false; // Removed
// float g_postGain = 1.0f; // Default Post-Filter Gain (Removed)

// CMSIS FIR Decimator for 44.1kHz -> 8kHz (factor ~5.5)
// We'll use decimation factor of 6 (44.1kHz / 6 = 7.35kHz, close enough)
#define DECIMATION_FACTOR 6
#define DECIMATOR_NUM_TAPS 48 // FIR filter taps for anti-aliasing
arm_fir_decimate_instance_f32 decimator;
float decimatorState[AUDIO_BLOCK_SAMPLES + DECIMATOR_NUM_TAPS - 1];
float decimatorCoeffs[DECIMATOR_NUM_TAPS];
float decimatorInputBuf[256];
int decimatorInputLen = 0;

// Buffer for 8kHz downsampled audio
int16_t accumulationBuf[512]; // Circular-ish buffer for outgoing samples
int accHead = 0;

// --- Configuration (Managed by ConfigManager) ---
// const char* CLIENT_PWD = "password"; (Removed)
// const char* HOST_PWD   = "bloodhound";
// IPAddress   HOST_IP(192, 168, 1, 100);
// uint16_t    HOST_PORT = 1667;

// --- Audio System ---
AudioInputI2S i2s_in;
AudioMixer4 mixer1;
AudioRecordQueue recordQueue;
AudioOutputI2S i2s_out; // Defined before connections

AudioConnection patchCord1(i2s_in, 0, mixer1, 0); // L -> Mixer
// Right Channel (Unfiltered for now, or unused)
AudioConnection patchCord2(i2s_in, 1, mixer1, 1);

// Monitor Output (Use Mixer Output so we hear what we send)
AudioConnection patchCord4(mixer1, 0, i2s_out, 0);
AudioConnection patchCord5(mixer1, 0, i2s_out, 1);
// AudioConnection patchCord3(mixer1, 0, recordQueue, 0); // REPLACED by LPF
// chain below

AudioAnalyzePeak peak1;
AudioConnection patchCordMeter(mixer1, 0, peak1, 0);

// Anti-Aliasing Filter (LPF) before Downsampling
AudioFilterBiquad lpf1;
AudioConnection patchCord3(mixer1, 0, lpf1, 0);        // Mixer -> LPF
AudioConnection patchCordLPF(lpf1, 0, recordQueue, 0); // LPF -> RecordQueue

AudioControlSGTL5000 sgtl5000_1;

// --- Global Objects ---
// Network Drivers (Auto-select: Ethernet first, fallback to WiFi)
EthernetDriver ethDriver;
EspSpiDriver spiDriver(26, 24, 25); // CS=26 (Uncovered), Ready=24, Reset=25
NetworkManager netMgr;
VoterClient voterClient;
GPSManager gps;
DSPProcessor dsp;
WebServer webServer;
// WebInterface web;
ConfigManager cfg;

byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};

// -----------------------------------------------------------------------------
// Helper: Reset Audio State
// -----------------------------------------------------------------------------
void resetAudioState() {
  Serial.println("[Audio] Resetting Audio State...");
  recordQueue.clear();
}

// -----------------------------------------------------------------------------
// SETUP
// -----------------------------------------------------------------------------
void setup() {
  // 1. Serial & Boot Config
  Serial.begin(9600);
  delay(100);

  // Load Config
  cfg.begin();
  cfg.load();

  // Print Config
  Serial.println("--- Configuration ---");
  Serial.print("Host: ");
  Serial.println(cfg.getHostIP());
  Serial.print("Port: ");
  Serial.println(cfg.data.hostPort);
  Serial.print("WiFi SSID: ");
  Serial.println(cfg.data.wifiSSID);
  Serial.println("---------------------");

  // Init Menus
  // Init Menus
  // Serial.println("[System] Press any key for Menu...");

  // 2. Audio Setup
  AudioMemory(60);

  if (sgtl5000_1.enable()) {
    Serial.println("[Audio] SGTL5000 Audio Shield FOUND & Enabled");
  } else {
    Serial.println("[Audio] ERROR: SGTL5000 Audio Shield NOT DETECTED!");
    Serial.println("[Audio] Check I2C pins and power.");
  }

  sgtl5000_1.volume(g_headphoneVol);

  // Enforce Clean Audio Setup
  sgtl5000_1.adcHighPassFilterEnable();             // Remove DC Offset
  sgtl5000_1.micGain(0);                            // FIXED: 0dB (Preamp OFF)
  sgtl5000_1.autoVolumeControl(0, 0, 0, -18, 0, 0); // Disable AVC hard

  // Force LINE IN for Diagnostic Safety
  sgtl5000_1.inputSelect(AUDIO_INPUT_LINEIN);

  sgtl5000_1.lineInLevel(cfg.data.rxGain); // Line input level (0-15)

  // Start Recording
  mixer1.gain(0, 1.0); // Left Channel (Unity Gain - Reference)
  mixer1.gain(1, 0.0); // Right Channel (MUTED - Floating Pin Noise)
  recordQueue.begin();

  Serial.println("[Audio] SGTL5000 & Queue Initialized");
  Serial.printf("[Audio] Applied RX Gain: %u\r\n", cfg.data.rxGain);

  // Auto-correction removed to allow Gain 0
  // if (cfg.data.rxGain == 0) ...
  sgtl5000_1.lineInLevel(cfg.data.rxGain);

  // Configure Anti-Aliasing Filter (3.6kHz Cutoff for 8kHz Sample Rate
  // compatibility)
  lpf1.setLowpass(0, 3600, 0.707);
  lpf1.setLowpass(1, 3600,
                  0.707); // Total 2 stages for steeper rolloff (24dB/oct)

  lpf1.setLowpass(1, 3600,
                  0.707); // Total 2 stages for steeper rolloff (24dB/oct)

  // 2. Hardware Serial
  // WIFI_SERIAL.begin(115200); // Removed
  GPS_SERIAL.begin(9600);

  Serial.println("[System] Boot Complete: Audio + Network + GPS");

  // 3. Network - Auto-select: Try Ethernet first, fallback to WiFi
  Serial.println("[System] Initializing Network Driver...");

  bool networkReady = false;

  // Try Ethernet first
  Serial.println("[Network] Attempting Ethernet (NativeEthernet)...");

  bool ethSuccess = false;
  if (cfg.data.useStaticIP) {
    // Use static IP configuration
    IPAddress staticIP(cfg.data.staticIP);
    IPAddress subnet(cfg.data.subnetMask);
    IPAddress gw(cfg.data.gateway);
    IPAddress dns(cfg.data.staticDNS);
    ethSuccess = ethDriver.begin(mac, staticIP, subnet, gw, dns);
  } else {
    // Use DHCP
    ethSuccess = ethDriver.begin(mac);
  }

  if (ethSuccess) {
    Serial.println("[Network] ✓ Ethernet Connected!");
    netMgr.begin(&ethDriver, mac);
    networkReady = true;
  } else {
    Serial.println(
        "[Network] ✗ Ethernet unavailable (no cable or DHCP failure)");

    // Fallback to WiFi
    Serial.println("[Network] Falling back to WiFi (ESP32 SPI)...");

    // Give ESP32 time to boot
    Serial.println("[System] Waiting for ESP32 Boot (5s)...");
    delay(5000);

    // Send WiFi Credentials
    Serial.println("[System] Sending WiFi Credentials...");
    spiDriver.setCredentials(cfg.data.wifiSSID, cfg.data.wifiPass);

    // Initialize WiFi driver
    if (spiDriver.begin(mac)) {
      netMgr.begin(&spiDriver, mac);

      // Wait for IP (up to 15s) with Audio Maintenance
      Serial.print("[System] Waiting for WiFi Connection");
      bool ipFound = false;
      for (int i = 0; i < 30; i++) { // 30 * 500ms = 15 seconds
        // Maintain Audio Queue (Prevent Overflow/Crash)
        while (recordQueue.available() > 0) {
          recordQueue.readBuffer();
          recordQueue.freeBuffer();
        }

        // Check IP
        IPAddress myIP = netMgr.getLocalIP();
        if (myIP != IPAddress(0, 0, 0, 0)) {
          Serial.println();
          Serial.print("[System] WiFi Connected! IP: ");
          Serial.println(myIP);
          ipFound = true;
          networkReady = true;
          break;
        }

        Serial.print(".");
        delay(500);
      }

      if (!ipFound) {
        Serial.println(
            "\n[System] WARNING: WiFi Connection Timeout or Retrieval Failed.");
        Serial.println("[System] Check SSID/Password or ESP32 Link.");
      }
    } else {
      Serial.println("[Network] ✗ WiFi driver initialization failed!");
    }
  }

  if (!networkReady) {
    Serial.println("[Network] ERROR: No network available! Check Ethernet "
                   "cable or WiFi settings.");
  }

  delay(100);

  // 4. GPS

  Serial.println("[GPS] Initializing GPS...");
  gps.begin(&GPS_SERIAL, PPS_PIN);

  // 4.1 Config (Moved to top)
  // cfg.begin();

  // Now we have config, resolve hostname if set
  if (cfg.data.hostname[0] != '\0') {
    Serial.print("[DNS] Hostname configured: ");
    Serial.println(cfg.data.hostname);
    Serial.println("[DNS] Resolving...");

    IPAddress resolvedIP;
    if (netMgr.getType() == DRIVER_WIFI_SPI) {
      resolvedIP = spiDriver.resolveHostname(cfg.data.hostname);
    } else if (netMgr.getType() == DRIVER_ETHERNET) {
      resolvedIP = ethDriver.resolveHostname(cfg.data.hostname);
    }

    if (resolvedIP != IPAddress(0, 0, 0, 0)) {
      cfg.setHostIP(resolvedIP);
      Serial.print("[DNS] Using resolved IP: ");
      Serial.println(resolvedIP);
    } else {
      Serial.println("[DNS] Resolution failed, using stored IP");
    }
  }

  netMgr.setTarget(cfg.getHostIP(), cfg.data.hostPort);

  Serial.print("[System] Voter Target: ");
  Serial.print(cfg.getHostIP());
  Serial.printf(":%u\r\n", cfg.data.hostPort);

  // 5. Voter Client

  // 5. Voter Client
  // Serial.println("[Voter] Initializing Protocol Client...");
  voterClient.begin(&netMgr, &gps, cfg.getHostIP(), cfg.data.hostPort,
                    cfg.data.clientPwd, cfg.data.hostPwd);

  // 6. DSP
  dsp.begin();

  // Initialize Decimator
  // Coeffs: Simple averaging for now (1/48).
  // TODO: Replace with proper Low-Pass Sinc coefficients for better
  // anti-aliasing
  for (int i = 0; i < DECIMATOR_NUM_TAPS; i++) {
    decimatorCoeffs[i] = 1.0f / (float)DECIMATOR_NUM_TAPS;
  }

  // Use a safe input block size (multiple of M=6) for Init check. 120 is
  // safe.
  arm_status status = arm_fir_decimate_init_f32(
      &decimator, DECIMATOR_NUM_TAPS, DECIMATION_FACTOR, decimatorCoeffs,
      decimatorState, 120);

  if (status != ARM_MATH_SUCCESS) {
    Serial.print("ERROR: Decimator Init Failed! Code: ");
    Serial.println(status);
  }

  // 7. Web Server
  webServer.setConfig(&cfg);
  webServer.setSystemObjects(&netMgr, &voterClient, &gps);
  webServer.begin();
  IPAddress webIP = Ethernet.localIP();
  // Serial.printf("[Web] Access at http://%u.%u.%u.%u\r\n", webIP[0], webIP[1],
  //               webIP[2], webIP[3]);
}

// Helper for proper input echo
String readStringEcho() {
  String buffer = "";
  while (true) {
    if (Serial.available()) {
      char c = Serial.read();
      // Handle Enter
      if (c == '\r' || c == '\n') {
        Serial.println(); // Newline on terminal
        return buffer;
      }
      // Handle Backspace (0x08 or 0x7F)
      if (c == 0x08 || c == 0x7F) {
        if (buffer.length() > 0) {
          buffer.remove(buffer.length() - 1);
          Serial.print("\b \b"); // Erase character visually
        }
      } else {
        // Normal Character
        buffer += c;
        Serial.print(c);
      }
    }
  }
}

// --- Menu State Machine ---
enum MenuState {
  MENU_MAIN,
  MENU_STATUS,
  MENU_NETWORK,
  MENU_RADIO_CONFIG,
  MENU_DEBUG
};
MenuState g_menuState = MENU_MAIN;

// --- Settings Export/Import ---
void exportSettings() {
  Serial.println("\r\n# TeensyVoter Configuration Export");
  Serial.printf("# Version: %u\r\n", CONFIG_VERSION);
  Serial.println("# Copy this output to save your settings");
  Serial.println("#");

  // Network
  Serial.printf("hostname=%s\r\n", cfg.data.hostname);
  IPAddress hip(cfg.data.hostIP);
  Serial.printf("hostIP=%u.%u.%u.%u\r\n", hip[0], hip[1], hip[2], hip[3]);
  Serial.printf("hostPort=%u\r\n", cfg.data.hostPort);
  Serial.printf("clientPwd=%s\r\n", cfg.data.clientPwd);
  Serial.printf("hostPwd=%s\r\n", cfg.data.hostPwd);
  Serial.printf("wifiSSID=%s\r\n", cfg.data.wifiSSID);
  Serial.printf("wifiPass=%s\r\n", cfg.data.wifiPass);
  IPAddress dns(cfg.data.dnsServerIP);
  Serial.printf("dnsServerIP=%u.%u.%u.%u\r\n", dns[0], dns[1], dns[2], dns[3]);

  // Radio
  Serial.printf("useHwRSSI=%d\r\n", cfg.data.useHwRSSI ? 1 : 0);
  Serial.printf("cosInvert=%d\r\n", cfg.data.cosInvert ? 1 : 0);
  Serial.printf("cosMode=%d\r\n", cfg.data.cosMode);
  Serial.printf("dspSquelchThresh=%d\r\n", cfg.data.dspSquelchThresh);
  Serial.printf("rxGain=%d\r\n", cfg.data.rxGain);
  Serial.printf("inputSource=%d\r\n", cfg.data.inputSource);
  Serial.printf("rssiMin=%d\r\n", cfg.data.rssiMin);
  Serial.printf("rssiMax=%d\r\n", cfg.data.rssiMax);
  Serial.printf("dspCalib=%.1f\r\n", cfg.data.dspCalib);
  Serial.printf("enablePLFilter=%d\r\n", cfg.data.enablePLFilter ? 1 : 0);
  Serial.printf("enableDeemp=%d\r\n", cfg.data.enableDeemp ? 1 : 0);

  Serial.println("#");
  Serial.println("# End of export");
}

bool parseConfigLine(String line) {
  line.trim();
  if (line.length() == 0 || line.startsWith("#"))
    return true; // Skip comments/empty

  int eqPos = line.indexOf('=');
  if (eqPos < 0)
    return false; // Invalid format

  String key = line.substring(0, eqPos);
  String value = line.substring(eqPos + 1);
  key.trim();
  value.trim();

  // Parse and apply settings
  if (key == "hostname") {
    cfg.setHostname(value.c_str());
  } else if (key == "hostIP") {
    IPAddress ip;
    if (ip.fromString(value))
      cfg.setHostIP(ip);
    else
      return false;
  } else if (key == "hostPort") {
    cfg.data.hostPort = value.toInt();
  } else if (key == "clientPwd") {
    strncpy(cfg.data.clientPwd, value.c_str(), sizeof(cfg.data.clientPwd) - 1);
  } else if (key == "hostPwd") {
    strncpy(cfg.data.hostPwd, value.c_str(), sizeof(cfg.data.hostPwd) - 1);
  } else if (key == "wifiSSID") {
    strncpy(cfg.data.wifiSSID, value.c_str(), sizeof(cfg.data.wifiSSID) - 1);
  } else if (key == "wifiPass") {
    strncpy(cfg.data.wifiPass, value.c_str(), sizeof(cfg.data.wifiPass) - 1);
  } else if (key == "dnsServerIP") {
    IPAddress dns;
    if (dns.fromString(value))
      cfg.data.dnsServerIP = (uint32_t)dns;
    else
      return false;
  } else if (key == "useHwRSSI") {
    cfg.data.useHwRSSI = (value.toInt() != 0);
  } else if (key == "cosInvert") {
    cfg.data.cosInvert = (value.toInt() != 0);
  } else if (key == "cosMode") {
    cfg.data.cosMode = value.toInt();
  } else if (key == "dspSquelchThresh") {
    cfg.data.dspSquelchThresh = value.toInt();
  } else if (key == "rxGain") {
    cfg.data.rxGain = value.toInt();
  } else if (key == "inputSource") {
    cfg.data.inputSource = value.toInt();
  } else if (key == "rssiMin") {
    cfg.data.rssiMin = value.toInt();
  } else if (key == "rssiMax") {
    cfg.data.rssiMax = value.toInt();
  } else if (key == "dspCalib") {
    cfg.data.dspCalib = value.toFloat();
  } else if (key == "enablePLFilter") {
    cfg.data.enablePLFilter = (value.toInt() != 0);
  } else if (key == "enableDeemp") {
    cfg.data.enableDeemp = (value.toInt() != 0);
  } else {
    return false; // Unknown key
  }

  return true;
}

void importSettings() {
  Serial.println("\r\n--- Import Settings ---");
  Serial.println("Paste settings below (empty line to finish):");
  Serial.println();

  int imported = 0;
  int errors = 0;

  while (true) {
    // Wait for input with timeout
    uint32_t startTime = millis();
    String line = "";
    while (millis() - startTime < 30000) { // 30 second timeout per line
      if (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
          if (line.length() == 0) {
            // Empty line = done
            goto import_done;
          }
          break; // Process this line
        }
        line += c;
      }
    }

    if (line.length() == 0)
      break; // Timeout or empty

    // Parse line
    if (parseConfigLine(line)) {
      imported++;
      Serial.printf("✓ %s\r\n", line.c_str());
    } else {
      errors++;
      Serial.printf("✗ Invalid: %s\r\n", line.c_str());
    }
  }

import_done:
  Serial.println();
  Serial.printf("Import complete: %d imported, %d errors\r\n", imported,
                errors);

  if (imported > 0) {
    cfg.save();
    Serial.println("Settings saved!");
  }
}

void printMenu() {
  Serial.println("\r\n\r\n========================================");
  if (g_menuState == MENU_MAIN) {
    Serial.println("           TEENSY VOTER MENU");
    Serial.println("========================================");
    Serial.println(" [1] Status Menu >>");
    Serial.println(" [2] Network Menu >>");
    Serial.println(" [3] Radio Config Menu >>");
    Serial.println(" [4] Debug Menu >>");
    Serial.println("----------------------------------------");
    Serial.println(" [S] Save & Reboot");
    Serial.println(" [M] Refresh Menu");
  } else if (g_menuState == MENU_STATUS) {
    IPAddress local = netMgr.getLocalIP();
    Serial.println("           STATUS MENU");
    Serial.println("========================================");
    Serial.printf(" Device IP    : %u.%u.%u.%u\r\n", local[0], local[1],
                  local[2], local[3]);
    Serial.printf(" VOTER Server : %s\r\n",
                  voterClient.isConnected() ? "CONNECTED" : "DISCONNECTED");
    Serial.println(" -- GPS Status --");
    Serial.printf(" Locked       : %s\r\n", gps.isLocked() ? "YES" : "NO");
    Serial.printf(" Satellites   : %u\r\n", gps.getSatellites());
    Serial.printf(" Time Set     : %s\r\n", gps.isTimeSet() ? "YES" : "NO");
    if (gps.isTimeSet()) {
      VTIME t;
      gps.getNetworkTime(&t);
      Serial.printf(" Voter Time   : %u.%09u\r\n", t.vtime_sec, t.vtime_nsec);
    }
    Serial.printf(" PPS Jitter   : %u us\r\n", gps.getPpsJitter());
    char lat[16], lon[16], elev[16];
    gps.getGPSStrings(lat, lon, elev);
    Serial.printf(" Location     : %s, %s (Elev: %s m)\r\n", lat, lon, elev);
    Serial.println("----------------------------------------");
    Serial.println(" [R] Refresh GPS Data");
    Serial.println(" [x] Back to Main Menu");
  } else if (g_menuState == MENU_NETWORK) {
    Serial.println("           NETWORK MENU");
    Serial.println("========================================");

    // Show active network type
    if (netMgr.getType() == DRIVER_ETHERNET) {
      Serial.println(" Network: Ethernet (NativeEthernet)");
      IPAddress ip = netMgr.getLocalIP();
      Serial.printf(" IP: %u.%u.%u.%u\r\n", ip[0], ip[1], ip[2], ip[3]);
    } else if (netMgr.getType() == DRIVER_WIFI_SPI) {
      Serial.println(" Network: WiFi (ESP32 SPI)");
      IPAddress ip = netMgr.getLocalIP();
      Serial.printf(" IP: %u.%u.%u.%u\r\n", ip[0], ip[1], ip[2], ip[3]);
    }
    Serial.println("----------------------------------------");

    // Consolidate Host Address Display
    IPAddress hIP(cfg.data.hostIP);
    if (cfg.data.hostname[0] != '\0') {
      Serial.printf(" [3] Host Addr   : %s (Hostname)\r\n", cfg.data.hostname);
    } else {
      Serial.printf(" [3] Host Addr   : %u.%u.%u.%u (IP)\r\n", hIP[0], hIP[1],
                    hIP[2], hIP[3]);
    }

    Serial.printf(" [4] Host Port   : %u\r\n", cfg.data.hostPort);
    Serial.printf(" [5] Client PWD  : %s\r\n", cfg.data.clientPwd);
    Serial.printf(" [6] Host PWD    : %s\r\n", cfg.data.hostPwd);

    // Ethernet static IP options (only show when using Ethernet)
    if (netMgr.getType() == DRIVER_ETHERNET) {
      Serial.println("----------------------------------------");
      Serial.printf(" [E] Use Static IP: %s\r\n",
                    cfg.data.useStaticIP ? "YES" : "NO (DHCP)");
      if (cfg.data.useStaticIP) {
        IPAddress sip(cfg.data.staticIP);
        IPAddress smask(cfg.data.subnetMask);
        IPAddress gw(cfg.data.gateway);
        IPAddress sdns(cfg.data.staticDNS);
        Serial.printf(" [I] Static IP   : %u.%u.%u.%u\r\n", sip[0], sip[1],
                      sip[2], sip[3]);
        Serial.printf(" [N] Subnet Mask : %u.%u.%u.%u\r\n", smask[0], smask[1],
                      smask[2], smask[3]);
        Serial.printf(" [G] Gateway     : %u.%u.%u.%u\r\n", gw[0], gw[1], gw[2],
                      gw[3]);
        if (sdns != IPAddress(0, 0, 0, 0)) {
          Serial.printf(" [D] DNS Server  : %u.%u.%u.%u\r\n", sdns[0], sdns[1],
                        sdns[2], sdns[3]);
        } else {
          Serial.println(" [D] DNS Server  : (Use Gateway)");
        }
      }
    }

    // WiFi-specific options (only show when using WiFi)
    if (netMgr.getType() == DRIVER_WIFI_SPI) {
      Serial.println("----------------------------------------");
      Serial.printf(" [W] WiFi SSID   : %s\r\n", cfg.data.wifiSSID);
      Serial.println(" [P] Set WiFi Password");

      IPAddress actualDNS = spiDriver.getDNSServer();
      IPAddress cfgDNS(cfg.data.dnsServerIP);
      if (cfg.data.dnsServerIP == 0) {
        Serial.printf(
            " [D] DNS Server  : (DHCP default - Retrieved: %u.%u.%u.%u)\r\n",
            actualDNS[0], actualDNS[1], actualDNS[2], actualDNS[3]);
      } else {
        Serial.printf(" [D] DNS Server  : %u.%u.%u.%u (Static)\r\n", cfgDNS[0],
                      cfgDNS[1], cfgDNS[2], cfgDNS[3]);
      }
      Serial.println(" [R] Resend WiFi Credentials to ESP32");
    }

    Serial.println("----------------------------------------");
    Serial.println(" [S] Save & Reboot");
    Serial.println(" [x] Back to Main Menu");
  } else if (g_menuState == MENU_RADIO_CONFIG) {
    Serial.println("        RADIO CONFIGURATION");
    Serial.println("========================================");

    Serial.println(" -- Squelch Logic --");
    Serial.printf(" [1] COS Mode    : %d (0=On,1=Hw,2=DSP)\r\n",
                  cfg.data.cosMode);
    Serial.printf(" [2] Invert COS  : %s\r\n",
                  cfg.data.cosInvert ? "YES" : "NO");
    if (cfg.data.cosMode == COS_MODE_DSP) {
      Serial.printf(" [3] DSP Squelch : %d\r\n", cfg.data.dspSquelchThresh);
    }

    Serial.println(" -- Audio --");
    Serial.printf(" [4] Hw Pre-Amp  : %d (0-15)\r\n", cfg.data.rxGain);
    Serial.printf(" [5] Digital Gain: %d%%\r\n", g_digitalGainPct);
    Serial.printf(" [6] Input Source: %s\r\n",
                  cfg.data.inputSource == AUDIO_INPUT_LINEIN ? "LINE IN"
                                                             : "MIC");
    Serial.printf(" [7] De-emphasis : %s\r\n",
                  cfg.data.enableDeemp ? "ON" : "OFF");
    Serial.printf(" [8] PL Filter   : %s\r\n",
                  cfg.data.enablePLFilter ? "ON" : "OFF");

    Serial.println(" -- RSSI --");
    Serial.printf(" [9] RSSI Mode   : %s\r\n",
                  cfg.data.useHwRSSI ? "HARDWARE (ADC)" : "SOFTWARE (DSP)");
    if (cfg.data.useHwRSSI) {
      Serial.printf(" [0] Calibration : Min=%d, Max=%d\r\n", cfg.data.rssiMin,
                    cfg.data.rssiMax);
    }
    Serial.println("----------------------------------------");
    Serial.println(" [S] Save & Reboot");
    Serial.println(" [x] Back to Main Menu");
  } else if (g_menuState == MENU_DEBUG) {
    Serial.println("           DEBUG MENU");
    Serial.println("========================================");
    Serial.printf(" [T] Test Tone    : %s\r\n", g_testToneMode ? "ON" : "OFF");
    Serial.printf(" [F] Force RSSI   : %d\r\n", g_forcedRSSI);
    Serial.println(" [D] Signal Monitor (Live Dashboard)");
    Serial.println("----------------------------------------");
    Serial.println(" [E] Export Settings");
    Serial.println(" [I] Import Settings");
    Serial.println("----------------------------------------");
    Serial.println(" [x] Back to Main Menu");
  }
  Serial.println("========================================");
  Serial.print("> ");
}

void handleSerialCLI() {
  if (Serial.available() > 0) {
    char c = Serial.read();

    // Universal Back Button
    if (c == 'x' || c == 'X' || c == 'b' || c == 'B') {
      if (g_menuState != MENU_MAIN) {
        g_menuState = MENU_MAIN;
        printMenu();
        return;
      }
    }

    if (g_menuState == MENU_MAIN) {
      switch (c) {
      case '1':
        g_menuState = MENU_STATUS;
        printMenu();
        break;
      case '2':
        g_menuState = MENU_NETWORK;
        printMenu();
        break;
      case '3':
        g_menuState = MENU_RADIO_CONFIG;
        printMenu();
        break;
      case '4':
        g_menuState = MENU_DEBUG;
        printMenu();
        break;

      // System
      case 's':
      case 'S':
        cfg.save();
        Serial.println("\r\nSaving & Rebooting...");
        delay(1000);
        SCB_AIRCR = 0x05FA0004;
        break;
      case 'm':
      case 'M':
        printMenu();
        break;
      }
    } else if (g_menuState == MENU_STATUS) {
      switch (c) {
      case 'r':
      case 'R':
        printMenu();
        break;
      case 's':
      case 'S':
        cfg.save();
        Serial.println("\r\nSaving & Rebooting...");
        delay(1000);
        SCB_AIRCR = 0x05FA0004;
        break;
      }
    } else if (g_menuState == MENU_NETWORK) {
      switch (c) {
      case '3': {
        Serial.print("\r\nEnter Host Address (IP or Hostname): ");
        String val = readStringEcho();
        val.trim();
        IPAddress ip;
        if (ip.fromString(val)) {
          // It's a valid IP
          // Use direct cast instead of manual bit shifting to preserve byte
          // order
          cfg.data.hostIP = (uint32_t)ip;
          // Clear hostname so IP takes precedence/shows in menu
          cfg.setHostname("");
          Serial.println("\r\n[Network] Set Host IP directly.");
        } else {
          // Treat as Hostname
          if (val.length() < 64) {
            cfg.setHostname(val.c_str());
            Serial.println(
                "\r\n[Network] Set Hostname. Will resolve on boot/save.");
          }
        }
        printMenu();
        break;
      }
      case '4': {
        Serial.print("\r\nEnter Host Port: ");
        cfg.data.hostPort = readStringEcho().toInt();
        printMenu();
        break;
      }
      case '5': {
        Serial.print("\r\nEnter Client PWD: ");
        String s = readStringEcho();
        s.trim();
        if (s.length() < 20)
          strcpy(cfg.data.clientPwd, s.c_str());
        printMenu();
        break;
      }
      case '6': {
        Serial.print("\r\nEnter Host PWD: ");
        String s = readStringEcho();
        s.trim();
        if (s.length() < 20)
          strcpy(cfg.data.hostPwd, s.c_str());
        printMenu();
        break;
      }
      case 'w':
      case 'W': {
        Serial.print("\r\nEnter WiFi SSID: ");
        String s = readStringEcho();
        s.trim();
        if (s.length() < 32)
          strcpy(cfg.data.wifiSSID, s.c_str());
        printMenu();
        break;
      }
      case 'p':
      case 'P': {
        Serial.print("\r\nEnter WiFi Pass: ");
        String s = readStringEcho();
        s.trim();
        if (s.length() < 64)
          strcpy(cfg.data.wifiPass, s.c_str());
        printMenu();
        break;
      }

      // Static IP configuration (Ethernet only)
      case 'e':
      case 'E': {
        cfg.data.useStaticIP = !cfg.data.useStaticIP;
        Serial.printf("\r\n[Network] Static IP %s\r\n",
                      cfg.data.useStaticIP ? "ENABLED" : "DISABLED");
        if (cfg.data.useStaticIP) {
          Serial.println(
              "[Network] NOTE: Reboot required for changes to take effect");
        }
        printMenu();
        break;
      }
      case 'i':
      case 'I': {
        Serial.print("\r\nEnter Static IP: ");
        String s = readStringEcho();
        s.trim();
        IPAddress ip;
        if (ip.fromString(s)) {
          cfg.data.staticIP = (uint32_t)ip;
          Serial.println("\r\n[Network] Static IP updated");
        } else {
          Serial.println("\r\n[Network] Invalid IP address");
        }
        printMenu();
        break;
      }
      case 'n':
      case 'N': {
        Serial.print("\r\nEnter Subnet Mask: ");
        String s = readStringEcho();
        s.trim();
        IPAddress mask;
        if (mask.fromString(s)) {
          cfg.data.subnetMask = (uint32_t)mask;
          Serial.println("\r\n[Network] Subnet mask updated");
        } else {
          Serial.println("\r\n[Network] Invalid subnet mask");
        }
        printMenu();
        break;
      }
      case 'g':
      case 'G': {
        Serial.print("\r\nEnter Gateway IP: ");
        String s = readStringEcho();
        s.trim();
        IPAddress gw;
        if (gw.fromString(s)) {
          cfg.data.gateway = (uint32_t)gw;
          Serial.println("\r\n[Network] Gateway updated");
        } else {
          Serial.println("\r\n[Network] Invalid gateway address");
        }
        printMenu();
        break;
      }

      case 'd':
      case 'D': {
        // DNS configuration - works for both WiFi and Ethernet static IP
        if (netMgr.getType() == DRIVER_ETHERNET && cfg.data.useStaticIP) {
          Serial.print("\r\nEnter DNS Server IP (or 0.0.0.0 to use gateway): ");
          String s = readStringEcho();
          s.trim();
          if (s.length() > 0) {
            IPAddress dns;
            if (dns.fromString(s)) {
              cfg.data.staticDNS = (uint32_t)dns;
              Serial.println("\r\n[Network] Static DNS updated");
            }
          }
        } else if (netMgr.getType() == DRIVER_WIFI_SPI) {
          Serial.print(
              "\r\nEnter DNS Server IP (or 0.0.0.0 for DHCP default): ");
          String s = readStringEcho();
          s.trim();
          if (s.length() > 0) {
            IPAddress dns;
            if (dns.fromString(s)) {
              cfg.data.dnsServerIP = (uint32_t)dns;
            }
          }
        }
        printMenu();
        break;
      }
      case 'r':
      case 'R':
        Serial.println("Resending Credentials...");
        spiDriver.setCredentials(cfg.data.wifiSSID, cfg.data.wifiPass);
        break;
      case 's':
      case 'S':
        cfg.save();
        Serial.println("\r\nSaving & Rebooting...");
        delay(1000);
        SCB_AIRCR = 0x05FA0004;
        break;
      }
    } else if (g_menuState == MENU_RADIO_CONFIG) {
      switch (c) {
      // Squelch Logic
      case '1': {
        Serial.println("\r\nSelect COS Mode: [0] On [1] Hw [2] DSP");
        Serial.print("Enter mode: ");
        int mode = readStringEcho().toInt();
        if (mode >= 0 && mode <= 2)
          cfg.data.cosMode = mode;
        printMenu();
        break;
      }
      case '2':
        cfg.data.cosInvert = !cfg.data.cosInvert;
        printMenu();
        break;
      case '3':
        if (cfg.data.cosMode == COS_MODE_DSP) {
          Serial.print("\r\nEnter DSP Squelch Thresh (0-255): ");
          cfg.data.dspSquelchThresh = readStringEcho().toInt();
        }
        printMenu();
        break;

      // Audio
      case '4': {
        Serial.print("\r\nEnter Pre-Amp (0-15): ");
        int g = readStringEcho().toInt();
        if (g >= 0 && g <= 15) {
          cfg.data.rxGain = g;
          sgtl5000_1.lineInLevel(g);
        }
        printMenu();
        break;
      }
      case '5': {
        Serial.print("\r\nEnter Digital Gain % (0-500): ");
        int g = readStringEcho().toInt();
        if (g >= 0 && g <= 500) {
          g_digitalGainPct = g;
          float gainFactor = (float)g / 100.0f;
          mixer1.gain(0, gainFactor);
          mixer1.gain(1, gainFactor);
        }
        printMenu();
        break;
      }
      case '6':
        if (cfg.data.inputSource == AUDIO_INPUT_LINEIN) {
          cfg.data.inputSource = AUDIO_INPUT_MIC;
          sgtl5000_1.inputSelect(AUDIO_INPUT_MIC);
          sgtl5000_1.micGain(40);
        } else {
          cfg.data.inputSource = AUDIO_INPUT_LINEIN;
          sgtl5000_1.inputSelect(AUDIO_INPUT_LINEIN);
          sgtl5000_1.lineInLevel(cfg.data.rxGain);
        }
        printMenu();
        break;
      case '7':
        cfg.data.enableDeemp = !cfg.data.enableDeemp;
        printMenu();
        break;
      case '8':
        cfg.data.enablePLFilter = !cfg.data.enablePLFilter;
        printMenu();
        break;

      // RSSI
      case '9':
        cfg.data.useHwRSSI = !cfg.data.useHwRSSI;
        printMenu();
        break;
      case '0':
        if (cfg.data.useHwRSSI) {
          Serial.println(
              "\r\nCalibrate RSSI: [1] Min (Open Squelch) [2] Max (Full Sig)");
          Serial.print("Select: ");
          int sel = readStringEcho().toInt();
          if (sel == 1) {
            cfg.data.rssiMin = analogRead(RSSI_PIN);
            Serial.printf("Min set to %d\r\n", cfg.data.rssiMin);
          } else if (sel == 2) {
            cfg.data.rssiMax = analogRead(RSSI_PIN);
            Serial.printf("Max set to %d\r\n", cfg.data.rssiMax);
          }
        }
        printMenu();
        break;
      case 's':
      case 'S':
        cfg.save();
        Serial.println("\r\nSaving & Rebooting...");
        delay(1000);
        SCB_AIRCR = 0x05FA0004;
        break;
      }
    } else if (g_menuState == MENU_DEBUG) {
      switch (c) {
      case 't':
      case 'T':
        g_testToneMode = !g_testToneMode;
        resetAudioState();
        printMenu();
        break;
      case 'e':
      case 'E':
        exportSettings();
        printMenu();
        break;
      case 'i':
      case 'I':
        importSettings();
        printMenu();
        break;
      case 'f':
      case 'F':
        // TOGGLE: Disabled (-1) -> Min (0) -> Max (255) -> Disabled
        if (g_forcedRSSI == -1)
          g_forcedRSSI = 0; // Min Signal (Squelch Open, Weak)
        else if (g_forcedRSSI == 0)
          g_forcedRSSI = 255; // Saturated Signal
        else
          g_forcedRSSI = -1; // Disabled

        Serial.printf("\r\n[DEBUG] Force RSSI set to: %d\r\n", g_forcedRSSI);
        printMenu();
        break;
      case 'd':
      case 'D': {
        Serial.println("\r\n--- Live Tuner (Press 'x' to Exit) ---");
        Serial.println("Controls: '['/']' = HW Gain | '-'/'=' = Digital Gain");
        float localMax = 0.0f;
        uint32_t peakUpdates = 0;
        while (true) {
          if (recordQueue.available() > 0) {
            recordQueue.readBuffer();
            recordQueue.freeBuffer();
          }
          gps.update();
          netMgr.update();
          voterClient.update();
          if (peak1.available()) {
            float v = peak1.read();
            bool isCosActive = false;
            if (cfg.data.cosMode == COS_MODE_HARDWARE) {
              isCosActive =
                  (digitalRead(COS_PIN) == (cfg.data.cosInvert ? HIGH : LOW));
            } else if (cfg.data.cosMode == COS_MODE_DSP) {
              isCosActive = (dsp.getNoiseLevel() < cfg.data.dspSquelchThresh);
            } else {
              isCosActive = true;
            }
            if (v > localMax)
              localMax = v;
            peakUpdates++;
            static uint32_t lastPrint = 0;
            if (millis() - lastPrint > 100) {
              lastPrint = millis();
              Serial.print("\rGain: ");
              Serial.print(cfg.data.rxGain);
              Serial.print(" | Dig: ");
              Serial.print(g_digitalGainPct);
              Serial.print("% | Peak: ");
              Serial.print(localMax, 3);
              if (localMax > 0.95f)
                Serial.print(" CLIP!");
              else
                Serial.print("      ");
              Serial.print(" | COS: ");
              Serial.print(isCosActive ? "ACT" : "___");
              Serial.print(" | RSSI: ");
              if (cfg.data.useHwRSSI) {
                // Show raw analog and mapped
                int raw = analogRead(RSSI_PIN);
                int mapped =
                    map(raw, cfg.data.rssiMin, cfg.data.rssiMax, 0, 255);
                if (mapped < 0)
                  mapped = 0;
                if (mapped > 255)
                  mapped = 255;
                Serial.printf("%d (%d)", mapped, raw);
              } else {
                // DSP RSSI
                Serial.printf("%d", dsp.getNoiseLevel());
              }

              Serial.print(" [");
              // Bar graph 0-50 chars
              // 1.0 (Peak) = 50 chars
              int bars = (int)(localMax * 50.0f);
              if (bars > 50)
                bars = 50;

              // Calculate Gate Pos (Threshold 0-255 -> 0-50)
              // Only relevant if using DSP Squelch
              int gatePos = -1;
              if (cfg.data.cosMode == COS_MODE_DSP) {
                gatePos =
                    (int)(((float)cfg.data.dspSquelchThresh / 255.0f) * 50.0f);
              }

              // Gate Logic for Meter
              bool isCosActive = false;
              if (cfg.data.cosMode == COS_MODE_DSP) {
                isCosActive = (localMax * 100.0f) > cfg.data.dspSquelchThresh;
              } else if (cfg.data.cosMode == COS_MODE_HARDWARE) {
                isCosActive =
                    (digitalRead(COS_PIN) == (cfg.data.cosInvert ? LOW : HIGH));
              } else {
                isCosActive = true; // COS_MODE_ON
              }

              if (isCosActive) {
                bars = 0; // Force empty meter if squelch closed
              }

              for (int i = 0; i < 50; i++) {
                char c = ' ';
                if (i < bars)
                  c = '#';

                // Overlay markers
                if (i == 30) { // 60%
                  if (c == ' ')
                    c = '|';
                }
                if (i == gatePos) {
                  c = '|'; // Gate Threshold Info
                }
                Serial.print(c);
              }
              Serial.print("]");
              if (gatePos != -1)
                Serial.printf(" Gate:%d", cfg.data.dspSquelchThresh);

              peakUpdates = 0;
              localMax = 0.0f;
            }
          }
          if (Serial.available()) {
            char c = Serial.read();
            if (c == 'x' || c == 'X')
              break;
            if (c == ']') {
              if (cfg.data.rxGain < 15) {
                cfg.data.rxGain++;
                sgtl5000_1.lineInLevel(cfg.data.rxGain);
              }
            }
            if (c == '[') {
              if (cfg.data.rxGain > 0) {
                cfg.data.rxGain--;
                sgtl5000_1.lineInLevel(cfg.data.rxGain);
              }
            }
            if (c == '=') {
              g_digitalGainPct += 10;
              if (g_digitalGainPct > 500)
                g_digitalGainPct = 500;
              float gainFactor = (float)g_digitalGainPct / 100.0f;
              mixer1.gain(0, gainFactor);
              mixer1.gain(1, gainFactor);
            }
            if (c == '-') {
              if (g_digitalGainPct >= 10)
                g_digitalGainPct -= 10;
              float gainFactor = (float)g_digitalGainPct / 100.0f;
              mixer1.gain(0, gainFactor);
              mixer1.gain(1, gainFactor);
            }
          }
        }
        printMenu();
        break;
      }
      }
    }
  }
}

void loop() {
  // Auto-Show Menu on Connect or Error (Final State)
  static VoterState lastReportedState = VOTER_DISCONNECTED;
  VoterState currentState = voterClient.getState();

  // Frame Counting State (Crucial for Audio Timestamps)
  static uint32_t lastEpoch = 0;
  static uint32_t baseNsec = 0;
  static uint32_t framesSent = 0;

  if (currentState != lastReportedState) {
    if (currentState == VOTER_CONNECTED || currentState == VOTER_AUTH_ERROR) {
      Serial.println();
      printMenu();
    }
    lastReportedState = currentState;
  }

  // 1. Core Updates
  handleSerialCLI();

  gps.update();
  netMgr.update();
  voterClient.update();
  webServer.handleClient(); // Handle web requests

  // 2. Audio Processing Loop
  // Changed to 'if' to prevent starvation of GPS/Network if DSP is slow
  // We process up to 8 blocks per loop to ensure we drain the queue
  int blocksProcessed = 0;
  while (recordQueue.available() >= 1 && blocksProcessed < 8) {
    blocksProcessed++;
    int16_t *buff = recordQueue.readBuffer();
    if (!buff)
      continue; // Should not happen given check above, but safer than return

    // 1. DSP Moved to Frame Processing Loop (Lines ~690)
    // We only process Decimated 8kHz Audio (160 Samples).
    // Processing here caused 44.1kHz aliasing issues.

    // 3. Check if we have enough for a Frame (160 samples)
    // 3. Inject Test Tone if enabled (Overwrites Mic Data in 'buff')

    if (cfg.data.useHwRSSI) {
      // Just mapping code, no side effects
      int analogVal = analogRead(RSSI_PIN);
      (void)analogVal;
    }

    // 2. Fractional Resampling (44.1kHz -> 8000Hz)
    // Fixes timing drift (Pulsing) caused by integer decimation
    const float RESAMPLE_RATIO = AUDIO_SAMPLE_RATE_EXACT / 8000.0f; // ~5.5147

    // State variables (static to persist across blocks)
    static float resamplePos = 0.0f;
    static float lastFilteredSample = 0.0f;

    // Simple Anti-Aliasing Filter State
    // State variables (static to persist across blocks)
    // static float resamplePos = 0.0f;       // Already defined above
    // static float lastFilteredSample = 0.0f; // Already defined above

    // Process all 128 input samples
    for (int i = 0; i < 128; i++) {
      // Input is ALREADY FILTERED by AudioFilterBiquad hardware object
      float currentSample = (float)buff[i];

      // B. Generate Output Samples via Linear Interpolation
      while (resamplePos < (float)i) {
        float frac = resamplePos - ((float)i - 1.0f);
        // Linear Interpolation
        float out =
            lastFilteredSample + frac * (currentSample - lastFilteredSample);

        // Apply Post-Filter Attenuation (Digital Scaling)
        // out = out * g_postGain; // Removed

        // Clip and Store to Accumulation Buffer
        if (out > 32760.0f)
          out = 32760.0f;
        if (out < -32760.0f)
          out = -32760.0f;

        if (accHead < 512) {
          accumulationBuf[accHead++] = (int16_t)out;
        }

        // Advance target time for next output
        resamplePos += RESAMPLE_RATIO;
      }

      lastFilteredSample = currentSample;
    }

    // Adjust resamplePos for next block (subtract 128 input samples)
    resamplePos -= 128.0f;

    // Free the Audio Library buffer
    recordQueue.freeBuffer();

    // 3. Check if we have enough for a Frame (160 samples)
    // 3. Check if we have enough for a Frame (160 samples)
    if (accHead >= 160) {

      // VOTER2 TIMING: Capture GPS timestamp NOW (at frame assembly)
      // This timestamp will be used for packet transmission
      VTIME frameTime;
      gps.getNetworkTime(&frameTime);

      // 3. Inject Test Tone if enabled (Overwrites Accumulation Buffer)
      if (g_testToneMode) {
        // Pure Digital Sine Wave (1kHz @ 8kHz sample rate = 1/8 cycle per
        // sample) Magnitude 16000 (~50% Full Scale)
        const float phaseInc = 0.125f * 2.0f * PI; // 2 * PI * (1000 / 8000)

        for (int i = 0; i < 160; i++) {
          accumulationBuf[i] =
              (int16_t)(arm_sin_f32(g_testTonePhase) * 16000.0f);
          g_testTonePhase += phaseInc;
          if (g_testTonePhase >= 2.0f * PI) {
            g_testTonePhase -= 2.0f * PI;
          }
        }
      }

      // CRITICAL: Process Audio (Filter, De-emphasis, RSSI)

      // Let's call process anyway for now to get SOME filtering.
      // But this confirms why "Mechanical" - Mismatched block sizes!

      uint8_t measuredNoise = dsp.process(
          accumulationBuf, cfg.data.enablePLFilter, cfg.data.enableDeemp);

      // FINAL MASTER VOLUME (Post-DSP Scaling) - REMOVED (Moved back to
      // Resampler)

      uint8_t baseRSSI;
      if (cfg.data.useHwRSSI) {
        // Hardware RSSI Mode: Read ADC and Map
        int rawRSSI = analogRead(RSSI_PIN);
        // Constrain to calibrated range
        if (rawRSSI < cfg.data.rssiMin)
          rawRSSI = cfg.data.rssiMin;
        if (rawRSSI > cfg.data.rssiMax)
          rawRSSI = cfg.data.rssiMax;

        // Map to 0-255 (Simple linear map)
        // Note: map() uses integer math.
        long mapped = map(rawRSSI, cfg.data.rssiMin, cfg.data.rssiMax, 0, 255);
        baseRSSI = (uint8_t)mapped;
      } else {
        // DSP RSSI Mode
        // dsp.process returns a "Signal Strength" (0=Noise, 255=Quiet)
        // User wants: 255=Lowest Noise (Quiet), 0=Max Noise
        // So we pass it through directly.
        baseRSSI = measuredNoise;
      }

      uint8_t ulawFrame[160];
      dsp.encodeULaw(accumulationBuf, ulawFrame, 160);

      // Calculate Final RSSI for protocol
      // (This logic was inside the loop in original, but we can compute it
      // once per frame or use latest) For simplicity, we use the baseRSSI
      // computed for the last block (approximate is fine for 20ms frame)

      uint8_t finalRSSI = baseRSSI;
      switch (cfg.data.cosMode) {
      case COS_MODE_HARDWARE:
        // Hardware COS
        // Default (Invert=False): Low=Active (Signal), High=Inactive
        // (Squelched) If Invert=True: High=Active, Low=Inactive
        bool isCosActive;
        if (cfg.data.cosInvert) {
          isCosActive = (digitalRead(COS_PIN) == HIGH);
        } else {
          isCosActive = (digitalRead(COS_PIN) == LOW);
        }

        if (!isCosActive) {
          finalRSSI = 0; // Squelch Closed (Inactive)
        } else {
          // Squelch Open (Active)
          // Pass the Software RSSI (baseRSSI) through.
          // User Request: "hardware cos and software rssi"
        }
        break;
      case COS_MODE_DSP:
        if (dsp.getNoiseLevel() >= cfg.data.dspSquelchThresh)
          finalRSSI = 0;
        break;
      }

      // FORCE Test Tone to send
      if (g_testToneMode) {
        finalRSSI = 255;
        // logic moved to before DSP
      }

      bool shouldSend = (finalRSSI > 0);
      if (shouldSend) {
        // Use the proper client method which handles sequence, timestamp, and
        // sending
        VTIME frameTime = {0, 0};
        if (gps.isTimeSet()) {
          // STRICT FRAME COUNTING STRATEGY
          // Decouples timestamp from micros() jitter/phase drift.

          // CONTINUOUS FRAME COUNTING STRATEGY (Approved Plan)
          // Fixes "Timestamp Jumps" caused by buffer lag resyncing to wall
          // clock.

          uint32_t currentEpoch = gps.getEpoch();

          // Reset Condition:
          // 1. First run (lastEpoch == 0)
          // 2. Large Gap (Transmission restarted after silence/squelch)
          //    We detect restart by checking if we "stopped" sending recently?
          //    Actually, we can just check if 'framesSent' is 0 (we must ensure
          //    it is reset when we STOP sending)

          if (framesSent == 0) {
            // Capture EXACT start time (Seconds + Nanoseconds)
            // This preserves the phase offset between GPS PPS and Audio Start
            VTIME t;
            gps.getNetworkTime(&t);
            lastEpoch = t.vtime_sec;
            baseNsec = t.vtime_nsec;
          }

          // Compute Time: Base + Offset
          // Use 64-bit math to prevent overflow of NSEC calculation
          // Add the initial random phase offset (baseNsec) to the continuous
          // frame count
          uint64_t totalNsec =
              (uint64_t)baseNsec +
              ((uint64_t)framesSent * 20000000ULL); // 20ms per frame

          uint32_t secOffset = (uint32_t)(totalNsec / 1000000000ULL);
          uint32_t nsecRemainder = (uint32_t)(totalNsec % 1000000000ULL);

          frameTime.vtime_sec = lastEpoch + secOffset;
          frameTime.vtime_nsec = nsecRemainder;

          // Deduct Fixed Delay (180ms) for Voter Receiver Buffer
          uint32_t delayNs = 180000000;

          if (frameTime.vtime_nsec >= delayNs) {
            frameTime.vtime_nsec -= delayNs;
          } else {
            // Borrow from seconds
            frameTime.vtime_sec--;
            frameTime.vtime_nsec =
                (1000000000 + frameTime.vtime_nsec) - delayNs;
          }

          // DRIFT GUARD: If we drift > 2 seconds from Real GPS Time, force a
          // resync. This handles startup lags or long periods of silence where
          // framesSent wasn't incrementing.
          int32_t driftSec = (int32_t)(currentEpoch - frameTime.vtime_sec);
          if (abs(driftSec) > 2) {
            // Force Resync
            lastEpoch = currentEpoch;
            framesSent = 0;
            // Re-calculate for this frame (recursive-ish, but simple linear
            // calc is faster)
            frameTime.vtime_sec = currentEpoch;
            frameTime.vtime_nsec = 0;
            // Apply delay
            if (frameTime.vtime_nsec >= delayNs) {
              frameTime.vtime_nsec -= delayNs;
            } // Should not happen for 0
            else {
              frameTime.vtime_sec--;
              frameTime.vtime_nsec = (1000000000 + 0) - delayNs;
            }

            // Debug print only if serial is open/fast enough?
            // Serial.printf("[Time] Resync! Drift: %d s\r\n", driftSec);
          }
        }

        // FORCE RSSI Debug Override
        if (g_forcedRSSI >= 0) {
          finalRSSI = (uint8_t)g_forcedRSSI;
        }

        voterClient.processAudioFrame(ulawFrame, finalRSSI, frameTime);
        if (gps.isTimeSet()) {
          framesSent++;
        }
        // Serial.println("[Test] Generated Audio Frame (Not Sent)");
      } else {
        // Not sending, reset continuous counter so next start uses fresh epoch
        framesSent = 0;
      }

      // Move remaining
      int remaining = accHead - 160;
      if (remaining > 0) {
        memmove(accumulationBuf, &accumulationBuf[160],
                remaining * sizeof(int16_t));
        accHead = remaining;
      } else {
        accHead = 0;
      }
    }
  }
}
