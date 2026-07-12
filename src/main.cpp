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
#include <TimeLib.h>
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
#include "TeensyWebServer.h"
#include "Version.h"
#include "Logger.h"

#define RSSI_PIN 38 // A14 - Connect to voltage divider output (0-3.3V)
#define COS_PIN 41  // Hardware COS input (active HIGH/LOW depending on radio)
#define PTT_PIN 40  // PTT Output (Active LOW/HIGH configurable)
#define PPS_PIN 2   // GPS PPS Input

#define GPS_SERIAL Serial1 // GPS Module RX/TX (Pins 0/1)

// --- Global State ---
float g_headphoneVol = 0.5f;
// cfg.data.radioRxDigitalGainPct removed - replaced by cfg.data.radioRxDigitalGainPct

bool g_testToneMode = false; // If true, send 1kHz test tone
int g_forcedRSSI = -1;       // -1 = Disabled, 0-255 = Force Value
float g_testTonePhase = 0.0f;

// --- Debug Flags ---
enum DebugFlags {
  DEBUG_NONE = 0,
  DEBUG_JITTER = 1 << 0,  // [J] Jitter Buffer & Audio Timing
  DEBUG_NETWORK = 1 << 1, // [N] Network Packets (Future)
  DEBUG_GPS = 1 << 2,     // [G] GPS Sync/PPS (Future)
  DEBUG_DSP = 1 << 3      // [D] DSP Processing (Future)
};
uint32_t g_debugMask = DEBUG_NONE;

// Status Globals (for Web Interface)
volatile uint8_t g_currentRSSI = 0;
volatile bool g_cosActive = false;
static bool g_testToneActive = false; // Forced 1kHz transmit for testing

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

// --- Audio System ---
AudioInputI2S i2s_in;
AudioMixer4 mixerTX; // Mixed audio going to the TRANSMITTER (i2s_out)
AudioMixer4 mixerRX; // Audio going to ASTERISK (recordQueue)
AudioRecordQueue recordQueue;
AudioPlayQueue playQueue; // Playback Queue (Jitter Buffer Output)
AudioOutputI2S i2s_out;

AudioFilterBiquad hpf_playback; // Disabled for pure testing
AudioFilterBiquad lpf_playback; // Disabled for pure testing
AudioSynthWaveform sine1;      // Test Tone Generator

// ==============================================
// 1. RADIO RECEIVER -> ASTERISK (recordQueue)
// ==============================================
AudioConnection patchCord1(i2s_in, 0, mixerRX, 0);     // Left  Incoming Radio
AudioConnection patchCord2(i2s_in, 1, mixerRX, 1);     // Right Incoming Radio

// Anti-Aliasing Filter (LPF) before Downsampling and sending to Asterisk
AudioFilterBiquad lpf1;
AudioConnection patchCord3(mixerRX, 0, lpf1, 0);       // RX Mixer -> LPF
AudioConnection patchCordLPF(lpf1, 0, recordQueue, 0); // LPF -> Network Queue

// ==============================================
// 2. ASTERISK (playQueue) -> RADIO TRANSMITTER (i2s_out)
// ==============================================
// Full Filtering: Strip hum and limit frequencies to standard 3.5kHz telecom quality
AudioConnection patchCordP1(playQueue, 0, hpf_playback, 0); 
AudioConnection patchCordP2(hpf_playback, 0, lpf_playback, 0);
AudioConnection patchCordP3(lpf_playback, 0, mixerTX, 0); // Network Audio -> TX Mixer Ch0
AudioConnection patchCordS1(sine1, 0, mixerTX, 1);        // Test Tone -> TX Mixer Ch1

// Drive both L and R outputs of the I2S DAC with the TX Mixer
AudioConnection patchCord4(mixerTX, 0, i2s_out, 0);
AudioConnection patchCord5(mixerTX, 0, i2s_out, 1);

// Connect the Peak Meter to the RX incoming audio so the UI shows actual radio squelch activity
AudioAnalyzePeak peak1;
AudioConnection patchCordMeter(mixerRX, 0, peak1, 0);

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
bool networkReady = false; // Global: set in setup(), used in loop()

byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};

// --- Forward Declarations ---
void handleSerialCLI();
void printMenu();

// --- Yield Callback for blocking network operations ---
void voterYield() {
  gps.update();
  handleSerialCLI();
}

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

  // Increase GPS Serial Buffer to handle stalls during DNS resolution
  static uint8_t gpsReadBuffer[2048];
  GPS_SERIAL.addMemoryForRead(gpsReadBuffer, sizeof(gpsReadBuffer));
  GPS_SERIAL.begin(9600);

  Serial.println("========================================");
  Serial.print("TeensyVoter Firmware v");
  Serial.println(FIRMWARE_VERSION);
  Serial.print("Build Date: ");
  Serial.print(BUILD_DATE);
  Serial.print(" ");
  Serial.println(BUILD_TIME);
  Serial.println("========================================");
  Serial.println();

  // Load Config
  cfg.begin();

  // 1.1 Hardware I/O Setup
  pinMode(PTT_PIN, OUTPUT_OPENDRAIN);
  bool pttIdleState = cfg.data.pttInvert ? HIGH : LOW;
  digitalWrite(PTT_PIN, pttIdleState); // Set to Idle (High-Impedance in OD mode)
  Serial.printf("[PTT] Initializing Pin %d, Polarity: %s, Default State: %s\r\n",
                PTT_PIN, cfg.data.pttInvert ? "Active Low" : "Active High",
                pttIdleState ? "HIGH" : "LOW");
  
  // Debug Print Passwords
  Serial.printf("[System] Voter Client Pwd: %s\r\n", cfg.data.clientPwd);
  Serial.printf("[System] Voter Host Pwd:   %s\r\n", cfg.data.hostPwd);

  pinMode(COS_PIN, (cfg.data.cosMode == COS_MODE_HARDWARE) ? INPUT_PULLUP : INPUT);
  pinMode(RSSI_PIN, INPUT);

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

  // 2. Audio Setup
  AudioMemory(60);

  if (sgtl5000_1.enable()) {
    Serial.println("[Audio] SGTL5000 Audio Shield FOUND & Enabled");
  } else {
    Serial.println("[Audio] ERROR: SGTL5000 Audio Shield NOT DETECTED!");
    Serial.println("[Audio] Check I2C pins and power.");
  }

  sgtl5000_1.volume(0.8f); // Fixed master hardware volume
  sgtl5000_1.lineOutLevel(13); // Fixed master output level
  
  // Configure Playback Filter Chain for Sub-Audible (CTCSS/PL) Support
  // We must pass frequencies down to ~60Hz for PL tones, so we lower the HPF to 20Hz.
  hpf_playback.setHighpass(0, 20.0f, 0.707f);
  
  // Set the Biquad to create a sharp 24dB/octave LPF
  // This drastically reduces Catmull-Rom upsampling artifacts but leaves voice crisp
  lpf_playback.setLowpass(0, 4000.0f, 0.707f);
  lpf_playback.setLowpass(1, 4000.0f, 0.707f);

  // Configure Test Tone Generator
  sine1.begin(0.0f, 1000.0f, WAVEFORM_SINE); // Amplitude 0 by default, controlled via CLI
  
  // Apply Directional Gain Settings
  // --- Radio TX (Teensy -> Radio) ---
  // Scaling: 100% slider = 0.4 Magnitude (Safe limit for Motorola)
  float masterTxGain = (float)cfg.data.radioTxMasterGainPct / 250.0f;
  mixerTX.gain(0, masterTxGain * 5.0f); // 5.0x boost for Network audio 
  mixerTX.gain(1, masterTxGain * 3.0f); // 3.0x for Test Tone so it plays loud

  // --- Radio RX (Radio -> Teensy) ---
  sgtl5000_1.lineInLevel(cfg.data.radioRxAnalogGain);
  dsp.setRxDigitalGainPct(cfg.data.radioRxDigitalGainPct);

  // Configure Hardware DSP (SGTL5000 hardware EQ disabled temporarily to debug tones)
  // sgtl5000_1.audioPostProcessorEnable();
  // sgtl5000_1.eqSelect(3); // 3 = GRAPHIC_EQUALIZER
  // eqBands: bass (115Hz), mid_low (330Hz), mid (990Hz), mid_high (3kHz), treble (9.9kHz)
  // sgtl5000_1.eqBands(0.0f, 3.0f, 5.0f, 2.0f, -8.0f);

  // Only drive the Left channel to the I2S output (User is using 'Left Pin')
  // To avoid ground loops or fighting drivers on mono-shorted cables.
  // (Connections moved to global scope)

  // Enforce Clean Audio Setup
  // Enforce Absolute Silence
  sgtl5000_1.adcHighPassFilterDisable();            // Try disabling HPF to stop "ringing"
  sgtl5000_1.micGain(0);                            
  sgtl5000_1.autoVolumeControl(0, 0, 0, -18, 0, 0); 
  sgtl5000_1.lineInLevel(cfg.data.radioRxAnalogGain);

  // Start Recording & Monitoring
  mixerRX.gain(0, 1.0f); // Radio Mic In
  mixerRX.gain(1, 0.0f); // Right unused
  // Channel 2 (Network Playback) already has masterTxGain applied!! Do not reset to 1.0f here!
  
  recordQueue.begin();

  Serial.println("[Audio] SGTL5000 & Queue Initialized");
  Serial.printf("[Audio] Applied RX Gain: %u\r\n", cfg.data.radioRxAnalogGain);

  sgtl5000_1.lineInLevel(cfg.data.radioRxAnalogGain);

  // Configure Anti-Aliasing Filter (3.6kHz Cutoff for 8kHz Sample Rate
  // compatibility)
  lpf1.setLowpass(0, 3600, 0.707);
  lpf1.setLowpass(1, 3600,
                  0.707); // Total 2 stages for steeper rolloff (24dB/oct)

  GPS_SERIAL.begin(9600);

  // 2.1 Start GPS Acquisition immediately (don't wait for Network)
  Serial.println("[GPS] Initializing GPS...");
  gps.begin(&GPS_SERIAL, PPS_PIN);

  Serial.println("[System] Boot Complete: Audio + Network + GPS");

  // 3. Network - Auto-select: Try Ethernet first, fallback to WiFi
  Serial.println("[System] Initializing Network Driver...");

  networkReady = false;

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
    netMgr.setYieldCallback(voterYield);
    networkReady = true;
  }
 else {
    Serial.println(
        "[Network] ✗ Ethernet unavailable (no cable or DHCP failure)");

    // Fallback to WiFi
    Serial.println("[Network] Falling back to WiFi (ESP32 SPI)...");

    // Initialize WiFi driver (This completely hard-resets the ESP32 and starts the SPI bus)
    if (spiDriver.begin(mac)) {
      // Give ESP32 time to boot AFTER the hardware reset
      Serial.println("[System] Waiting for ESP32 Boot (5s)...");
      uint32_t bootStart = millis();
      while (millis() - bootStart < 5000) {
        handleSerialCLI();
        delay(10);
      }

      // Send WiFi Credentials (retry 3x to guarantee delivery across SPI slave arming gaps)
      for (int attempt = 1; attempt <= 3; attempt++) {
        Serial.printf("[System] Sending WiFi Credentials (attempt %d/3)...\r\n", attempt);
        spiDriver.setCredentials(cfg.data.wifiSSID, cfg.data.wifiPass);
        delay(200); // Allow ESP32 to process before next attempt
      }

      spiDriver.setStaticDNS(IPAddress(cfg.data.dnsServerIP));
      netMgr.begin(&spiDriver, mac);
      netMgr.setYieldCallback(voterYield);

      // Wait for IP (up to 45s) with Audio Maintenance & CLI access
      Serial.print("[System] Waiting for WiFi Connection");
      bool ipFound = false;
      for (int i = 0; i < 90; i++) { // 90 * 500ms = 45 seconds
        // Maintain Audio Queue (Prevent Overflow/Crash)
        while (recordQueue.available() > 0) {
          recordQueue.readBuffer();
          recordQueue.freeBuffer();
        }

        // Keep Serial CLI responsive during connection
        handleSerialCLI();

        // Check IP
        IPAddress myIP = netMgr.getLocalIP();
        if (myIP != IPAddress(0, 0, 0, 0)) {
          Serial.println();
          Serial.print("[System] WiFi Connected! IP: ");
          Serial.println(myIP);
          ipFound = true;
          networkReady = true;

          // Push full config to ESP32 for its web server
          delay(200); // Let ESP32 settle
          spiDriver.pushConfig(&cfg.data, sizeof(cfg.data));

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
    Serial.println("[Network] ERROR: No network available! Will retry in background.");
  } else {
    Serial.println("[Network] Status: ONLINE");
  }

  delay(500); // Stabilization delay

  // --- Only proceed with network-dependent init if we have a connection ---
  if (networkReady) {
    // Resolve hostname if set
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

    // Resolve Syslog Hostname if set
    if (cfg.data.useSyslog && cfg.data.syslogHostname[0] != '\0') {
      Serial.printf("[DNS] Resolving Syslog Host: %s\r\n", cfg.data.syslogHostname);
      IPAddress resolved = netMgr.resolveHostname(cfg.data.syslogHostname);
      if (resolved != IPAddress(0, 0, 0, 0)) {
        cfg.data.syslogIP = (uint32_t)resolved;
        Serial.printf("[DNS] Syslog IP: %u.%u.%u.%u\r\n", resolved[0],
                      resolved[1], resolved[2], resolved[3]);
      }
    }

    netMgr.setTarget(cfg.getHostIP(), cfg.data.hostPort);

    Serial.print("[System] Voter Target: ");
    Serial.print(cfg.getHostIP());
    Serial.printf(":%u\r\n", cfg.data.hostPort);
  }

  // 5. Voter Client (only start if network available)
  if (networkReady) {
    voterClient.begin(&netMgr, &gps, cfg.getHostIP(), cfg.data.hostPort,
                      cfg.data.clientPwd, cfg.data.hostPwd);
  }

  // 6. DSP
  dsp.begin();

  // Initialize Decimator
  for (int i = 0; i < DECIMATOR_NUM_TAPS; i++) {
    decimatorCoeffs[i] = 1.0f / (float)DECIMATOR_NUM_TAPS;
  }

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

  // 8. Logger (only start if network available)
  if (networkReady) {
    logger.begin(&netMgr, &cfg);
    logger.info("SYS", "TeensyVoter v%s Boot Complete", FIRMWARE_VERSION);
  }
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
  MENU_VOTER_CONFIG,
  MENU_RADIO_CONFIG,
  MENU_DEBUG
};
MenuState g_menuState = MENU_MAIN;

// --- Settings Export/Import ---
void exportSettings() {
  Serial.println("\r\n# TeensyVoter Configuration Export");
  Serial.printf("# Version: %u\r\n", 17);
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
  Serial.printf("rxGain=%d\r\n", cfg.data.radioRxAnalogGain);
  Serial.printf("inputSource=%d\r\n", cfg.data.inputSource);
  Serial.printf("rssiMin=%d\r\n", cfg.data.rssiMin);
  Serial.printf("rssiMax=%d\r\n", cfg.data.rssiMax);
  Serial.printf("dspCalib=%.1f\r\n", cfg.data.dspCalib);
  Serial.printf("enablePLFilter=%d\r\n", cfg.data.enablePLFilter ? 1 : 0);
  Serial.printf("enableDeemp=%d\r\n", cfg.data.enableDeemp ? 1 : 0);
  
  // Syslog
  Serial.printf("useSyslog=%d\r\n", cfg.data.useSyslog ? 1 : 0);
  IPAddress sip(cfg.data.syslogIP);
  Serial.printf("syslogIP=%u.%u.%u.%u\r\n", sip[0], sip[1], sip[2], sip[3]);
  Serial.printf("syslogHostname=%s\r\n", cfg.data.syslogHostname);
  Serial.printf("syslogPort=%u\r\n", cfg.data.syslogPort);

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
    cfg.data.radioRxAnalogGain = value.toInt();
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
  } else if (key == "useSyslog") {
    cfg.data.useSyslog = (value.toInt() != 0);
  } else if (key == "syslogIP") {
    IPAddress ip;
    if (ip.fromString(value)) cfg.data.syslogIP = (uint32_t)ip;
  } else if (key == "syslogHostname") {
    strncpy(cfg.data.syslogHostname, value.c_str(), 63);
    cfg.data.syslogHostname[63] = '\0';
  } else if (key == "syslogPort") {
    cfg.data.syslogPort = value.toInt();
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
    Serial.println(" [5] Voter Config Menu >>");
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

    // Password Status
    VoterAuthError authErr = voterClient.getAuthError();
    const char *hostPwdStatus = "ACCEPTED";
    const char *clientPwdStatus = "ACCEPTED";

    if (!voterClient.isConnected()) {
      if (authErr == AUTH_ERR_NO_RESPONSE) {
        hostPwdStatus = "UNKNOWN";
        clientPwdStatus = "UNKNOWN";
      } else if (authErr == AUTH_ERR_HOST_MISMATCH) {
        hostPwdStatus = "MISMATCH";
        clientPwdStatus = "UNKNOWN";
      } else if (authErr == AUTH_ERR_CLIENT_REJECTED) {
        clientPwdStatus = "REJECTED";
      } else {
        hostPwdStatus = "PENDING";
        clientPwdStatus = "PENDING";
      }
    }

    Serial.printf(" Host PWD     : %s\r\n", hostPwdStatus);
    Serial.printf(" Voter PWD    : %s\r\n", clientPwdStatus);
    Serial.println(" -- GPS Status --");
    Serial.printf(" Locked       : %s\r\n", gps.isLocked() ? "YES" : "NO");
    Serial.printf(" Satellites   : %u\r\n", gps.getSatellites());
    Serial.printf(" Time Set     : %s\r\n", gps.isTimeSet() ? "YES" : "NO");
    if (gps.isTimeSet()) {
      VTIME t;
      gps.getNetworkTime(&t);
      // Format: YYYY-MM-DD HH:MM:SS
      time_t tt = (time_t)t.vtime_sec;
      Serial.printf(" Voter Time   : %04d-%02d-%02d %02d:%02d:%02d UTC\r\n",
                    year(tt), month(tt), day(tt), hour(tt), minute(tt),
                    second(tt));
    }
    Serial.printf(" PPS Jitter   : %u us\r\n", gps.getPpsJitter());
    char lat[16], lon[16], elev[32];
    gps.getGPSStrings(lat, lon, elev);
    Serial.printf(" Location     : %s, %s (Elev: %s)\r\n", lat, lon, elev);
    Serial.println("----------------------------------------");
    Serial.println(" [R] Refresh GPS Data");
    Serial.println(" [x] Back to Main Menu");
  } else if (g_menuState == MENU_NETWORK) {
    Serial.println("           NETWORK MENU");
    Serial.println("========================================");

    // Show active network type
    if (netMgr.getType() == DRIVER_ETHERNET) {
      Serial.println(" Network: Ethernet (NativeEthernet)");
    } else if (netMgr.getType() == DRIVER_WIFI_SPI) {
      Serial.println(" Network: WiFi (ESP32 SPI)");
    }

    IPAddress ip = netMgr.getLocalIP();
    Serial.printf(" IP Address  : %u.%u.%u.%u\r\n", ip[0], ip[1], ip[2], ip[3]);

    // Only show full IP config for Ethernet (WiFi SPI doesn't support subnet/gw queries)
    if (netMgr.getType() == DRIVER_ETHERNET) {
      IPAddress subnet = netMgr.getSubnetMask();
      IPAddress gw = netMgr.getGateway();
      IPAddress dns = netMgr.getDNS();
      Serial.printf(" Subnet Mask : %u.%u.%u.%u\r\n", subnet[0], subnet[1],
                    subnet[2], subnet[3]);
      Serial.printf(" Gateway     : %u.%u.%u.%u\r\n", gw[0], gw[1], gw[2], gw[3]);
      Serial.printf(" DNS Server  : %u.%u.%u.%u\r\n", dns[0], dns[1], dns[2],
                    dns[3]);
    }

    Serial.println("----------------------------------------");

    // Ethernet static IP options (only show when using Ethernet)
    if (netMgr.getType() == DRIVER_ETHERNET) {
      Serial.printf(" [1] Use Static IP: %s\r\n",
                    cfg.data.useStaticIP ? "YES" : "NO (DHCP)");
      if (cfg.data.useStaticIP) {
        IPAddress sip(cfg.data.staticIP);
        IPAddress smask(cfg.data.subnetMask);
        IPAddress gw(cfg.data.gateway);
        IPAddress sdns(cfg.data.staticDNS);
        Serial.printf(" [2] Static IP   : %u.%u.%u.%u\r\n", sip[0], sip[1],
                      sip[2], sip[3]);
        Serial.printf(" [3] Subnet Mask : %u.%u.%u.%u\r\n", smask[0], smask[1],
                      smask[2], smask[3]);
        Serial.printf(" [4] Gateway     : %u.%u.%u.%u\r\n", gw[0], gw[1], gw[2],
                      gw[3]);
        if (sdns != IPAddress(0, 0, 0, 0)) {
          Serial.printf(" [5] DNS Server  : %u.%u.%u.%u\r\n", sdns[0], sdns[1],
                        sdns[2], sdns[3]);
        } else {
          Serial.println(" [5] DNS Server  : (Use Gateway)");
        }
      }
    }

    // WiFi-specific options (only show when using WiFi)
    if (netMgr.getType() == DRIVER_WIFI_SPI) {
      Serial.printf(" [1] WiFi SSID   : %s\r\n", cfg.data.wifiSSID);
      Serial.println(" [2] Set WiFi Password");
      Serial.println(" [R] Resend WiFi Credentials to ESP32");
      
      IPAddress wdns(cfg.data.dnsServerIP);
      if (wdns != IPAddress(0, 0, 0, 0)) {
        Serial.printf(" [3] WiFi DNS    : %u.%u.%u.%u\r\n", wdns[0], wdns[1], wdns[2], wdns[3]);
      } else {
        Serial.println(" [3] WiFi DNS    : (DHCP Default)");
      }
    }

    Serial.println(" -- Syslog Settings --");
    Serial.printf(" [6] Use Syslog   : %s\r\n", cfg.data.useSyslog ? "YES" : "NO");
    if (cfg.data.syslogHostname[0] != '\0') {
        Serial.printf(" [7] Syslog Host  : %s\r\n", cfg.data.syslogHostname);
    } else {
        IPAddress sip(cfg.data.syslogIP);
        Serial.printf(" [7] Syslog IP    : %u.%u.%u.%u\r\n", sip[0], sip[1], sip[2], sip[3]);
    }
    Serial.printf(" [8] Syslog Port  : %u\r\n", cfg.data.syslogPort);

    Serial.println("----------------------------------------");
    Serial.println(" [S] Save & Reboot");
    Serial.println(" [x] Back to Main Menu");

  } else if (g_menuState == MENU_VOTER_CONFIG) {
    Serial.println("        VOTER CONFIGURATION");
    Serial.println("========================================");

    // Consolidate Host Address Display
    IPAddress hIP(cfg.data.hostIP);
    if (cfg.data.hostname[0] != '\0') {
      Serial.printf(" [1] Host Addr   : %s (Hostname)\r\n", cfg.data.hostname);
    } else {
      Serial.printf(" [1] Host Addr   : %u.%u.%u.%u (IP)\r\n", hIP[0], hIP[1],
                    hIP[2], hIP[3]);
    }

    Serial.printf(" [2] Host Port   : %u\r\n", cfg.data.hostPort);
    Serial.printf(" [3] Client PWD  : %s\r\n", cfg.data.clientPwd);
    Serial.printf(" [4] Host PWD    : %s\r\n", cfg.data.hostPwd);

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

    Serial.println(" -- Receive (Radio RX) --");
    Serial.printf(" [4] Analog Gain : %d (0-15)\r\n", cfg.data.radioRxAnalogGain);
    Serial.printf(" [5] Digital Gain: %d%%\r\n", cfg.data.radioRxDigitalGainPct);
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

    Serial.println(" -- Transmit (Radio TX) --");
    Serial.printf(" [V] Master Gain : %d%%\r\n", cfg.data.radioTxMasterGainPct);
    Serial.printf(" [P] Polarity    : %s\r\n", cfg.data.pttInvert ? "Active LOW" : "Active HIGH");
    Serial.printf(" [L] Tail Timing : %u ms\r\n", cfg.data.pttTailMs);
    Serial.printf(" [T] Test Tone   : %s\r\n", g_testToneActive ? "ON (PTT Keyed)" : "OFF");

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
    Serial.printf(" [J] Jitter Log   : %s\r\n",
                  (g_debugMask & DEBUG_JITTER) ? "ON" : "OFF");
    Serial.printf(" [N] Network Log  : %s\r\n",
                  (g_debugMask & DEBUG_NETWORK) ? "ON" : "OFF");
    Serial.printf(" [G] GPS Log      : %s\r\n",
                  (g_debugMask & DEBUG_GPS) ? "ON" : "OFF");
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
      case '5':
        g_menuState = MENU_VOTER_CONFIG;
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

      // Ethernet Configuration
      case '1':
        if (netMgr.getType() == DRIVER_ETHERNET) {
          cfg.data.useStaticIP = !cfg.data.useStaticIP;
          Serial.printf("\r\n[Network] Static IP %s\r\n",
                        cfg.data.useStaticIP ? "ENABLED" : "DISABLED");
          if (cfg.data.useStaticIP) {
            Serial.println(
                "[Network] NOTE: Reboot required for changes to take effect");
          }
          printMenu();
        } else if (netMgr.getType() == DRIVER_WIFI_SPI) {
          Serial.print("\r\nEnter WiFi SSID: ");
          String s = readStringEcho();
          s.trim();
          if (s.length() < 32)
            strcpy(cfg.data.wifiSSID, s.c_str());
          printMenu();
        }
        break;

      case '2':
        if (netMgr.getType() == DRIVER_ETHERNET && cfg.data.useStaticIP) {
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
        } else if (netMgr.getType() == DRIVER_WIFI_SPI) {
          Serial.print("\r\nEnter WiFi Pass: ");
          String s = readStringEcho();
          s.trim();
          if (s.length() < 64)
            strcpy(cfg.data.wifiPass, s.c_str());
          printMenu();
        }
        break;

      case '3':
        if (netMgr.getType() == DRIVER_ETHERNET && cfg.data.useStaticIP) {
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
        } else if (netMgr.getType() == DRIVER_WIFI_SPI) {
          // WiFi DNS
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
          printMenu();
        }
        break;

      case '4':
        if (netMgr.getType() == DRIVER_ETHERNET && cfg.data.useStaticIP) {
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
        }
        break;

      case '5':
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
          printMenu();
        }
        break;

      case 'r':
      case 'R':
        Serial.println("Resending Credentials...");
        spiDriver.setCredentials(cfg.data.wifiSSID, cfg.data.wifiPass);
        break;
      case '6':
        cfg.data.useSyslog = !cfg.data.useSyslog;
        printMenu();
        break;
      case '7': {
        Serial.print("\r\nEnter Syslog Server IP or Hostname: ");
        String val = readStringEcho();
        val.trim();
        IPAddress ip;
        if (ip.fromString(val)) {
          cfg.data.syslogIP = (uint32_t)ip;
          memset(cfg.data.syslogHostname, 0, sizeof(cfg.data.syslogHostname));
        } else if (val.length() > 0 && val.length() < 64) {
          strncpy(cfg.data.syslogHostname, val.c_str(), 63);
          cfg.data.syslogHostname[63] = '\0';
          cfg.data.syslogIP = 0;
          
          // Resolve immediately
          Serial.println("\r\n[DNS] Resolving...");
          IPAddress resolved = netMgr.resolveHostname(cfg.data.syslogHostname);
          if (resolved != IPAddress(0,0,0,0)) {
              cfg.data.syslogIP = (uint32_t)resolved;
              Serial.printf("[DNS] Resolved to: %u.%u.%u.%u\r\n", resolved[0], resolved[1], resolved[2], resolved[3]);
          }
        }
        printMenu();
        break;
      }
      case '8': {
        Serial.print("\r\nEnter Syslog Port: ");
        cfg.data.syslogPort = readStringEcho().toInt();
        printMenu();
        break;
      }
      case 's':
      case 'S':
        cfg.save();
        Serial.println("\r\nSaving & Rebooting...");
        delay(1000);
        SCB_AIRCR = 0x05FA0004;
        break;
      }
    } else if (g_menuState == MENU_VOTER_CONFIG) {
      switch (c) {
      case '1': {
        Serial.print("\r\nEnter Host Address (IP or Hostname): ");
        String val = readStringEcho();
        val.trim();
        IPAddress ip;
        if (ip.fromString(val)) {
          cfg.data.hostIP = (uint32_t)ip;
          cfg.setHostname("");
          Serial.println("\r\n[Voter] Set Host IP.");
        } else {
          if (val.length() < 64) {
            cfg.setHostname(val.c_str());
            Serial.println("\r\n[Voter] Set Hostname.");
          }
        }
        printMenu();
        break;
      }
      case '2': {
        Serial.print("\r\nEnter Host Port: ");
        cfg.data.hostPort = readStringEcho().toInt();
        printMenu();
        break;
      }
      case '3': {
        Serial.print("\r\nEnter Client PWD: ");
        String s = readStringEcho();
        s.trim();
        if (s.length() < 20)
          strcpy(cfg.data.clientPwd, s.c_str());
        printMenu();
        break;
      }
      case '4': {
        Serial.print("\r\nEnter Host PWD: ");
        String s = readStringEcho();
        s.trim();
        if (s.length() < 20)
          strcpy(cfg.data.hostPwd, s.c_str());
        printMenu();
        break;
      }
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
        Serial.print("\r\nEnter Analog RX Gain (0-15): ");
        int g = readStringEcho().toInt();
        if (g >= 0 && g <= 15) {
          cfg.data.radioRxAnalogGain = (uint8_t)g;
          sgtl5000_1.lineInLevel(g);
        }
        printMenu();
        break;
      }
      case '5': {
        Serial.print("\r\nEnter Digital RX Gain (0-200%): ");
        int g = readStringEcho().toInt();
        if (g >= 0 && g <= 200) {
          cfg.data.radioRxDigitalGainPct = (uint8_t)g;
          dsp.setRxDigitalGainPct(g);
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
          sgtl5000_1.lineInLevel(cfg.data.radioRxAnalogGain);
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
      case 'v':
      case 'V': {
        Serial.print("\r\nEnter Master TX Gain (0-100%): ");
        int g = readStringEcho().toInt();
        if (g >= 0 && g <= 100) {
          cfg.data.radioTxMasterGainPct = (uint8_t)g;
          float masterTxGain = (float)g / 250.0f; // 100% -> 0.4
          mixerTX.gain(0, masterTxGain * 5.0f); // Boosted Network Audio
          mixerTX.gain(1, masterTxGain);        // Reference Tone
        }
        printMenu();
        break;
      }
      case 't':
      case 'T': {
        g_testToneActive = !g_testToneActive;
        if (g_testToneActive) {
          sine1.amplitude(1.0f);
          Serial.println("\r\n[TEST] 1kHz Sine Tone ON (PTT Active)");
        } else {
          sine1.amplitude(0.0f);
          Serial.println("\r\n[TEST] 1kHz Sine Tone OFF");
        }
        printMenu();
        break;
      }
      case 'p':
      case 'P':
        cfg.data.pttInvert = !cfg.data.pttInvert;
        printMenu();
        break;
      case 'l':
      case 'L': {
        Serial.print("\r\nEnter PTT Tail (ms): ");
        cfg.data.pttTailMs = (uint16_t)readStringEcho().toInt();
        printMenu();
        break;
      }
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

      // Debug Flags
      case 'j':
      case 'J':
        g_debugMask ^= DEBUG_JITTER;
        printMenu();
        break;
      case 'n':
      case 'N':
        g_debugMask ^= DEBUG_NETWORK;
        printMenu();
        break;
      case 'g':
      case 'G':
        g_debugMask ^= DEBUG_GPS;
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
              Serial.print(cfg.data.radioRxAnalogGain);
              Serial.print(" | Dig: ");
              Serial.print(cfg.data.radioRxDigitalGainPct);
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
              if (cfg.data.radioRxAnalogGain < 15) {
                cfg.data.radioRxAnalogGain++;
                sgtl5000_1.lineInLevel(cfg.data.radioRxAnalogGain);
              }
            }
            if (c == '[') {
              if (cfg.data.radioRxAnalogGain > 0) {
                cfg.data.radioRxAnalogGain--;
                sgtl5000_1.lineInLevel(cfg.data.radioRxAnalogGain);
              }
            }
            if (c == '=') {
              cfg.data.radioRxDigitalGainPct += 10;
              if (cfg.data.radioRxDigitalGainPct > 500)
                cfg.data.radioRxDigitalGainPct = 255;
              float gainFactor = (float)cfg.data.radioRxDigitalGainPct / 100.0f;
              mixerRX.gain(0, gainFactor);
              mixerRX.gain(1, gainFactor);
            }
            if (c == '-') {
              if (cfg.data.radioRxDigitalGainPct >= 10)
                cfg.data.radioRxDigitalGainPct -= 10;
              float gainFactor = (float)cfg.data.radioRxDigitalGainPct / 100.0f;
              mixerRX.gain(0, gainFactor);
              mixerRX.gain(1, gainFactor);
            }
            if (c == ')') {
              if (cfg.data.radioTxMasterGainPct < 100) {
                cfg.data.radioTxMasterGainPct += 1;
                float masterTxGain = (float)cfg.data.radioTxMasterGainPct / 250.0f;
                mixerTX.gain(0, masterTxGain * 5.0f);
                mixerTX.gain(1, masterTxGain);
                Serial.printf("\r\n[Audio] Radio TX Master: %u%%\r\n", cfg.data.radioTxMasterGainPct);
              }
            }
            if (c == '(') {
              if (cfg.data.radioTxMasterGainPct > 0) {
                cfg.data.radioTxMasterGainPct -= 1;
                float masterTxGain = (float)cfg.data.radioTxMasterGainPct / 250.0f;
                mixerTX.gain(0, masterTxGain * 5.0f);
                mixerTX.gain(1, masterTxGain);
                Serial.printf("\r\n[Audio] Radio TX Master: %u%%\r\n", cfg.data.radioTxMasterGainPct);
              }
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

  // Frame Counting

  if (currentState != lastReportedState) {
    if (currentState == VOTER_CONNECTED || currentState == VOTER_AUTH_ERROR) {
      Serial.println();
      printMenu();
    }
    lastReportedState = currentState;
  }

  // 1. Core Updates
  handleSerialCLI();
  logger.update();

  gps.update();
  if (gps.checkPPS()) {
    voterClient.alignTimestampPhase();
  }

  netMgr.update();

  // Handle ESP32 web server config commands (WiFi mode only)
  if (spiDriver.hasConfigCmd()) {
    uint8_t cmdBuf[256];
    int cmdLen = spiDriver.readConfigCmd(cmdBuf, sizeof(cmdBuf));
    if (cmdLen > 0) {
      uint8_t subCmd = cmdBuf[0];
      if (subCmd == CFG_CMD_SET_PARAM && cmdLen >= 4) {
        uint8_t paramId = cmdBuf[1];
        uint8_t valLen = cmdBuf[2];
        uint8_t *val = &cmdBuf[3];
        // Apply parameter
        switch (paramId) {
          case PARAM_HOST_IP:
            if (valLen == 4) memcpy(&cfg.data.hostIP, val, 4);
            break;
          case PARAM_HOST_PORT:
            if (valLen == 2) cfg.data.hostPort = (val[0] << 8) | val[1];
            break;
          case PARAM_HOSTNAME:
            if (valLen < sizeof(cfg.data.hostname)) {
              memset(cfg.data.hostname, 0, sizeof(cfg.data.hostname));
              memcpy(cfg.data.hostname, val, valLen);
            }
            break;
          case PARAM_CLIENT_PWD:
            if (valLen < sizeof(cfg.data.clientPwd)) {
              memset(cfg.data.clientPwd, 0, sizeof(cfg.data.clientPwd));
              memcpy(cfg.data.clientPwd, val, valLen);
            }
            break;
          case PARAM_HOST_PWD:
            if (valLen < sizeof(cfg.data.hostPwd)) {
              memset(cfg.data.hostPwd, 0, sizeof(cfg.data.hostPwd));
              memcpy(cfg.data.hostPwd, val, valLen);
            }
            break;
          case PARAM_WIFI_SSID:
            if (valLen < sizeof(cfg.data.wifiSSID)) {
              memset(cfg.data.wifiSSID, 0, sizeof(cfg.data.wifiSSID));
              memcpy(cfg.data.wifiSSID, val, valLen);
            }
            break;
          case PARAM_WIFI_PASS:
            if (valLen < sizeof(cfg.data.wifiPass)) {
              memset(cfg.data.wifiPass, 0, sizeof(cfg.data.wifiPass));
              memcpy(cfg.data.wifiPass, val, valLen);
            }
            break;
          case PARAM_RX_GAIN:
            if (valLen == 1) cfg.data.radioRxAnalogGain = val[0];
            break;
          case PARAM_TX_GAIN_PCT:
            if (valLen == 1) cfg.data.radioTxMasterGainPct = val[0];
            break;
          case PARAM_COS_MODE:
            if (valLen == 1) cfg.data.cosMode = val[0];
            break;
          case PARAM_COS_INVERT:
            if (valLen == 1) cfg.data.cosInvert = val[0];
            break;
          case PARAM_DSP_SQUELCH:
            if (valLen == 1) cfg.data.dspSquelchThresh = val[0];
            break;
          case PARAM_USE_HW_RSSI:
            if (valLen == 1) cfg.data.useHwRSSI = val[0];
            break;
          case PARAM_RSSI_MIN:
            if (valLen == 2) cfg.data.rssiMin = (val[0] << 8) | val[1];
            break;
          case PARAM_RSSI_MAX:
            if (valLen == 2) cfg.data.rssiMax = (val[0] << 8) | val[1];
            break;
          case PARAM_PL_FILTER:
            if (valLen == 1) cfg.data.enablePLFilter = val[0];
            break;
          case PARAM_DEEMP:
            if (valLen == 1) cfg.data.enableDeemp = val[0];
            break;
          case PARAM_PTT_INVERT:
            if (valLen == 1) cfg.data.pttInvert = val[0];
            break;
          case PARAM_PTT_TAIL_MS:
            if (valLen == 2) cfg.data.pttTailMs = (val[0] << 8) | val[1];
            break;
          case PARAM_DSP_CALIB:
            if (valLen == 4) memcpy(&cfg.data.dspCalib, val, 4);
            break;
          case PARAM_INPUT_SOURCE:
            if (valLen == 1) cfg.data.inputSource = val[0];
            break;
          case PARAM_USE_SYSLOG:
            if (valLen == 1) cfg.data.useSyslog = val[0];
            break;
          case PARAM_SYSLOG_IP:
            if (valLen == 4) memcpy(&cfg.data.syslogIP, val, 4);
            break;
          case PARAM_SYSLOG_HOSTNAME:
            if (valLen < sizeof(cfg.data.syslogHostname)) {
                memset(cfg.data.syslogHostname, 0, sizeof(cfg.data.syslogHostname));
                memcpy(cfg.data.syslogHostname, val, valLen);
            }
            break;
          case PARAM_SYSLOG_PORT:
            if (valLen == 2) cfg.data.syslogPort = (val[0] << 8) | val[1];
            break;
        }
        Serial.printf("[WebCfg] SET_PARAM id=0x%02X len=%d\r\n", paramId, valLen);
      } else if (subCmd == CFG_CMD_SAVE_REBOOT) {
        Serial.println("[WebCfg] SAVE_REBOOT received! Saving...");
        cfg.save();
        delay(100);
        SCB_AIRCR = 0x05FA0004; // Reboot Teensy
      } else if (subCmd == CFG_CMD_REQUEST_CONFIG) {
        // Rate limit config pushes to once every 10 seconds to avoid UDP collisions
        static uint32_t lastPush = 0;
        if (millis() - lastPush > 10000) {
            lastPush = millis();
            Serial.println("[WebCfg] Config request from ESP32 - Pushing...");
            spiDriver.pushConfig(&cfg.data, sizeof(cfg.data));
        }
      }
    }
  }

  // Periodic Network Recovery if offline
  static uint32_t lastNetCheck = 0;
  if (!networkReady && (millis() - lastNetCheck > 5000)) {
    lastNetCheck = millis();
    IPAddress myIP = netMgr.getLocalIP();
    if (myIP != IPAddress(0, 0, 0, 0)) {
        Serial.printf("\r\n[Network] Late Connection Detected! IP: %u.%u.%u.%u\r\n", 
                      myIP[0], myIP[1], myIP[2], myIP[3]);
        networkReady = true;
        
        // Late Init Components
        voterClient.begin(&netMgr, &gps, cfg.getHostIP(), cfg.data.hostPort,
                          cfg.data.clientPwd, cfg.data.hostPwd);
        logger.begin(&netMgr, &cfg);
        logger.info("SYS", "TeensyVoter v%s Late Boot Complete", FIRMWARE_VERSION);
    }
  }

  if (networkReady) {
    voterClient.update();
  }
  webServer.handleClient(); // Handle web requests

  // Debug Jitter Buffer
  static uint32_t lastJitterDebug = 0;
  if ((g_debugMask & DEBUG_JITTER) && (millis() - lastJitterDebug > 1000)) {
    lastJitterDebug = millis();
    if (voterClient.jitterBuffer.available() > 0 ||
        voterClient.jitterBuffer.isBuffering()) {
      Serial.printf("[Jitter] Avail: %u, Space: %u, Buffering: %d\r\n",
                    voterClient.jitterBuffer.available(),
                    voterClient.jitterBuffer.space(),
                    voterClient.jitterBuffer.isBuffering());
      
      // Dump 16 bytes of the payload
      Serial.print("[Jitter Payload Head] Hex: ");
      uint8_t peekBuf[16];
      int toPeek = voterClient.jitterBuffer.available();
      if (toPeek > 16) toPeek = 16;
      for(int i=0; i<toPeek; i++) {
          uint8_t b = voterClient.jitterBuffer._buffer[(voterClient.jitterBuffer._tail + i) % JITTER_BUF_SIZE];
          Serial.printf("%02X ", b);
      }
      Serial.println();
    }
  }

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

        // Map to 0-127 (7-bit RSSI per protocol spec)
        // Note: map() uses integer math.
        long mapped = map(rawRSSI, cfg.data.rssiMin, cfg.data.rssiMax, 0, 127);
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
          // Set bit 7 (COR) and use baseRSSI (0-127) for bits 0-6
          finalRSSI = (baseRSSI & 0x7F) | 0x80;
        }
        break;
      case COS_MODE_DSP:
        if (dsp.getNoiseLevel() >= cfg.data.dspSquelchThresh) {
          finalRSSI = 0;
        } else {
          finalRSSI = (baseRSSI & 0x7F) | 0x80;
        }
        break;
      }

      // Update Global Status for Web Interface
      g_currentRSSI = finalRSSI;
      bool isCosActiveForStatus = false;
      if (cfg.data.cosMode == COS_MODE_HARDWARE) {
        isCosActiveForStatus =
            (digitalRead(COS_PIN) == (cfg.data.cosInvert ? HIGH : LOW));
      } else if (cfg.data.cosMode == COS_MODE_DSP) {
        isCosActiveForStatus =
            (dsp.getNoiseLevel() < cfg.data.dspSquelchThresh);
      } else {
        isCosActiveForStatus = true;
      }
      g_cosActive = isCosActiveForStatus;

      // FORCE Test Tone to send
      if (g_testToneMode) {
        finalRSSI = 255;
        // logic moved to before DSP
      }

      static int tailCount = 0;
      bool isTransmitting = (finalRSSI > 0);
      if (isTransmitting) {
        tailCount = 10; // Send 200ms of tail packets on unkey
      }

      if (isTransmitting || tailCount > 0) {
        if (!isTransmitting) {
          // Send "Tail Packet" with RSSI 0 to clear server decoder
          finalRSSI = 0;
          tailCount--;
        }

        // Standard -120ms delay offset for modern Ethernet (was -180ms)
        voterClient.setTimingOffset(-120);
        
        // Periodic Status Heartbeat (every 5 seconds)
        static uint32_t lastHb = 0;
        if (millis() - lastHb > 5000) {
            lastHb = millis();
            Serial.printf("[Heartbeat] GPS: %s, Sats: %u, Epoch: %lu\r\n", 
                          gps.isLocked() ? "LOCKED" : "WAITING", 
                          gps.getSatellites(), gps.getEpoch());
        }
        
        VTIME frameTime = {0, 0};
        if (gps.isTimeSet()) {
          gps.getNetworkTime(&frameTime, true);
        }

        // FORCE RSSI Debug Override (only if we were actively transmitting)
        if (g_forcedRSSI >= 0 && isTransmitting) {
          finalRSSI = (uint8_t)g_forcedRSSI;
        }

        static bool lastAudioState = false;
        if (!lastAudioState) {
            Serial.println("[Voter] First audio frame dispatched to driver.");
            lastAudioState = true;
        }
        voterClient.processAudioFrame(ulawFrame, finalRSSI, frameTime);
      } else {
        static bool lastAudioState = true;
        if (lastAudioState) {
            // Only clear when we stop sending
            lastAudioState = false;
        }
        // Completely Idle - Do nothing.
      }

      // Drive Free-Running Packet Counter per Authority / Voter2 Reference
      voterClient.incrementPacketCounter();

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

  // Send any audio frame staged by processAudioFrame() above, now that
  // this pass's record-queue draining is done - see VoterClient.h for why
  // this is deferred rather than sent synchronously mid-drain.
  voterClient.flushPendingAudio();

  // --- Jitter Buffer Playback ---
  // Allow JitterBuffer to manage its own hysteresis (buffering state) by requesting
  // audio as long as the DMA playQueue needs it. We use 30 blocks (~87ms) as
  // our low-watermark. If JitterBuffer is starved, it returns 0 and waits until
  // 200ms is accumulated before returning 160 again.
  if (AudioMemoryUsage() < 30) {
    uint8_t uBus[160];
    int16_t pcmBuf[160];

    // Retrieve from Jitter Buffer
    size_t read = voterClient.jitterBuffer.get(uBus, 160);
    if (read == 160) {
      dsp.decodeULaw(uBus, pcmBuf, 160);
      dsp.upsampleAndPlay(pcmBuf, 160, playQueue);
    }
  }

  // --- Hardware PTT Pin Drive (Host Inactivity Timer) ---
  // We key the radio based on incoming network traffic from the host, 
  // rather than the DAC output. This provides a natural ~200ms lead-time
  // while the jitter buffer fills, preventing clipped audio and pops.
  uint32_t lastHostAudio = voterClient.getLastHostAudioTime();
  bool pttActive = (lastHostAudio > 0 && (millis() - lastHostAudio < cfg.data.pttTailMs)) || g_testToneActive;
  bool pttPinState = cfg.data.pttInvert ? !pttActive : pttActive;
  digitalWrite(PTT_PIN, pttPinState);

  static bool lastReportedPtt = false;
  if (pttActive != lastReportedPtt) {
    // Serial debug removed as requested
    lastReportedPtt = pttActive;
  }
}
