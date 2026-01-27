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
#include "GPSManager.h"
#include "NetworkManager.h"
#include "VoterClient.h"
#include "VoterProtocol.h"
// #include "WebInterface.h" (Removed)

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
AudioAnalyzePeak peak1_raw;
AudioConnection patchCordMeter(mixer1, 0, peak1, 0);
AudioConnection patchCordMeterRaw(i2s_in, 0, peak1_raw, 0);

// Anti-Aliasing Filter (LPF) before Downsampling
AudioFilterBiquad lpf1;
AudioConnection patchCord3(mixer1, 0, lpf1, 0);        // Mixer -> LPF
AudioConnection patchCordLPF(lpf1, 0, recordQueue, 0); // LPF -> RecordQueue

AudioSynthWaveformSine sine1;
// AudioConnection patchCord6(sine1, 0, i2s_out, 1); // Tone Disabled

AudioControlSGTL5000 sgtl5000_1;

// --- Global Objects ---
// Select your Active Driver Here:
// EthernetDriver ethDriver;
EspSpiDriver spiDriver(26, 24, 25); // CS=26 (Uncovered), Ready=24, Reset=25
NetworkManager netMgr;
GPSManager gpsMgr;
VoterClient voter;
DSPProcessor dsp;
// WebInterface web;
ConfigManager cfg;

byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};

// uint8_t g_simRSSI = 0; // Removed

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
  Serial.print("Auth: ");
  Serial.println(cfg.data.clientPwd);
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

  // 3. Network
  Serial.println("[System] Initializing Network Driver...");
  // netMgr.begin(&ethDriver, mac);
  netMgr.begin(&spiDriver, mac);

  // Give ESP32 time to boot before sending credentials
  Serial.println("[System] Waiting for ESP32 Boot (5s)...");
  delay(5000); // Increased to 5s to match Spirit.ino startup delay

  // Send WiFi Credentials (Unconditional)
  Serial.println("[System] Sending WiFi Credentials...");
  spiDriver.setCredentials(cfg.data.wifiSSID, cfg.data.wifiPass);

  // Wait for IP (up to 15s) with Audio Maintenance
  Serial.print("[System] Waiting for WiFi Connection");
  uint32_t waitStart = millis();
  bool ipFound = false;

  while (millis() - waitStart < 15000) {
    // 1. Maintain Audio Queue (Prevent Overflow/Crash)
    while (recordQueue.available() > 0) {
      recordQueue.readBuffer();
      recordQueue.freeBuffer();
    }

    // 2. Check IP
    IPAddress myIP = netMgr.getLocalIP();
    if (myIP != IPAddress(0, 0, 0, 0)) {
      Serial.println();
      Serial.print("[System] WiFi Connected! IP: ");
      Serial.println(myIP);
      ipFound = true;
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

  delay(100);

  // 7. Voter Client
  Serial.println("[Voter] Initializing Protocol Client...");
  // voter.begin(&netMgr);

  // Serial.println("[System] Network Drivers DISABLED for Audio Test");

  // 4. GPS

  Serial.println("[GPS] Initializing GPS...");
  gpsMgr.begin(&GPS_SERIAL, PPS_PIN);

  // 4.1 Config (Moved to top)
  // cfg.begin();

  // Now we have config, set network target
  netMgr.setTarget(cfg.getHostIP(), cfg.data.hostPort);

  Serial.print("[System] Voter Target: ");
  Serial.print(cfg.getHostIP());
  Serial.printf(":%u\r\n", cfg.data.hostPort);

  // 5. Voter Client

  // 5. Voter Client
  // Serial.println("[Voter] Initializing Protocol Client...");
  voter.begin(&netMgr, &gpsMgr, cfg.getHostIP(), cfg.data.hostPort,
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
                  voter.isConnected() ? "CONNECTED" : "DISCONNECTED");
    Serial.println(" -- GPS Status --");
    Serial.printf(" Locked       : %s\r\n", gpsMgr.isLocked() ? "YES" : "NO");
    Serial.printf(" Satellites   : %u\r\n", gpsMgr.getSatellites());
    Serial.printf(" Time Set     : %s\r\n", gpsMgr.isTimeSet() ? "YES" : "NO");
    if (gpsMgr.isTimeSet()) {
      VTIME t;
      gpsMgr.getNetworkTime(&t);
      Serial.printf(" Voter Time   : %u.%09u\r\n", t.vtime_sec, t.vtime_nsec);
    }
    Serial.printf(" PPS Jitter   : %u us\r\n", gpsMgr.getPpsJitter());
    char lat[16], lon[16], elev[16];
    gpsMgr.getGPSStrings(lat, lon, elev);
    Serial.printf(" Location     : %s, %s (Elev: %s m)\r\n", lat, lon, elev);
    Serial.println("----------------------------------------");
    Serial.println(" [R] Refresh GPS Data");
    Serial.println(" [x] Back to Main Menu");
  } else if (g_menuState == MENU_NETWORK) {
    Serial.println("           NETWORK MENU");
    Serial.println("========================================");
    IPAddress hIP(cfg.data.hostIP);
    Serial.printf(" [3] Host IP     : %u.%u.%u.%u\r\n", hIP[0], hIP[1], hIP[2],
                  hIP[3]);
    Serial.printf(" [4] Host Port   : %u\r\n", cfg.data.hostPort);
    Serial.printf(" [5] Client PWD  : %s\r\n", cfg.data.clientPwd);
    Serial.printf(" [6] Host PWD    : %s\r\n", cfg.data.hostPwd);
    Serial.println("----------------------------------------");
    Serial.printf(" [W] WiFi SSID   : %s\r\n", cfg.data.wifiSSID);
    Serial.println(" [P] Set WiFi Password");
    Serial.println(" [R] Resend WiFi Credentials to ESP32");
    Serial.println("----------------------------------------");
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
    Serial.println(" [x] Back to Main Menu");
  } else if (g_menuState == MENU_DEBUG) {
    Serial.println("           DEBUG MENU");
    Serial.println("========================================");
    Serial.printf(" [T] Test Tone    : %s\r\n", g_testToneMode ? "ON" : "OFF");
    Serial.println(" [D] Signal Monitor (Live Dashboard)");
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
      }
    } else if (g_menuState == MENU_NETWORK) {
      switch (c) {
      case '3': {
        Serial.print("\r\nEnter Host IP: ");
        String ipStr = readStringEcho();
        IPAddress ip;
        if (ip.fromString(ipStr)) {
          cfg.data.hostIP =
              (ip[0] << 24) | (ip[1] << 16) | (ip[2] << 8) | ip[3];
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
      case 'r':
      case 'R':
        Serial.println("Resending Credentials...");
        spiDriver.setCredentials(cfg.data.wifiSSID, cfg.data.wifiPass);
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
      }
    } else if (g_menuState == MENU_DEBUG) {
      switch (c) {
      case 't':
      case 'T':
        g_testToneMode = !g_testToneMode;
        resetAudioState();
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
          gpsMgr.update();
          netMgr.update();
          voter.update();
          if (peak1.available()) {
            float v = peak1.read();
            bool isCosActive = false;
            if (cfg.data.cosMode == COS_MODE_HARDWARE) {
              isCosActive =
                  (digitalRead(COS_PIN) == (cfg.data.cosInvert ? HIGH : LOW));
            } else if (cfg.data.cosMode == COS_MODE_DSP) {
              float v_raw = peak1_raw.available() ? peak1_raw.read() : 0.0f;
              int dspVal = (int)(v_raw * 255.0f);
              isCosActive = (dspVal > cfg.data.dspSquelchThresh);
              if (cfg.data.cosInvert)
                isCosActive = !isCosActive;
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
  // Auto-Show Menu on Connect
  static bool wasConnected = false;
  // Frame Counting State (Moved to top level scope)
  static uint32_t lastEpoch = 0;
  static uint32_t framesSent = 0;

  if (voter.isConnected() && !wasConnected) {
    Serial.println();
    printMenu();
  }
  wasConnected = voter.isConnected();

  // 1. Core Updates
  handleSerialCLI();
  // Serial Passthrough Removed

  gpsMgr.update();
  netMgr.update();
  voter.update();

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
      gpsMgr.getNetworkTime(&frameTime);

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

      // g_noSignalMode logic removed

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
        if (gpsMgr.isTimeSet()) {
          // STRICT FRAME COUNTING STRATEGY
          // Decouples timestamp from micros() jitter/phase drift.

          uint32_t currentEpoch = gpsMgr.getEpoch();

          if (currentEpoch != lastEpoch) {
            lastEpoch = currentEpoch;
            framesSent = 0;
          }

          // Basic Ideal Timestamp
          frameTime.vtime_sec = currentEpoch;
          frameTime.vtime_nsec = framesSent * 20000000; // 20ms in ns

          // Normalize NSEC
          while (frameTime.vtime_nsec >= 1000000000) {
            frameTime.vtime_nsec -= 1000000000;
            frameTime.vtime_sec++;
          }

          // Offset Logic (Maintained from previous step)
          uint32_t delayNs = 180000000; // 180ms
          // Subtract Delay safely
          if (frameTime.vtime_nsec >= delayNs) {
            frameTime.vtime_nsec -= delayNs;
          } else {
            // Borrow from seconds
            frameTime.vtime_sec--;
            frameTime.vtime_nsec =
                (1000000000 + frameTime.vtime_nsec) - delayNs;
          }
        }
        voter.processAudioFrame(ulawFrame, finalRSSI, frameTime);
        if (gpsMgr.isTimeSet()) {
          framesSent++;
        }
        // Serial.println("[Test] Generated Audio Frame (Not Sent)");
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
