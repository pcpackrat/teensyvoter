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
uint8_t g_digitalGainPct = 100; // Default 100% (Unity)
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
AudioConnection patchCordMeter(mixer1, 0, peak1, 0);

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
  Serial.println("[System] Press any key for Menu...");

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

void printMenu() {
  IPAddress ip = cfg.getHostIP();
  char ipStr[20];
  sprintf(ipStr, "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);

  Serial.println("\r\n========================================");
  Serial.println("\r           TEENSY VOTER MENU            ");
  Serial.println("\r========================================");
  IPAddress local = netMgr.getLocalIP();
  Serial.printf(" [Status] Device IP: %u.%u.%u.%u\r\n", local[0], local[1],
                local[2], local[3]);
  Serial.printf(" [1] Host IP     : %-15s\r\n", ipStr);
  Serial.printf(" [2] Host Port   : %-5u\r\n", cfg.data.hostPort);
  Serial.printf(" [W] WiFi SSID   : %s\r\n", cfg.data.wifiSSID);
  Serial.printf(" [3] RSSI Mode   : %s\r\n",
                cfg.data.useHwRSSI ? "HARDWARE (Analog)" : "SOFTWARE (DSP)");
  Serial.printf(" [4] Client PWD  : %s\r\n", cfg.data.clientPwd);
  Serial.printf(" [5] Host PWD    : %s\r\n", cfg.data.hostPwd);
  Serial.printf(" [6] COS Mode    : %s\r\n",
                cfg.data.cosMode == COS_MODE_ALWAYS_ON  ? "Always On"
                : cfg.data.cosMode == COS_MODE_HARDWARE ? "Hardware GPIO"
                                                        : "DSP Squelch");
  Serial.printf(" [7] DSP Squelch : %u\r\n", cfg.data.dspSquelchThresh);

  Serial.printf(" [G] Hardware pre amp  : %u (0-15)\r\n", cfg.data.rxGain);
  Serial.printf(" [H] Headphone Vol: %d (0-100)\r\n",
                (int)(g_headphoneVol * 100));
  Serial.printf(" [L] Input Source: %s\r\n",
                cfg.data.inputSource == AUDIO_INPUT_MIC ? "MIC" : "LINE IN");
  Serial.printf(" [T] Test Tone    : %s\r\n",
                g_testToneMode ? "ON (1kHz sine wave)" : "OFF");
  Serial.println("----------------------------------------");
  Serial.printf(" [8] Cal Min RSSI: %u (Current: %d)\r\n", cfg.data.rssiMin,
                analogRead(RSSI_PIN));
  Serial.printf(" [9] Cal Max RSSI: %u (Current: %d)\r\n", cfg.data.rssiMax,
                analogRead(RSSI_PIN));
  Serial.printf(" [F] Digital Gain: %u%%\r\n", g_digitalGainPct);
  // Serial.printf(" [A] Post Gain   : %.2f\r\n", g_postGain);
  Serial.printf(" [E] De-emphasis : %s\r\n",
                cfg.data.enableDeemp ? "ON" : "OFF");
  Serial.printf(" [K] PL Filter   : %s\r\n",
                cfg.data.enablePLFilter ? "ON" : "OFF");
  Serial.println("\r----------------------------------------");
  Serial.println("\r [S] Save & Reboot");
  Serial.println("\r [C] Resend WiFi Credentials");
  Serial.println("\r [M] Refresh Menu");
  Serial.println("\r [I] GPS Status");
  Serial.println("\r [D] Signal Monitor (Live Dashboard)");
  Serial.printf(" [O] Invert COS: %s\r\n",
                cfg.data.cosInvert ? "YES (Active High)" : "NO (Active Low)");
  Serial.println("\r [W] Set WiFi SSID");
  Serial.println("\r [P] Set WiFi Password");
  Serial.println("========================================\r\n");
  Serial.print("> ");
}

void handleSerialCLI() {
  if (Serial.available()) {
    char c = Serial.read();
    switch (c) {
    case 'c':
    case 'C':
      Serial.println("\nResending WiFi Credentials...");
      spiDriver.setCredentials(cfg.data.wifiSSID, cfg.data.wifiPass);
      break;
    case 'm':
    case 'M':
      printMenu();
      break;
    case '1': {
      Serial.print("\nEnter New Host IP: ");
      // Use new non-blocking-ish echo reader
      String ipStr = readStringEcho();
      ipStr.trim();
      IPAddress newIP;
      if (newIP.fromString(ipStr)) {
        cfg.setHostIP(newIP);
        Serial.print("Updated Host IP to: ");
        Serial.println(newIP);
      } else {
        Serial.println("Invalid IP Address!");
      }
      // Reprint menu to show change
      printMenu();
      break;
      break;
    }
    case '2': {
      Serial.print("\nEnter New Host Port: ");
      String portStr = readStringEcho();
      int port = portStr.toInt();
      if (port > 0 && port < 65535) {
        cfg.data.hostPort = (uint16_t)port;
        Serial.printf("\nUpdated Port to: %u\n", cfg.data.hostPort);
      } else {
        Serial.println("\nInvalid Port!");
      }
      printMenu();
      break;
    }
    case '3': {
      cfg.data.useHwRSSI = !cfg.data.useHwRSSI;
      Serial.printf("\nToggled RSSI Mode to: %s\n",
                    cfg.data.useHwRSSI ? "HARDWARE" : "SOFTWARE");
      printMenu();
      break;
    }
    case '4': {
      Serial.print("\nEnter Client Password: ");
      String pwd = readStringEcho();
      pwd.trim();
      if (pwd.length() < 20) {
        strcpy(cfg.data.clientPwd, pwd.c_str());
        Serial.println("\nUpdated Client Password.");
      } else {
        Serial.println("\nPassword too long (Max 19 chars)!");
      }
      printMenu();
      break;
    }
    case '5': {
      Serial.print("\nEnter Host Password: ");
      String pwd = readStringEcho();
      pwd.trim();
      if (pwd.length() < 20) {
        strcpy(cfg.data.hostPwd, pwd.c_str());
        Serial.println("\nUpdated Host Password.");
      } else {
        Serial.println("\nPassword too long (Max 19 chars)!");
      }
      printMenu();
      break;
    }
    case 's':
    case 'S':
      cfg.save();
      Serial.println("\nSaving Config & Rebooting...");
      delay(1000);
      SCB_AIRCR = 0x05FA0004; // System Reset
      break;
    case 'g':
    case 'G': {
      Serial.print("\nEnter Hardware pre amp (0-15, default 5): ");
      String val = readStringEcho();
      int g = val.toInt();
      if (g >= 0 && g <= 15) {
        cfg.data.rxGain = (uint8_t)g;
        sgtl5000_1.lineInLevel(cfg.data.rxGain);
        Serial.printf("\nHardware pre amp set to %u\n", cfg.data.rxGain);
      } else {
        Serial.println("\nInvalid Value (0-15).");
      }
      printMenu();
      break;
    }
    case 'f':
    case 'F': {
      Serial.print("\nEnter Digital Gain % (0-200, Default 100): ");
      String val = readStringEcho();
      int g = val.toInt();
      if (g >= 0 && g <= 500) { // Limit to 500% just in case
        g_digitalGainPct = (uint8_t)g;
        float gainFactor = (float)g_digitalGainPct / 100.0f;
        mixer1.gain(0, gainFactor);
        mixer1.gain(1, gainFactor);
        Serial.printf("\nDigital Gain set to %u%% (Factor: %.2f)\n",
                      g_digitalGainPct, gainFactor);
      } else {
        Serial.println("\nInvalid Value (0-500).");
      }
      printMenu();
      break;
    }
    /*
    case 'a':
    case 'A': {
      Serial.print("\nEnter Post Gain (0.0 - 2.0): ");
      String val = readStringEcho();
      float g = val.toFloat();
      if (g >= 0.0f && g <= 5.0f) {
        g_postGain = g;
        // postAmp.gain(g_postGain); // Removed: Software Scaling used instead
        Serial.printf("\nPost Gain set to %.2f\n", g_postGain);
      } else {
        Serial.println("\nInvalid Value.");
      }
      printMenu();
      break;
    }
    */
    case 'e':
    case 'E': {
      cfg.data.enableDeemp = !cfg.data.enableDeemp;
      Serial.printf("\nDe-emphasis toggled %s\n",
                    cfg.data.enableDeemp ? "ON" : "OFF");
      printMenu();
      break;
    }
    case 'k':
    case 'K': {
      cfg.data.enablePLFilter = !cfg.data.enablePLFilter;
      Serial.printf("\nPL Filter toggled %s\n",
                    cfg.data.enablePLFilter ? "ON" : "OFF");
      printMenu();
      break;
    }
    /*
    case 'q':
    case 'Q': {
      g_dspSilenceMode = !g_dspSilenceMode;
      Serial.printf("\nDSP Silence Mode toggled %s\n",
                    g_dspSilenceMode ? "ON" : "OFF");
      printMenu();
      break;
    }
    */
    case 'h':
    case 'H': {
      Serial.print("\nEnter Headphone Vol (0-100): ");
      String val = readStringEcho();
      int v = val.toInt();
      if (v >= 0 && v <= 100) {
        g_headphoneVol = (float)v / 100.0f;
        sgtl5000_1.volume(g_headphoneVol);
        Serial.printf("\nHeadphone Volume set to %d\n", v);
      } else {
        Serial.println("\nInvalid Value (0-100).");
      }
      printMenu();
      break;
    }
    case 'l':
    case 'L': {
      if (cfg.data.inputSource == AUDIO_INPUT_LINEIN) {
        cfg.data.inputSource = AUDIO_INPUT_MIC;
        sgtl5000_1.inputSelect(AUDIO_INPUT_MIC);
        sgtl5000_1.micGain(40); // Default robust mic gain
        Serial.println("\nInput switched to MIC (Gain 40dB)");
      } else {
        cfg.data.inputSource = AUDIO_INPUT_LINEIN;
        sgtl5000_1.inputSelect(AUDIO_INPUT_LINEIN);
        sgtl5000_1.lineInLevel(cfg.data.rxGain);
        Serial.println("\nInput switched to LINE IN");
      }
      printMenu();
      break;
    }
    case 'i':
    case 'I':
      Serial.println("\r\n--- GPS Status ---");
      Serial.printf("Locked    : %s\r\n", gpsMgr.isLocked() ? "YES" : "NO");
      Serial.printf("Time Set  : %s\r\n", gpsMgr.isTimeSet() ? "YES" : "NO");
      if (gpsMgr.isTimeSet()) {
        VTIME t;
        gpsMgr.getNetworkTime(&t);
        Serial.printf("Voter Time: %u.%09u\r\n", t.vtime_sec, t.vtime_nsec);
      }
      Serial.printf("PPS Jitter: %u us\r\n", gpsMgr.getPpsJitter());
      // Re-print menu after a pause or keypress?
      // For now just back to prompt
      Serial.println("------------------\r");
      Serial.print("> ");
      Serial.print("> ");
      break;
    /*
    case 'r':
    case 'R': {
      Serial.print("\nEnter Simulated RSSI (0=Disable, 1-255): ");
      String val = readStringEcho(); // Use helper
      int r = val.toInt();
      if (r >= 0 && r <= 255) {
        g_simRSSI = (uint8_t)r;
        Serial.printf("\nSimulated RSSI set to %u\n", g_simRSSI);
      } else {
        Serial.println("\nInvalid Value (0-255).");
      }
      printMenu();
      break;
    }
    */
    case '6': {
      Serial.println("\nSelect COS Mode:");
      Serial.println(" [0] Always On (No Squelch)");
      Serial.println(" [1] Hardware COS (GPIO Pin)");
      Serial.println(" [2] DSP Squelch (Noise Detection)");
      Serial.print("Enter mode: ");
      String val = readStringEcho();
      int mode = val.toInt();
      if (mode >= 0 && mode <= 2) {
        cfg.data.cosMode = mode;
        Serial.printf("\nCOS Mode set to %d\n", mode);
      } else {
        Serial.println("\nInvalid Mode (0-2).");
      }
      printMenu();
      break;
    }
    case '7': {
      Serial.print("\nEnter DSP Squelch Threshold (0-255): ");
      String val = readStringEcho();
      int thresh = val.toInt();
      if (thresh >= 0 && thresh <= 255) {
        cfg.data.dspSquelchThresh = thresh;
        Serial.printf("\nDSP Squelch Threshold set to %u\n", thresh);
      } else {
        Serial.println("\nInvalid Value (0-255).");
      }
      printMenu();
      break;
    }

    case '8': {
      int val = analogRead(RSSI_PIN);
      cfg.data.rssiMin = (uint16_t)val;
      Serial.printf("\nSet Min RSSI (0%%) to: %u\n", val);
      printMenu();
      break;
    }
    case '9': {
      int val = analogRead(RSSI_PIN);
      cfg.data.rssiMax = (uint16_t)val;
      Serial.printf("\nSet Max RSSI (100%%) to: %u\n", val);
      printMenu();
      break;
    }
    /*
    case 'n':
    case 'N': {
      g_noSignalMode = !g_noSignalMode;
      Serial.printf("\nNo Signal Mode: %s\n",
                    g_noSignalMode ? "ON (simulating squelched RX)" : "OFF");
      printMenu();
      break;
      printMenu();
      break;
    }
    */
    case 'o':
    case 'O': {
      cfg.data.cosInvert = !cfg.data.cosInvert;
      Serial.printf("\nInvert COS set to: %s\n",
                    cfg.data.cosInvert ? "YES" : "NO");
      printMenu();
      break;
    }
    case 't':
    case 'T': {
      g_testToneMode = !g_testToneMode;
      resetAudioState(); // CRITICAL: Reset filters and buffers
      Serial.printf("\nTest Tone Mode: %s\n",
                    g_testToneMode ? "ON (1kHz sine wave)" : "OFF");
      printMenu();
      break;
    }
    case 'd':
    case 'D': {
      Serial.println("\n--- Live Tuner (Press 'x' to Exit) ---");
      Serial.println("Controls: '['/']' = HW Gain | '-'/'=' = Digital Gain");

      while (true) {
        // Handle Input
        if (Serial.available()) {
          char c = Serial.read();
          if (c == 'x' || c == 'X')
            break; // Exit

          bool changed = false;
          // Hardware Gain
          if (c == ']') {
            if (cfg.data.rxGain < 15)
              cfg.data.rxGain++;
            sgtl5000_1.lineInLevel(cfg.data.rxGain);
            changed = true;
          }
          if (c == '[') {
            if (cfg.data.rxGain > 0)
              cfg.data.rxGain--;
            sgtl5000_1.lineInLevel(cfg.data.rxGain);
            changed = true;
          }
          // Digital Gain (- to decrease, = to increase)
          if (c == '=') {
            if (g_digitalGainPct < 250)
              g_digitalGainPct += 5;
            changed = true;
          }
          if (c == '-') {
            if (g_digitalGainPct >= 5)
              g_digitalGainPct -= 5;
            changed = true;
          }

          if (changed) {
            float factor = (float)g_digitalGainPct / 100.0f;
            mixer1.gain(0, factor);
            // mixer1.gain(1, factor); // REMOVED: Keep Right Channel Muted!
          }
        }

        // Update Display (Limit refresh rate)
        static uint32_t lastPrint = 0;
        if (millis() - lastPrint > 50) {
          lastPrint = millis();

          float val = 0;
          if (peak1.available())
            val = peak1.read();

          int bars = (int)(val * 30.0f);
          if (bars > 30)
            bars = 30;

          Serial.print("\rLevel:[");
          for (int i = 0; i < 30; i++)
            Serial.print(i < bars ? "#" : " ");
          Serial.print("]");

          // Values
          Serial.printf(" Val:%.3f | HW:%2u | Dig:%3u%% | COS:%d ", val,
                        cfg.data.rxGain, g_digitalGainPct,
                        digitalRead(COS_PIN));

          if (val >= 0.99f)
            Serial.print("CLIP!");
          else
            Serial.print("     "); // Clear artifact
        }
      }
      printMenu();
      break;
    }
    case 'w':
    case 'W': {
      Serial.print("\nEnter WiFi SSID: ");
      String val = readStringEcho();
      val.trim();
      if (val.length() > 0 && val.length() < 32) {
        strcpy(cfg.data.wifiSSID, val.c_str());
        Serial.printf("\nWiFi SSID set to: %s\n", cfg.data.wifiSSID);
      } else {
        Serial.println("\nInvalid Length!");
      }
      printMenu();
      break;
    }
    case 'p':
    case 'P': {
      Serial.print("\nEnter WiFi Password: ");
      String val = readStringEcho();
      val.trim();
      if (val.length() > 0 && val.length() < 64) {
        strcpy(cfg.data.wifiPass, val.c_str());
        Serial.println("\nWiFi Password Updated.");
      } else {
        Serial.println("\nInvalid Length!");
      }
      printMenu();
      break;
    }
    }
  }
}

void loop() {

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
        baseRSSI = 255 - measuredNoise;
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
          static uint32_t lastEpoch = 0;
          static uint32_t framesSent = 0;

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

          framesSent++;

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
