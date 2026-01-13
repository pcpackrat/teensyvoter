/*
  TeensyVoter - Voter Receiver/Transmitter Firmware
  Platform: Teensy 4.1 + Audio Shield
  Network: ESP32 (SPI) or Native Ethernet

  Dependencies:
  - FNET (NativeEthernet)
  - Audio, SPI, Wire
  - TinyGPSPlus
*/

#include "ConfigManager.h"
#include "DSPProcessor.h"
#include "EspSpiDriver.h"
#include "GPSManager.h"
#include "NetworkManager.h"
#include "VoterClient.h"
#include "VoterProtocol.h"
#include "WebInterface.h"
#include <Arduino.h>
#include <Audio.h>
#include <NativeEthernet.h>
#include <SPI.h>
#include <Wire.h>
#include <arm_math.h> // CMSIS DSP Library for FIR decimator

#define RSSI_PIN A14 // Connect to voltage divider output (0-3.3V)
#define COS_PIN 41   // Hardware COS input (active HIGH/LOW depending on radio)
#define PIN_DEBUG_TX 3 // Debug / oscilloscope pin
#define PPS_PIN 2      // GPS PPS Input
// #define WIFI_SERIAL Serial5 // REMOVED (Conflicted with GPS)
#define GPS_SERIAL Serial1 // GPS Module RX/TX (Pins 0/1)

// --- Global State ---
float g_headphoneVol = 0.5f;
bool g_testToneMode = false;
float g_testTonePhase = 0.0f;

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
AudioConnection patchCord4(i2s_in, 0, i2s_out,
                           0); // Left In -> Left Out (Monitoring)
AudioConnection patchCord5(i2s_in, 0, i2s_out,
                           1); // Left In -> Right Out (Mono Mix)
AudioConnection patchCord3(mixer1, 0, recordQueue,
                           0); // Mixer -> Queue (CRITICAL FOR DSP)

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
WebInterface web;
ConfigManager cfg;

byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};

uint8_t g_simRSSI = 0; // 0 = Disabled, 1-255 = Forced Value
bool g_noSignalMode =
    false; // If true, suppress all audio sending (simulated squelch)

// -----------------------------------------------------------------------------
// Helper: Reset Audio State
// -----------------------------------------------------------------------------
void resetAudioState() {
  // Clear DSP filters
  // dsp.reset(); // If DSP class has reset

  // Reset decimation
  accHead = 0;
  g_testTonePhase = 0.0f;

  // Clear Decimator State (Filter History)
  // This resets the internal state of the CMSIS filter
  memset(decimatorState, 0, sizeof(decimatorState));

  // Clear Input Buffer
  decimatorInputLen = 0;
  // memset(decimatorInputBuf, 0, sizeof(decimatorInputBuf)); // Optional

  memset(accumulationBuf, 0, sizeof(accumulationBuf));
  recordQueue.clear();
  Serial.println("Audio State Reset");
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
  Serial.printf(" [1] Host IP     : %-15s\r\n", ipStr);
  Serial.printf(" [2] Host Port   : %-5u\r\n", cfg.data.hostPort);
  Serial.printf(" [3] RSSI Mode   : %s\r\n",
                cfg.data.useHwRSSI ? "HARDWARE (Analog)" : "SOFTWARE (DSP)");
  Serial.printf(" [4] Client PWD  : %s\r\n", cfg.data.clientPwd);
  Serial.printf(" [5] Host PWD    : %s\r\n", cfg.data.hostPwd);
  Serial.printf(" [6] COS Mode    : %s\r\n",
                cfg.data.cosMode == COS_MODE_ALWAYS_ON  ? "Always On"
                : cfg.data.cosMode == COS_MODE_HARDWARE ? "Hardware GPIO"
                                                        : "DSP Squelch");
  Serial.printf(" [7] DSP Squelch : %u\r\n", cfg.data.dspSquelchThresh);
  Serial.printf(" [R] Sim RSSI    : %u\r\n", g_simRSSI);
  Serial.printf(" [N] No Signal   : %s\r\n",
                g_noSignalMode ? "ON (No Audio)" : "OFF");
  Serial.printf(" [G] Set RX Gain  : %u (0-15)\r\n", cfg.data.rxGain);
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
  Serial.println("\r----------------------------------------");
  Serial.println("\r [S] Save & Reboot");
  Serial.println("\r [C] Resend WiFi Credentials");
  Serial.println("\r [M] Refresh Menu");
  Serial.println("\r [I] GPS Status");
  Serial.println("\r [D] Signal Monitor (Live Dashboard)");
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
      spiDriver.setCredentials("ImWatchinYou", "n0Password");
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
      Serial.print("\nEnter RX Gain (0-15, default 5): ");
      String val = readStringEcho();
      int g = val.toInt();
      if (g >= 0 && g <= 15) {
        cfg.data.rxGain = (uint8_t)g;
        sgtl5000_1.lineInLevel(cfg.data.rxGain);
        Serial.printf("\nRX Gain set to %u\n", cfg.data.rxGain);
      } else {
        Serial.println("\nInvalid Value (0-15).");
      }
      printMenu();
      break;
    }
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
    case 'n':
    case 'N': {
      g_noSignalMode = !g_noSignalMode;
      Serial.printf("\nNo Signal Mode: %s\n",
                    g_noSignalMode ? "ON (simulating squelched RX)" : "OFF");
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
      Serial.println("\n--- Signal Monitor (Press any key to exit) ---");
      while (!Serial.available()) {
        // Clear line / Return to start
        Serial.print("\r");

        // 1. RSSI
        uint8_t noise = dsp.getNoiseLevel();
        int analogRSSI = analogRead(RSSI_PIN);
        // Calculate "final" RSSI used by logic
        uint8_t finalRSSI = 0;
        if (cfg.data.useHwRSSI) {
          // Map with calibration
          long mapped =
              map(analogRSSI, cfg.data.rssiMin, cfg.data.rssiMax, 0, 255);
          if (mapped < 0)
            mapped = 0;
          if (mapped > 255)
            mapped = 255;
          finalRSSI = (uint8_t)mapped;
        } else {
          finalRSSI = (255 - noise);
        }

        // 2. COS
        bool hwCosActive = (digitalRead(COS_PIN) == LOW);
        bool dspCosActive = (noise < cfg.data.dspSquelchThresh);
        bool finalCos = (cfg.data.cosMode == COS_MODE_HARDWARE) ? hwCosActive
                        : (cfg.data.cosMode == COS_MODE_DSP)    ? dspCosActive
                                                                : true;

        // 3. GPS
        long ppsJitter = gpsMgr.getPpsJitter();
        (void)ppsJitter; // Usage in printf removed, silence warning
        bool gpsLocked = gpsMgr.isLocked();

        // Display
        Serial.printf("RSSI:%3u (ADC:%4d N:%3u) | COS:%s | GPS:%s | Min:%u "
                      "Max:%u   ",
                      finalRSSI, analogRSSI, noise, finalCos ? "ACT" : "idle",
                      gpsLocked ? "LCK" : "SRC", cfg.data.rssiMin,
                      cfg.data.rssiMax);

        delay(200);
      }
      // Consume the key that broke the loop
      if (Serial.available())
        Serial.read();
      printMenu();
      break;
    }
    }
  }
}

void setup() {
  // Force Recompile Check
  Serial.begin(115200);
  while (!Serial && millis() < 3000)
    ;
  Serial.println("[System] TeensyVoter Booting...");

  // Debug Pin for Oscilloscope
  pinMode(PIN_DEBUG_TX, OUTPUT);
  digitalWrite(PIN_DEBUG_TX, LOW);

  // COS Input Pin
  pinMode(COS_PIN, INPUT_PULLUP);

  // 0. Config (MUST BE FIRST)
  cfg.begin();

  // 1. Audio
  AudioMemory(50); // Need more memory for queues

  if (sgtl5000_1.enable()) {
    Serial.println("[Audio] SGTL5000 Audio Shield FOUND & Enabled");
  } else {
    Serial.println("[Audio] ERROR: SGTL5000 Audio Shield NOT DETECTED!");
    Serial.println("[Audio] Check I2C pins and power.");
  }

  sgtl5000_1.volume(0.5);
  sgtl5000_1.inputSelect(
      (cfg.data.inputSource == 0)
          ? AUDIO_INPUT_LINEIN
          : AUDIO_INPUT_MIC); // for radio discriminator/audio output
  sgtl5000_1.lineInLevel(cfg.data.rxGain); // Line input level (0-15)

  // Start Recording
  mixer1.gain(0, 0.5); // Left Channel
  mixer1.gain(1, 0.5); // Right Channel
  recordQueue.begin();

  // Test Tone (Disabled - connection commented out at line 65)
  // sine1.frequency(440);
  // sine1.amplitude(0.5);

  // Initialize audio state
  // resetAudioState();

  Serial.println("[Audio] SGTL5000 & Queue Initialized");

  Serial.printf("[Audio] Applied RX Gain: %u\r\n", cfg.data.rxGain);

  if (cfg.data.rxGain == 0)
    Serial.println("[WARNING] Gain is 0! Auto-Correcting to 12...");
  if (cfg.data.rxGain == 0) {
    cfg.data.rxGain = 12;
    sgtl5000_1.lineInLevel(12);
  }

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

  // Send WiFi Credentials (HARDCODED - TODO: Move to Config)
  Serial.println("[System] Sending WiFi Credentials...");
  spiDriver.setCredentials("ImWatchinYou", "n0Password");
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

  // Use a safe input block size (multiple of M=6) for Init check. 120 is safe.
  arm_status status = arm_fir_decimate_init_f32(
      &decimator, DECIMATOR_NUM_TAPS, DECIMATION_FACTOR, decimatorCoeffs,
      decimatorState, 120);

  if (status != ARM_MATH_SUCCESS) {
    Serial.print("ERROR: Decimator Init Failed! Code: ");
    Serial.println(status);
  }

  // 7. Web
  web.begin(&cfg, &gpsMgr, &voter);
  // Serial.println("[DEBUG] Minimal Mode: Only Audio + Serial Active");
}

void loop() {

  // 1. Core Updates
  handleSerialCLI();
  // Serial Passthrough Removed

  web.update();
  gpsMgr.update();
  netMgr.update();
  voter.update();

  // 2. Audio Processing Loop
  // Changed to 'if' to prevent starvation of GPS/Network if DSP is slow
  // We process up to 2 blocks per loop to catch up if needed, but yield to
  // other tasks
  int blocksProcessed = 0;
  while (recordQueue.available() >= 1 && blocksProcessed < 2) {
    blocksProcessed++;
    int16_t *buff = recordQueue.readBuffer();
    if (!buff)
      continue; // Should not happen given check above, but safer than return

    // 1. DSP Moved to Frame Processing Loop (Lines ~690)
    // We only process Decimated 8kHz Audio (160 Samples).
    // Processing here caused 44.1kHz aliasing issues.

    if (g_testToneMode) {
      // Use global phase for consistency across blocks & reset capability
      const float freq = 1000.0f;                       // 1kHz
      const float sampleRate = AUDIO_SAMPLE_RATE_EXACT; // 44117.6Hz
      const float amplitude = 5000.0f;
      const float phaseIncrement = 2.0f * 3.14159265f * freq / sampleRate;

      for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
        buff[i] = (int16_t)(amplitude * sinf(g_testTonePhase));
        g_testTonePhase += phaseIncrement;
        if (g_testTonePhase >= 2.0f * 3.14159265f)
          g_testTonePhase -= 2.0f * 3.14159265f;
      }
    }

    if (cfg.data.useHwRSSI) {
      // Just mapping code, no side effects
      int analogVal = analogRead(RSSI_PIN);
      (void)analogVal;
    }

    // 2. Fractional Resampling (44.1kHz -> 8000Hz)
    // Fixes timing drift (Pulsing) caused by integer decimation
    const float RESAMPLE_RATIO = AUDIO_SAMPLE_RATE_EXACT / 8000.0f; // ~5.5147

    // State variables (static to persist across blocks)
    static float resamplePos =
        0.0f; // Next output time relative to current block start
    static float lastFilteredSample =
        0.0f; // Previous input sample (y[i-1]) for interpolation

    // Simple Anti-Aliasing Filter State
    static float lpfState = 0.0f;
    const float lpfAlpha =
        0.42f; // ~3kHz cutoff (fc = 3000, fs = 44100 -> alpha ~ 0.42)

    // Process all 128 input samples
    for (int i = 0; i < 128; i++) {
      // A. Anti-Aliasing (IIR LPF)
      float in = (float)buff[i];
      lpfState += lpfAlpha * (in - lpfState);
      float currentSample = lpfState;

      // B. Generate Output Samples via Linear Interpolation
      // We generate an output whenever 'resamplePos' falls within the interval
      // (i-1, i] i.e., while resamplePos < i

      while (resamplePos < (float)i) {
        // Calculate interpolation fraction
        // Interval is [i-1, i]. currentSample is at i. lastFilteredSample is at
        // i-1. Fraction from i-1:
        float frac = resamplePos - ((float)i - 1.0f);

        float out =
            lastFilteredSample + frac * (currentSample - lastFilteredSample);

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

      // CRITICAL: Process Audio (Filter, De-emphasis, RSSI)
      // Note: accumulationBuf is 160 samples of int16_t.
      // DSP process takes int16_t* and modifies in-place (or uses internal
      // buffers). Wait, DSP process takes BLOCK_SIZE (128). We have 160. We
      // need to process in chunks or update DSP to handle count. But for now,
      // let's process the first 128? No, that leaves 32 unfiltered. Let's
      // modify DSP to take count? Or just loop. actually standard ARM functions
      // often work on blocks. Let's update dsp.process to take 'count'. But for
      // this quick fix, I need to check DSPProcessor.h signature. Signature:
      // uint8_t process(int16_t *samples, bool enablePLFilter, bool
      // enableDeemp); It assumes AUDIO_BLOCK_SAMPLES inside.

      // We can't easily change the block size of the DSP class without breaking
      // internal buffers. So we will process the first 128 samples to update
      // RSSI. The Remaining 32 samples will be unfiltered... that causes
      // Jitter/Artifacts!

      // FIX: We need to process ALL 160 samples.
      // Since DSP buffers are 128... this is tricky.
      // The "Voter" standard is 160 samples.
      // The "Teensy Audio" standard is 128 samples.

      // Hack: Process in two chunks?
      // Or just resize DSP buffers to 160?
      // 160 > 128. So we need to increase AUDIO_BLOCK_SAMPLES in DSPProcessor.h
      // to 160.

      // For now, I will assume I can update DSPProcessor.h to 160.

      // Let's call process anyway for now to get SOME filtering.
      // But this confirms why "Mechanical" - Mismatched block sizes!

      uint8_t measuredNoise = dsp.process(
          accumulationBuf, cfg.data.enablePLFilter, cfg.data.enableDeemp);

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
      // (This logic was inside the loop in original, but we can compute it once
      // per frame or use latest) For simplicity, we use the baseRSSI computed
      // for the last block (approximate is fine for 20ms frame)

      uint8_t finalRSSI = baseRSSI;
      switch (cfg.data.cosMode) {
      case COS_MODE_HARDWARE:
        // Hardware COS: PIN LOW = Carrier Present (Active)
        if (digitalRead(COS_PIN) == HIGH) {
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

      if (g_noSignalMode)
        finalRSSI = 0;

      bool shouldSend = (finalRSSI > 0);
      if (shouldSend) {
        // Use the proper client method which handles sequence, timestamp, and
        // sending
        VTIME frameTime = {0, 0};
        if (gpsMgr.isLocked()) {
          gpsMgr.getNetworkTime(&frameTime);
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
