#include "driver/spi_slave.h"
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>

// --- Configuration ---
// VSPI Pins
#define GPIO_MOSI 23
#define GPIO_MISO 19
#define GPIO_SCLK 18
#define GPIO_CS 5
#define GPIO_READY 22

#define RCV_HOST VSPI_HOST
#define DMA_CHAN 2

#define MAX_SPI_BUF 512

// SPI Protocol (must match SpiProtocol.h on Teensy)
#define CMD_SEND_UDP 0x10
#define CMD_SET_CONFIG 0x02
#define CMD_GET_IP 0x03
#define CMD_DNS_LOOKUP 0x04
#define CMD_GET_DNS 0x05
#define CMD_PUSH_CONFIG 0x06

#define STATUS_HAS_DATA 0x01
#define STATUS_CONFIG_CMD 0x40

#define CFG_CMD_SET_PARAM 0x07
#define CFG_CMD_SAVE_REBOOT 0x08
#define CFG_CMD_REQUEST_CONFIG 0x09

// Parameter IDs
#define PARAM_HOST_IP 0x01
#define PARAM_HOST_PORT 0x02
#define PARAM_HOSTNAME 0x03
#define PARAM_CLIENT_PWD 0x04
#define PARAM_HOST_PWD 0x05
#define PARAM_WIFI_SSID 0x06
#define PARAM_WIFI_PASS 0x07
#define PARAM_RX_GAIN 0x10
#define PARAM_TX_GAIN_PCT 0x11
#define PARAM_COS_MODE 0x12
#define PARAM_COS_INVERT 0x13
#define PARAM_DSP_SQUELCH 0x14
#define PARAM_USE_HW_RSSI 0x15
#define PARAM_RSSI_MIN 0x16
#define PARAM_RSSI_MAX 0x17
#define PARAM_PL_FILTER 0x18
#define PARAM_DEEMP 0x19
#define PARAM_PTT_INVERT 0x1A
#define PARAM_PTT_TAIL_MS 0x1B
#define PARAM_DSP_CALIB 0x1C
#define PARAM_INPUT_SOURCE 0x1D

// --- SysConfig Mirror (must match Teensy's SysConfig struct layout) ---
struct SysConfigMirror {
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


// --- Globals ---
WiFiUDP udp;
WebServer webServer(80);

// Static DMA Buffers (Aligned)
WORD_ALIGNED_ATTR char sendbuf[MAX_SPI_BUF];
WORD_ALIGNED_ATTR char recvbuf[MAX_SPI_BUF];
spi_slave_transaction_t t;

volatile bool dataPending = false;
volatile uint32_t cbCount = 0;

// Config cache from Teensy
SysConfigMirror cachedConfig;
bool configReceived = false;

// Pending config command to send back to Teensy
volatile bool cfgCmdPending = false;
uint8_t cfgCmdBuf[64];
uint8_t cfgCmdLen = 0;

// CSS Style (shared across pages)
const char* HTML_STYLE = R"rawliteral(
<style>
body{font-family:Arial,sans-serif;margin:0;padding:20px;background:#2c3e50;color:#ecf0f1}
.container{max-width:800px;margin:0 auto}
.card{background:#34495e;padding:20px;margin:10px 0;border-radius:8px}
h1{color:#3498db;margin-top:0}
h2{color:#ecf0f1;border-bottom:2px solid #3498db;padding-bottom:10px}
h3{color:#3498db;border-bottom:1px solid #7f8c8d;padding-bottom:5px;margin-top:25px}
.btn{background:#3498db;color:#fff;border:none;padding:10px 20px;border-radius:4px;cursor:pointer;font-size:16px;margin:5px}
.btn:hover{background:#2980b9}
label{display:block;margin:15px 0 5px 0;font-weight:bold;color:#bdc3c7}
label.checkbox{display:flex;align-items:center;font-weight:normal;margin:10px 0;color:#ecf0f1;cursor:pointer}
label.checkbox input[type="checkbox"]{width:auto;margin:0 10px 0 0}
input[type="number"],input[type="text"],input[type="password"],select{width:100%;padding:10px;margin:5px 0;border:1px solid #7f8c8d;border-radius:4px;box-sizing:border-box;background:#ecf0f1;color:#2c3e50;font-size:14px}
p small{color:#95a5a6;display:block;margin-top:-5px;margin-bottom:15px}
.nav a{color:#3498db;text-decoration:none;margin:0 15px;font-weight:bold}
.nav a:hover{color:#ecf0f1}
.status{display:inline-block;padding:5px 10px;border-radius:4px}
.status.ok{background:#27ae60;color:#fff}
.status.warn{background:#f39c12;color:#fff}
</style>
)rawliteral";

// --- SPI Callback ---
void IRAM_ATTR spi_post_setup_cb(spi_slave_transaction_t *trans) {
  cbCount++;
  if (dataPending || cfgCmdPending) {
    digitalWrite(GPIO_READY, HIGH);
  }
}

// --- Queue a config command to be sent to Teensy on next SPI transaction ---
void queueConfigCmd(uint8_t *data, uint8_t len) {
  if (len > sizeof(cfgCmdBuf)) return;
  memcpy(cfgCmdBuf, data, len);
  cfgCmdLen = len;

  // Prep sendbuf
  memset(sendbuf, 0, MAX_SPI_BUF);
  sendbuf[0] = STATUS_CONFIG_CMD;
  sendbuf[1] = (len >> 8) & 0xFF;
  sendbuf[2] = len & 0xFF;
  memcpy(&sendbuf[3], data, len);

  cfgCmdPending = true;
  digitalWrite(GPIO_READY, HIGH);
}

// --- Helper: IP to string ---
String ipToStr(uint32_t ip) {
  return String(ip & 0xFF) + "." + String((ip >> 8) & 0xFF) + "." +
         String((ip >> 16) & 0xFF) + "." + String((ip >> 24) & 0xFF);
}

// --- Web Server Handlers ---

String htmlHeader(const char* title) {
  String html = "<!DOCTYPE html><html><head>";
  html += "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">";
  html += "<title>TeensyVoter - " + String(title) + "</title>";
  html += HTML_STYLE;
  html += "</head><body><div class=\"container\">";
  html += "<h1>TeensyVoter Configuration</h1>";
  html += "<div class=\"nav\">";
  html += "<a href=\"/\">Status</a>";
  html += "<a href=\"/voter\">Voter</a>";
  html += "<a href=\"/audio\">Audio</a>";
  html += "<a href=\"/wifi\">WiFi</a>";
  html += "</div>";
  return html;
}

String htmlFooter() {
  return "</div></body></html>";
}

void handleRoot() {
  String html = htmlHeader("Status");
  html += "<div class=\"card\">";
  html += "<h2>System Status</h2>";

  if (!configReceived) {
    html += "<p><span class=\"status warn\">Waiting for config from Teensy...</span></p>";
  } else {
    html += "<p><strong>Network:</strong> WiFi (ESP32 SPI)</p>";
    html += "<p><strong>WiFi IP:</strong> " + WiFi.localIP().toString() + "</p>";
    html += "<p><strong>WiFi SSID:</strong> " + String(cachedConfig.wifiSSID) + "</p>";
    html += "<p><strong>RSSI:</strong> " + String(WiFi.RSSI()) + " dBm</p>";
    html += "<hr>";

    String hostStr;
    if (cachedConfig.hostname[0] != '\0') {
      hostStr = String(cachedConfig.hostname);
    } else {
      hostStr = ipToStr(cachedConfig.hostIP);
    }
    html += "<p><strong>Voter Host:</strong> " + hostStr + ":" + String(cachedConfig.hostPort) + "</p>";
    html += "<hr>";
    html += "<p style=\"color:#bdc3c7;font-size:0.8em;text-align:center\">";
    html += "ESP32 Web Interface v1.0<br>Serving config for Teensy Firmware v" + String(cachedConfig.version) + "</p>";
  }

  html += "</div>";
  html += htmlFooter();
  webServer.send(200, "text/html", html);
}

void handleVoter() {
  String html = htmlHeader("Voter");
  html += "<div class=\"card\">";
  html += "<h2>Voter Configuration</h2>";

  if (!configReceived) {
    html += "<p>Waiting for config...</p>";
  } else {
    html += "<form method=\"POST\" action=\"/api/config/voter\">";

    // Host Address
    html += "<label>Host Address (IP or hostname):</label>";
    String hostVal = cachedConfig.hostname[0] ? String(cachedConfig.hostname) : ipToStr(cachedConfig.hostIP);
    html += "<input type=\"text\" name=\"host\" value=\"" + hostVal + "\">";

    // Port
    html += "<label>Port:</label>";
    html += "<input type=\"number\" name=\"port\" value=\"" + String(cachedConfig.hostPort) + "\">";

    // Passwords
    html += "<label>Client Password:</label>";
    html += "<input type=\"password\" name=\"clientpwd\" value=\"" + String(cachedConfig.clientPwd) + "\">";
    html += "<label>Host Password:</label>";
    html += "<input type=\"password\" name=\"hostpwd\" value=\"" + String(cachedConfig.hostPwd) + "\">";

    // TX Gain
    html += "<label>Transmit Gain (0-100%):</label>";
    html += "<input type=\"number\" name=\"txgain\" min=\"0\" max=\"100\" value=\"" + String(cachedConfig.radioTxMasterGainPct) + "\">";

    html += "<button type=\"submit\" class=\"btn\">Save &amp; Reboot</button>";
    html += "</form>";
    html += "<p><small>Note: Changes require reboot to take effect</small></p>";
  }

  html += "</div>";
  html += htmlFooter();
  webServer.send(200, "text/html", html);
}

void handleAudio() {
  String html = htmlHeader("Audio");
  html += "<div class=\"card\">";
  html += "<h2>Radio &amp; Audio Configuration</h2>";

  if (!configReceived) {
    html += "<p>Waiting for config...</p>";
  } else {
    html += "<form method=\"POST\" action=\"/api/config/audio\">";

    // RX Gain
    html += "<h3>Hardware Gain</h3>";
    html += "<label>RX Gain (0-15):</label>";
    html += "<input type=\"number\" name=\"rxgain\" min=\"0\" max=\"15\" value=\"" + String(cachedConfig.radioRxAnalogGain) + "\">";
    html += "<p><small>SGTL5000 Line Input Level</small></p>";

    // TX Gain
    html += "<label>TX Gain (0-100%):</label>";
    html += "<input type=\"number\" name=\"txgain\" min=\"0\" max=\"100\" value=\"" + String(cachedConfig.radioTxMasterGainPct) + "\">";

    // DSP Calibration
    html += "<label>RSSI Calibration Factor (DSP):</label>";
    html += "<input type=\"number\" name=\"dspgain\" min=\"0\" max=\"50\" step=\"0.1\" value=\"" + String(cachedConfig.dspCalib, 1) + "\">";
    html += "<p><small>Adjusts RSSI sensitivity (Default: 13.0)</small></p>";

    // Input Source
    html += "<label>Input Source:</label>";
    html += "<select name=\"inputsource\">";
    html += "<option value=\"0\"" + String(cachedConfig.inputSource == 0 ? " selected" : "") + ">Line In</option>";
    html += "<option value=\"1\"" + String(cachedConfig.inputSource == 1 ? " selected" : "") + ">Microphone</option>";
    html += "</select>";

    // COS
    html += "<h3>COS (Carrier Operated Squelch)</h3>";
    html += "<label>COS Mode:</label>";
    html += "<select name=\"cosmode\">";
    html += "<option value=\"0\"" + String(cachedConfig.cosMode == 0 ? " selected" : "") + ">Always On</option>";
    html += "<option value=\"1\"" + String(cachedConfig.cosMode == 1 ? " selected" : "") + ">Hardware</option>";
    html += "<option value=\"2\"" + String(cachedConfig.cosMode == 2 ? " selected" : "") + ">DSP</option>";
    html += "</select>";

    html += "<label class=\"checkbox\"><input type=\"checkbox\" name=\"cosinvert\" value=\"1\"";
    if (cachedConfig.cosInvert) html += " checked";
    html += ">Invert COS Polarity</label>";

    html += "<label>DSP Squelch Threshold (0-255):</label>";
    html += "<input type=\"number\" name=\"squelch\" min=\"0\" max=\"255\" value=\"" + String(cachedConfig.dspSquelchThresh) + "\">";

    // PTT
    html += "<h3>PTT (Transmit)</h3>";
    html += "<label class=\"checkbox\"><input type=\"checkbox\" name=\"pttinvert\" value=\"1\"";
    if (cachedConfig.pttInvert) html += " checked";
    html += ">Invert PTT Polarity</label>";

    html += "<label>PTT Tail (ms):</label>";
    html += "<input type=\"number\" name=\"ptttail\" min=\"0\" max=\"1000\" value=\"" + String(cachedConfig.pttTailMs) + "\">";

    // RSSI
    html += "<h3>RSSI</h3>";
    html += "<label class=\"checkbox\"><input type=\"checkbox\" name=\"hwrssi\" value=\"1\"";
    if (cachedConfig.useHwRSSI) html += " checked";
    html += ">Use Hardware RSSI</label>";

    html += "<label>RSSI Min (ADC):</label>";
    html += "<input type=\"number\" name=\"rssimin\" min=\"0\" max=\"1023\" value=\"" + String(cachedConfig.rssiMin) + "\">";
    html += "<label>RSSI Max (ADC):</label>";
    html += "<input type=\"number\" name=\"rssimax\" min=\"0\" max=\"1023\" value=\"" + String(cachedConfig.rssiMax) + "\">";

    // Filters
    html += "<h3>Audio Filtering</h3>";
    html += "<label class=\"checkbox\"><input type=\"checkbox\" name=\"plfilter\" value=\"1\"";
    if (cachedConfig.enablePLFilter) html += " checked";
    html += ">Enable PL Filter (300Hz HPF)</label>";

    html += "<label class=\"checkbox\"><input type=\"checkbox\" name=\"deemph\" value=\"1\"";
    if (cachedConfig.enableDeemp) html += " checked";
    html += ">Enable De-Emphasis</label>";

    html += "<button type=\"submit\" class=\"btn\">Save &amp; Reboot</button>";
    html += "</form>";
  }

  html += "</div>";
  html += htmlFooter();
  webServer.send(200, "text/html", html);
}

void handleWiFi() {
  String html = htmlHeader("WiFi");
  html += "<div class=\"card\">";
  html += "<h2>WiFi Configuration</h2>";

  if (!configReceived) {
    html += "<p>Waiting for config...</p>";
  } else {
    html += "<form method=\"POST\" action=\"/api/config/wifi\">";
    html += "<label>WiFi SSID:</label>";
    html += "<input type=\"text\" name=\"ssid\" value=\"" + String(cachedConfig.wifiSSID) + "\">";
    html += "<label>WiFi Password:</label>";
    html += "<input type=\"password\" name=\"pass\" value=\"" + String(cachedConfig.wifiPass) + "\">";
    html += "<button type=\"submit\" class=\"btn\">Save &amp; Reboot</button>";
    html += "</form>";
    html += "<p><small>Note: Changing WiFi settings requires full reboot</small></p>";
  }

  html += "</div>";
  html += htmlFooter();
  webServer.send(200, "text/html", html);
}

// --- Helper to queue SET_PARAM ---
void sendSetParam(uint8_t paramId, const uint8_t *value, uint8_t valueLen) {
  uint8_t buf[64];
  buf[0] = CFG_CMD_SET_PARAM;
  buf[1] = paramId;
  buf[2] = valueLen;
  memcpy(&buf[3], value, valueLen);
  queueConfigCmd(buf, 3 + valueLen);

  // Wait for Teensy to pick up the command via SPI
  delay(200);
}

void sendSetParamStr(uint8_t paramId, const String &val) {
  sendSetParam(paramId, (const uint8_t*)val.c_str(), val.length());
}

void sendSetParamU8(uint8_t paramId, uint8_t val) {
  sendSetParam(paramId, &val, 1);
}

void sendSetParamU16(uint8_t paramId, uint16_t val) {
  uint8_t buf[2] = {(uint8_t)(val >> 8), (uint8_t)(val & 0xFF)};
  sendSetParam(paramId, buf, 2);
}

void sendSetParamU32(uint8_t paramId, uint32_t val) {
  sendSetParam(paramId, (const uint8_t*)&val, 4);
}

void sendSetParamFloat(uint8_t paramId, float val) {
  sendSetParam(paramId, (const uint8_t*)&val, 4);
}

void sendSaveReboot() {
  uint8_t buf[1] = {CFG_CMD_SAVE_REBOOT};
  queueConfigCmd(buf, 1);
}

// --- POST Handlers ---

void handleVoterPost() {
  String host = webServer.arg("host");
  String port = webServer.arg("port");
  String clientPwd = webServer.arg("clientpwd");
  String hostPwd = webServer.arg("hostpwd");
  String txGain = webServer.arg("txgain");

  // Send each parameter
  sendSetParamStr(PARAM_HOSTNAME, host);

  // Try parse as IP too
  IPAddress ip;
  if (ip.fromString(host)) {
    uint32_t ipVal = (uint32_t)ip;
    sendSetParamU32(PARAM_HOST_IP, ipVal);
  }

  sendSetParamU16(PARAM_HOST_PORT, port.toInt());
  sendSetParamStr(PARAM_CLIENT_PWD, clientPwd);
  sendSetParamStr(PARAM_HOST_PWD, hostPwd);
  if (txGain.length() > 0) sendSetParamU8(PARAM_TX_GAIN_PCT, txGain.toInt());

  // Save & reboot
  sendSaveReboot();

  webServer.send(200, "text/html",
    "<html><body style=\"background:#2c3e50;color:#ecf0f1;font-family:Arial;text-align:center;padding-top:100px\">"
    "<h1>Configuration Saved</h1><p>Teensy is rebooting...</p>"
    "<p><small>This page will not auto-refresh. Reconnect after reboot.</small></p>"
    "</body></html>");
}

void handleAudioPost() {
  sendSetParamU8(PARAM_RX_GAIN, webServer.arg("rxgain").toInt());
  sendSetParamU8(PARAM_TX_GAIN_PCT, webServer.arg("txgain").toInt());
  sendSetParamFloat(PARAM_DSP_CALIB, webServer.arg("dspgain").toFloat());
  sendSetParamU8(PARAM_INPUT_SOURCE, webServer.arg("inputsource").toInt());
  sendSetParamU8(PARAM_COS_MODE, webServer.arg("cosmode").toInt());
  sendSetParamU8(PARAM_COS_INVERT, webServer.hasArg("cosinvert") ? 1 : 0);
  sendSetParamU8(PARAM_DSP_SQUELCH, webServer.arg("squelch").toInt());
  sendSetParamU8(PARAM_PTT_INVERT, webServer.hasArg("pttinvert") ? 1 : 0);
  sendSetParamU16(PARAM_PTT_TAIL_MS, webServer.arg("ptttail").toInt());
  sendSetParamU8(PARAM_USE_HW_RSSI, webServer.hasArg("hwrssi") ? 1 : 0);
  sendSetParamU16(PARAM_RSSI_MIN, webServer.arg("rssimin").toInt());
  sendSetParamU16(PARAM_RSSI_MAX, webServer.arg("rssimax").toInt());
  sendSetParamU8(PARAM_PL_FILTER, webServer.hasArg("plfilter") ? 1 : 0);
  sendSetParamU8(PARAM_DEEMP, webServer.hasArg("deemph") ? 1 : 0);

  sendSaveReboot();

  webServer.send(200, "text/html",
    "<html><body style=\"background:#2c3e50;color:#ecf0f1;font-family:Arial;text-align:center;padding-top:100px\">"
    "<h1>Configuration Saved</h1><p>Teensy is rebooting...</p></body></html>");
}

void handleWiFiPost() {
  String ssid = webServer.arg("ssid");
  String pass = webServer.arg("pass");

  sendSetParamStr(PARAM_WIFI_SSID, ssid);
  sendSetParamStr(PARAM_WIFI_PASS, pass);

  sendSaveReboot();

  webServer.send(200, "text/html",
    "<html><body style=\"background:#2c3e50;color:#ecf0f1;font-family:Arial;text-align:center;padding-top:100px\">"
    "<h1>WiFi Settings Saved</h1><p>Teensy is rebooting with new WiFi credentials...</p></body></html>");
}

// ======================= SETUP =======================

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("Spirit Firmware 2.0 (Web Enabled)");

  memset(sendbuf, 0, MAX_SPI_BUF);
  memset(recvbuf, 0, MAX_SPI_BUF);
  memset(&cachedConfig, 0, sizeof(cachedConfig));

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  pinMode(GPIO_READY, OUTPUT);
  digitalWrite(GPIO_READY, LOW);

  // Setup SPI Bus
  spi_bus_config_t buscfg = {
      .mosi_io_num = GPIO_MOSI,
      .miso_io_num = GPIO_MISO,
      .sclk_io_num = GPIO_SCLK,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
      .max_transfer_sz = MAX_SPI_BUF,
  };

  // Setup SPI Slave Interface
  spi_slave_interface_config_t slvcfg = {
      .spics_io_num = GPIO_CS,
      .flags = 0,
      .queue_size = 3,
      .mode = 0,
      .post_setup_cb = spi_post_setup_cb,
      .post_trans_cb = NULL};

  esp_err_t ret = spi_slave_initialize(RCV_HOST, &buscfg, &slvcfg, DMA_CHAN);
  if (ret != ESP_OK) {
    Serial.print("SPI Init Failed: ");
    Serial.println(ret);
  } else {
    Serial.println("SPI Slave Started (512 byte buffer)");
  }
}

// ======================= LOOP =======================

void loop() {
  static uint32_t lastHeartbeat = 0;
  static bool webServerStarted = false;

  // Heartbeat
  if (millis() - lastHeartbeat > 2000) {
    lastHeartbeat = millis();
    IPAddress myIP = WiFi.localIP();
    Serial.printf("HB (IP:%u.%u.%u.%u) Cfg:%d Web:%d CBs:%u\r\n",
                  myIP[0], myIP[1], myIP[2], myIP[3],
                  configReceived, webServerStarted, cbCount);
  }

  // Track WiFi Connection
  static bool wasConnected = false;
  if (WiFi.status() == WL_CONNECTED && !wasConnected) {
    wasConnected = true;
    Serial.println("WiFi Connected! Binding UDP to port 8888...");
    udp.begin(8888);
  } else if (WiFi.status() != WL_CONNECTED && wasConnected) {
    wasConnected = false;
    Serial.println("WiFi Disconnected!");
    udp.stop();
    webServerStarted = false;
  }

  // Request config from Teensy if we don't have it yet and WiFi is up
  static uint32_t lastCfgRequest = 0;
  if (wasConnected && !configReceived && (millis() - lastCfgRequest > 10000)) {
    lastCfgRequest = millis();
    Serial.println("Requesting fresh config from Teensy...");
    uint8_t req = CFG_CMD_REQUEST_CONFIG;
    queueConfigCmd(&req, 1);
  }

  // Start web server once we have both WiFi and config
  if (wasConnected && configReceived && !webServerStarted) {
    webServer.on("/", HTTP_GET, handleRoot);
    webServer.on("/voter", HTTP_GET, handleVoter);
    webServer.on("/audio", HTTP_GET, handleAudio);
    webServer.on("/wifi", HTTP_GET, handleWiFi);
    webServer.on("/api/config/voter", HTTP_POST, handleVoterPost);
    webServer.on("/api/config/audio", HTTP_POST, handleAudioPost);
    webServer.on("/api/config/wifi", HTTP_POST, handleWiFiPost);
    webServer.begin();
    webServerStarted = true;
    Serial.println("========================================");
    Serial.println("Web server started on port 80!");
    Serial.print("Access at: http://");
    Serial.println(WiFi.localIP());
    Serial.println("========================================");
  }

  // Handle HTTP requests (non-blocking)
  if (webServerStarted) {
    webServer.handleClient();
  }

  // --- UDP RX Polling ---
  if (!dataPending && !cfgCmdPending && wasConnected) {
    int pktSize = udp.parsePacket();
    if (pktSize > 0) {
      memset(sendbuf, 0, MAX_SPI_BUF);
      sendbuf[0] = STATUS_HAS_DATA;
      sendbuf[1] = (pktSize >> 8) & 0xFF;
      sendbuf[2] = (pktSize & 0xFF);

      int readSize = pktSize;
      if (readSize > MAX_SPI_BUF - 4) {
        readSize = MAX_SPI_BUF - 4;
      }

      udp.read((uint8_t*)&sendbuf[3], readSize);
      dataPending = true;
      digitalWrite(GPIO_READY, HIGH);
    }
  }

  // --- SPI Transaction ---
  memset(recvbuf, 0, MAX_SPI_BUF);
  t.length = MAX_SPI_BUF * 8;
  t.tx_buffer = sendbuf;
  t.rx_buffer = recvbuf;

  esp_err_t ret = spi_slave_transmit(RCV_HOST, &t, pdMS_TO_TICKS(50));

  if (ret == ESP_OK) {
    // Transaction Completed
    digitalWrite(GPIO_READY, LOW);
    dataPending = false;
    cfgCmdPending = false;

    size_t bytesTransferred = t.trans_len / 8;

    if (bytesTransferred > 0) {
      uint8_t cmd = (uint8_t)recvbuf[0];

      // Debug config/system commands
      if (cmd != 0x00 && cmd != 0x10) {
          Serial.printf("SPI RX: cmd=0x%02X, len=%d\r\n", cmd, bytesTransferred);
      }

      if (cmd == CMD_SEND_UDP) {
        if (bytesTransferred >= 9) {
          uint16_t payloadLen =
              ((uint8_t)recvbuf[1] << 8) | (uint8_t)recvbuf[2];
          if (bytesTransferred >= (3 + payloadLen)) { // Relaxed length check
            IPAddress ip((uint8_t)recvbuf[3], (uint8_t)recvbuf[4],
                         (uint8_t)recvbuf[5], (uint8_t)recvbuf[6]);
            uint16_t port = ((uint8_t)recvbuf[7] << 8) | (uint8_t)recvbuf[8];
            udp.beginPacket(ip, port);
            udp.write((uint8_t *)&recvbuf[9], payloadLen);
            udp.endPacket();
          }
        }
      } else if (cmd == CMD_SET_CONFIG) {
        Serial.println("Got Config Command!");
        uint8_t ssidLen = (uint8_t)recvbuf[1];
        if (ssidLen > 0 && ssidLen < 32) {
          char ssid[33] = {0};
          memcpy(ssid, &recvbuf[2], ssidLen);
          uint8_t passOffset = 2 + ssidLen;
          uint8_t passLen = (uint8_t)recvbuf[passOffset];
          char pass[64] = {0};
          if (passLen > 0 && passLen < 64)
            memcpy(pass, &recvbuf[passOffset + 1], passLen);

          Serial.printf("Setting WiFi: %s\r\n", ssid);
          WiFi.disconnect();
          delay(100);
          WiFi.begin(ssid, pass);
        }
      } else if (cmd == CMD_GET_IP) {
        memset(sendbuf, 0, MAX_SPI_BUF);
        sendbuf[0] = STATUS_HAS_DATA;
        sendbuf[1] = 0x00;
        sendbuf[2] = 0x04;
        IPAddress myIP = WiFi.localIP();
        sendbuf[3] = myIP[0];
        sendbuf[4] = myIP[1];
        sendbuf[5] = myIP[2];
        sendbuf[6] = myIP[3];
        dataPending = true;
      } else if (cmd == CMD_PUSH_CONFIG) {
        // Teensy is sending us its full SysConfig struct
        uint16_t cfgLen = ((uint8_t)recvbuf[1] << 8) | (uint8_t)recvbuf[2];
        Serial.printf("Handling CMD_PUSH_CONFIG. cfgLen=%d, rxLen=%d\r\n", cfgLen, bytesTransferred);
        
        if (cfgLen <= sizeof(SysConfigMirror)) {
          memcpy(&cachedConfig, &recvbuf[3], cfgLen);
          configReceived = true;
          Serial.printf("Config received! Host:%s Port:%d\r\n",
                        cachedConfig.hostname[0] ? cachedConfig.hostname : ipToStr(cachedConfig.hostIP).c_str(),
                        cachedConfig.hostPort);
        } else {
          Serial.printf("Config size mismatch: got %d, max %d\r\n", cfgLen, sizeof(SysConfigMirror));
        }
      } else if (cmd == CMD_DNS_LOOKUP) {
        uint8_t hostnameLen = (uint8_t)recvbuf[1];
        if (hostnameLen > 0 && hostnameLen < 64) {
          char hostname[64] = {0};
          memcpy(hostname, &recvbuf[2], hostnameLen);
          IPAddress resolvedIP;
          bool success = WiFi.hostByName(hostname, resolvedIP);

          memset(sendbuf, 0, MAX_SPI_BUF);
          sendbuf[0] = STATUS_HAS_DATA;
          sendbuf[1] = 0x00;
          sendbuf[2] = 0x04;
          if (success && resolvedIP != IPAddress(0, 0, 0, 0)) {
            sendbuf[3] = resolvedIP[0];
            sendbuf[4] = resolvedIP[1];
            sendbuf[5] = resolvedIP[2];
            sendbuf[6] = resolvedIP[3];
          }
          dataPending = true;
        }
      } else if (cmd == CMD_GET_DNS) {
        memset(sendbuf, 0, MAX_SPI_BUF);
        sendbuf[0] = STATUS_HAS_DATA;
        sendbuf[1] = 0x00;
        sendbuf[2] = 0x04;
        IPAddress dnsIP = WiFi.dnsIP();
        sendbuf[3] = dnsIP[0];
        sendbuf[4] = dnsIP[1];
        sendbuf[5] = dnsIP[2];
        sendbuf[6] = dnsIP[3];
        dataPending = true;
      }
    }
  }
}