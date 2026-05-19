#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include "driver/spi_slave.h"
#include "SpiProtocol.h"

// Pin Definitions for ESP32 SPI Slave (per wiring_pin_table.csv)
#define GPIO_MOSI 23
#define GPIO_MISO 19
#define GPIO_SCLK 18
#define GPIO_CS   5
#define GPIO_READY 22  // Signal to Teensy that we have data

#define RCV_HOST    VSPI_HOST
#define DMA_CHAN    1

#pragma pack(push, 1)
struct SysConfigMirror {
  uint32_t magic;
  uint32_t version;
  uint32_t hostIP;
  uint32_t staticIP;
  uint32_t subnetMask;
  uint32_t gateway;
  uint32_t staticDNS;
  uint32_t dnsServerIP;
  float dspCalib;
  uint32_t syslogIP;

  uint16_t hostPort;
  uint16_t rssiMin;
  uint16_t rssiMax;
  int16_t timingOffsetMs;
  uint16_t pttTailMs;
  uint16_t syslogPort;

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
  bool useSyslog;

  char hostname[64];
  char syslogHostname[64];
  char clientPwd[20];
  char hostPwd[20];
  char wifiSSID[32];
  char wifiPass[64];
};
#pragma pack(pop)

// --- Globals ---
WiFiUDP udp;
WebServer webServer(80);

WORD_ALIGNED_ATTR char sendbuf[MAX_SPI_BUF];
WORD_ALIGNED_ATTR char recvbuf[MAX_SPI_BUF];
spi_slave_transaction_t t;

volatile bool dataPending = false;
volatile uint32_t cbCount = 0;
SysConfigMirror cachedConfig;
bool configReceived = false;
volatile bool cfgCmdPending = false;
uint8_t cfgCmdBuf[64];
uint8_t cfgCmdLen = 0;

const char* HTML_STYLE = R"rawliteral(
<style>
body{font-family:Arial,sans-serif;margin:0;padding:20px;background:#2c3e50;color:#ecf0f1}
.container{max-width:800px;margin:0 auto}
.card{background:#34495e;padding:20px;margin:10px 0;border-radius:8px;box-shadow:0 4px 6px rgba(0,0,0,0.3)}
h1{color:#3498db;margin-top:0}
h2{color:#ecf0f1;border-bottom:2px solid #3498db;padding-bottom:10px}
.btn{background:#3498db;color:#fff;border:none;padding:10px 20px;border-radius:4px;cursor:pointer;font-size:16px;margin:5px}
.btn:hover{background:#2980b9}
label{display:block;margin:15px 0 5px 0;font-weight:bold;color:#bdc3c7}
input[type="text"],input[type="password"],input[type="number"],select{width:100%;padding:10px;margin:5px 0;border-radius:4px;border:1px solid #7f8c8d;background:#ecf0f1;color:#2c3e50}
.nav{margin-bottom:20px}
.nav a{color:#3498db;text-decoration:none;margin-right:15px;font-weight:bold}
.nav a:hover{color:#ecf0f1}
.status{display:inline-block;padding:5px 10px;border-radius:4px}
.status.ok{background:#27ae60;color:#fff}
.status.warn{background:#f39c12;color:#fff}
</style>
)rawliteral";

void IRAM_ATTR spi_post_setup_cb(spi_slave_transaction_t *trans) {
  cbCount++;
  // Raise READY if we have data for Teensy
  if (dataPending || cfgCmdPending) {
    GPIO.out_w1ts = (1 << GPIO_READY);
  }
}

void queueConfigCmd(uint8_t *data, uint8_t len) {
  if (len > 60) return;
  memset(sendbuf, 0, MAX_SPI_BUF);
  sendbuf[0] = STATUS_CONFIG_CMD;
  sendbuf[1] = 0;
  sendbuf[2] = len;
  memcpy(&sendbuf[3], data, len);
  cfgCmdPending = true;
  GPIO.out_w1ts = (1 << GPIO_READY);
}

void sendSetParam(uint8_t paramId, const uint8_t *value, uint8_t valueLen) {
  uint8_t buf[64];
  buf[0] = CFG_CMD_SET_PARAM;
  buf[1] = paramId;
  buf[2] = valueLen;
  memcpy(&buf[3], value, valueLen);
  queueConfigCmd(buf, 3 + valueLen);
}

void sendSetParamStr(uint8_t paramId, String val) {
  sendSetParam(paramId, (const uint8_t*)val.c_str(), val.length());
}

void sendSetParamU16(uint8_t paramId, uint16_t val) {
  uint8_t b[2] = {(uint8_t)(val >> 8), (uint8_t)(val & 0xFF)};
  sendSetParam(paramId, b, 2);
}

void sendSaveReboot() {
  uint8_t b = CFG_CMD_SAVE_REBOOT;
  queueConfigCmd(&b, 1);
}

String htmlHeader(const char* title) {
  String html = "<!DOCTYPE html><html><head><meta charset=\"UTF-8\">";
  html += "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">";
  html += "<title>Voter - " + String(title) + "</title>";
  html += HTML_STYLE;
  html += "</head><body><div class=\"container\">";
  html += "<h1>TeensyVoter</h1>";
  html += "<div class=\"nav\"><a href=\"/\">Status</a><a href=\"/voter\">Voter</a><a href=\"/wifi\">WiFi</a></div>";
  return html;
}

void handleRoot() {
  String html = htmlHeader("Status");
  html += "<div class=\"card\"><h2>System Status</h2>";
  if (!configReceived) {
    html += "<p><span class=\"status warn\">Waiting for Teensy...</span></p>";
  } else {
    html += "<p><strong>IP Address:</strong> " + WiFi.localIP().toString() + "</p>";
    html += "<p><strong>WiFi SSID:</strong> " + String(cachedConfig.wifiSSID) + "</p>";
    html += "<p><strong>RSSI:</strong> " + String(WiFi.RSSI()) + " dBm</p>";
    html += "<p><strong>Voter Host:</strong> " + String(cachedConfig.hostname) + "</p>";
  }
  html += "</div>" + String("</div></body></html>");
  webServer.send(200, "text/html", html);
}

void handleVoter() {
  String html = htmlHeader("Voter");
  html += "<div class=\"card\"><h2>Voter Settings</h2><form method=\"POST\" action=\"/voter\">";
  html += "<label>Host Address:</label><input type=\"text\" name=\"host\" value=\"" + String(cachedConfig.hostname) + "\">";
  html += "<label>Host Port:</label><input type=\"number\" name=\"port\" value=\"" + String(cachedConfig.hostPort) + "\">";
  html += "<input type=\"submit\" value=\"Save\" class=\"btn\"></form></div>";
  html += "</div></body></html>";
  webServer.send(200, "text/html", html);
}

void handleVoterPost() {
  sendSetParamStr(PARAM_HOSTNAME, webServer.arg("host"));
  sendSetParamU16(PARAM_HOST_PORT, webServer.arg("port").toInt());
  sendSaveReboot();
  webServer.send(200, "text/html", "<html><body><h1>Saved</h1><p>Rebooting Teensy...</p><a href=\"/\">Back</a></body></html>");
}

void handleWiFi() {
  String html = htmlHeader("WiFi");
  html += "<div class=\"card\"><h2>WiFi Settings</h2><form method=\"POST\" action=\"/wifi\">";
  html += "<label>SSID:</label><input type=\"text\" name=\"ssid\" value=\"" + String(cachedConfig.wifiSSID) + "\">";
  html += "<label>Password:</label><input type=\"password\" name=\"pass\">";
  html += "<input type=\"submit\" value=\"Save\" class=\"btn\"></form></div>";
  html += "</div></body></html>";
  webServer.send(200, "text/html", html);
}

void handleWiFiPost() {
  sendSetParamStr(PARAM_WIFI_SSID, webServer.arg("ssid"));
  sendSetParamStr(PARAM_WIFI_PASS, webServer.arg("pass"));
  sendSaveReboot();
  webServer.send(200, "text/html", "<html><body><h1>Saved</h1><p>Rebooting Teensy...</p><a href=\"/\">Back</a></body></html>");
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Spirit Firmware 3.4 (Sync Enabled)");

  pinMode(GPIO_READY, OUTPUT);
  digitalWrite(GPIO_READY, LOW);

  spi_bus_config_t buscfg = {
      .mosi_io_num = GPIO_MOSI,
      .miso_io_num = GPIO_MISO,
      .sclk_io_num = GPIO_SCLK,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
      .max_transfer_sz = MAX_SPI_BUF,
  };

  spi_slave_interface_config_t slvcfg = {
      .spics_io_num = GPIO_CS,
      .flags = 0,
      .queue_size = 3,
      .mode = 0,
      .post_setup_cb = spi_post_setup_cb,
  };

  spi_slave_initialize(RCV_HOST, &buscfg, &slvcfg, DMA_CHAN);

  WiFi.mode(WIFI_STA);
  Serial.println("SPI Slave Started");
}

void loop() {
  static uint32_t lastHb = 0;
  if (millis() - lastHb > 5000) {
    lastHb = millis();
    Serial.printf("HB IP:%s Cfg:%d CBs:%u\r\n", WiFi.localIP().toString().c_str(), configReceived, cbCount);
    
    if (WiFi.status() == WL_CONNECTED && !configReceived) {
      uint8_t req = CFG_CMD_REQUEST_CONFIG;
      queueConfigCmd(&req, 1);
    }
  }

  static bool wasConnected = false;
  if (WiFi.status() == WL_CONNECTED && !wasConnected) {
    wasConnected = true;
    udp.begin(8888);
    webServer.on("/", handleRoot);
    webServer.on("/voter", HTTP_GET, handleVoter);
    webServer.on("/voter", HTTP_POST, handleVoterPost);
    webServer.on("/wifi", HTTP_GET, handleWiFi);
    webServer.on("/wifi", HTTP_POST, handleWiFiPost);
    webServer.begin();
    Serial.println("WiFi Connected, Web Server Started");
  }

  if (wasConnected) webServer.handleClient();

  // SPI Transaction
  if (!dataPending && !cfgCmdPending && wasConnected) {
    int pktSize = udp.parsePacket();
    if (pktSize > 0) {
      memset(sendbuf, 0, MAX_SPI_BUF);
      sendbuf[0] = STATUS_HAS_DATA;
      sendbuf[1] = (pktSize >> 8) & 0xFF;
      sendbuf[2] = (pktSize & 0xFF);
      udp.read((uint8_t*)&sendbuf[3], (pktSize > MAX_SPI_BUF-3) ? MAX_SPI_BUF-3 : pktSize);
      dataPending = true;
      GPIO.out_w1ts = (1 << GPIO_READY); // READY HIGH
    }
  }

  memset(recvbuf, 0, MAX_SPI_BUF);
  t.length = MAX_SPI_BUF * 8;
  t.tx_buffer = sendbuf;
  t.rx_buffer = recvbuf;
  esp_err_t ret = spi_slave_transmit(RCV_HOST, &t, pdMS_TO_TICKS(50));
  
  if (ret == ESP_OK) {
    // Clear READY pin immediately after transaction finishes
    GPIO.out_w1tc = (1 << GPIO_READY);
    dataPending = false;
    cfgCmdPending = false;

    size_t bytesTransferred = t.trans_len / 8;
      if (bytesTransferred > 0) {
        uint8_t cmd = (uint8_t)recvbuf[0];
        
        // Remove verbose [SPI] CMD logs as they cause timing jitter

        if (cmd == CMD_SEND_UDP) {
          if (bytesTransferred >= 9) {
            uint16_t payloadLen = ((uint8_t)recvbuf[1] << 8) | (uint8_t)recvbuf[2];
            IPAddress ip((uint8_t)recvbuf[3], (uint8_t)recvbuf[4], (uint8_t)recvbuf[5], (uint8_t)recvbuf[6]);
            uint16_t port = ((uint8_t)recvbuf[7] << 8) | (uint8_t)recvbuf[8];
            
            // Debug non-audio packets (Syslog, etc)
            if (port != 1667) {
              Serial.printf("[UDP] Sending Admin Packet to %s:%d (Len: %d)\r\n", 
                            ip.toString().c_str(), port, payloadLen);
            }

            udp.beginPacket(ip, port);
            udp.write((uint8_t *)&recvbuf[9], payloadLen);
            udp.endPacket();
          }
        } else if (cmd == CMD_SET_CONFIG) {
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
          GPIO.out_w1ts = (1 << GPIO_READY);
          Serial.printf("[SPI] Reporting Local IP: %s\r\n", myIP.toString().c_str());
        } else if (cmd == CMD_PUSH_CONFIG) {
          uint16_t cfgLen = ((uint8_t)recvbuf[1] << 8) | (uint8_t)recvbuf[2];
          if (cfgLen <= sizeof(SysConfigMirror)) {
            memcpy(&cachedConfig, &recvbuf[3], cfgLen);
            configReceived = true;
            Serial.printf("Config received! Host:%s\r\n", cachedConfig.hostname);
          }
        } else if (cmd == CMD_DNS_LOOKUP) {
          uint8_t hostnameLen = (uint8_t)recvbuf[1];
          if (hostnameLen > 0 && hostnameLen < 64) {
            char hostname[64] = {0};
            memcpy(hostname, &recvbuf[2], hostnameLen);
            Serial.printf("[DNS] Request: %s\r\n", hostname);
            IPAddress resolvedIP;
            bool success = WiFi.hostByName(hostname, resolvedIP);

            memset(sendbuf, 0, MAX_SPI_BUF);
            sendbuf[0] = STATUS_HAS_DATA;
            sendbuf[1] = 0x00;
            sendbuf[2] = 0x04;
            if (success && resolvedIP != IPAddress(0,0,0,0)) {
              sendbuf[3] = resolvedIP[0];
              sendbuf[4] = resolvedIP[1];
              sendbuf[5] = resolvedIP[2];
              sendbuf[6] = resolvedIP[3];
              Serial.printf("[DNS] Resolved %s to %s\r\n", hostname, resolvedIP.toString().c_str());
            } else {
              sendbuf[3] = 0; sendbuf[4] = 0; sendbuf[5] = 0; sendbuf[6] = 0;
              Serial.printf("[DNS] Failed to resolve %s\r\n", hostname);
            }
            dataPending = true;
            GPIO.out_w1ts = (1 << GPIO_READY);
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
          GPIO.out_w1ts = (1 << GPIO_READY);
          Serial.printf("[SPI] Reporting DNS: %s\r\n", dnsIP.toString().c_str());
        }
      }
  }
}