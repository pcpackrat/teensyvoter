#include "driver/spi_slave.h"
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// --- Configuration ---
// VSPI Pins
#define GPIO_MOSI 23
#define GPIO_MISO 19
#define GPIO_SCLK 18
#define GPIO_CS 5
#define GPIO_READY 22

#define RCV_HOST VSPI_HOST
#define DMA_CHAN 2

#define MAX_SPI_BUF 256

// --- Globals ---
WiFiUDP udp;

// Static DMA Buffers (Aligned)
WORD_ALIGNED_ATTR char sendbuf[MAX_SPI_BUF];
WORD_ALIGNED_ATTR char recvbuf[MAX_SPI_BUF];
spi_slave_transaction_t t;

volatile bool dataPending = false;
volatile uint32_t cbCount = 0; // Diagnostic

// Callback runs AFTER SPI is armed but BEFORE transfer starts
void IRAM_ATTR spi_post_setup_cb(spi_slave_transaction_t *trans) {
  cbCount++;
  if (dataPending) {
    digitalWrite(GPIO_READY, HIGH);
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("Diagnostic Firmware 1.0");

  memset(sendbuf, 0, MAX_SPI_BUF);
  memset(recvbuf, 0, MAX_SPI_BUF);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  pinMode(GPIO_READY, OUTPUT);
  digitalWrite(GPIO_READY, LOW);

  // 2. Setup SPI Bus
  spi_bus_config_t buscfg = {
      .mosi_io_num = GPIO_MOSI,
      .miso_io_num = GPIO_MISO,
      .sclk_io_num = GPIO_SCLK,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
      .max_transfer_sz = MAX_SPI_BUF,
  };

  // 3. Setup SPI Slave Interface
  spi_slave_interface_config_t slvcfg = {
      .spics_io_num = GPIO_CS,
      .flags = 0,
      .queue_size = 3,
      .mode = 0,
      .post_setup_cb = spi_post_setup_cb, // Call us when armed
      .post_trans_cb = NULL};

  // 4. Initialize
  esp_err_t ret = spi_slave_initialize(RCV_HOST, &buscfg, &slvcfg, DMA_CHAN);
  if (ret != ESP_OK) {
    Serial.print("SPI Init Failed: ");
    Serial.println(ret);
  } else {
    Serial.println("ESP32 SPI Debug Mode Started");
  }
}

void loop() {
  static uint32_t lastHeartbeat = 0;

  // Heartbeat AND TX Debug
  if (millis() - lastHeartbeat > 2000) {
    lastHeartbeat = millis();
    IPAddress myIP = WiFi.localIP();
    int rdyState = digitalRead(GPIO_READY);
    Serial.printf("Heartbeat (IP: %u.%u.%u.%u) ReadyPin: %d, DataPending: %d, "
                  "CBs: %u\r\n",
                  myIP[0], myIP[1], myIP[2], myIP[3], rdyState, dataPending,
                  cbCount);
  }

  // Debug Print of TX Buffer if Reply Pending
  if (dataPending) {
    Serial.printf("[TX PREP] IP: %d.%d.%d.%d\r\n", (uint8_t)sendbuf[3],
                  (uint8_t)sendbuf[4], (uint8_t)sendbuf[5],
                  (uint8_t)sendbuf[6]);
  }

  // Setup Buffer
  memset(recvbuf, 0, MAX_SPI_BUF);
  t.length = MAX_SPI_BUF * 8;
  t.tx_buffer = sendbuf;
  t.rx_buffer = recvbuf;

  // Wait for Transaction (Timeout 100ms)
  esp_err_t ret = spi_slave_transmit(RCV_HOST, &t, pdMS_TO_TICKS(100));

  if (ret == ESP_OK) {
    // Transaction Completed!

    // 1. Clear Ready ASAP
    digitalWrite(GPIO_READY, LOW);
    dataPending = false;

    size_t bytesTransferred = t.trans_len / 8;

    if (bytesTransferred > 0) {
      uint8_t cmd = (uint8_t)recvbuf[0];

      if (cmd == 0x10) { // CMD_SEND_UDP
        if (bytesTransferred >= 9) {
          uint16_t payloadLen =
              ((uint8_t)recvbuf[1] << 8) | (uint8_t)recvbuf[2];
          if (bytesTransferred >= (9 + payloadLen)) {
            IPAddress ip((uint8_t)recvbuf[3], (uint8_t)recvbuf[4],
                         (uint8_t)recvbuf[5], (uint8_t)recvbuf[6]);
            uint16_t port = ((uint8_t)recvbuf[7] << 8) | (uint8_t)recvbuf[8];

            udp.beginPacket(ip, port);
            udp.write((uint8_t *)&recvbuf[9], payloadLen);
            udp.endPacket();
          }
        }
      } else if (cmd == 0x02) { // CMD_SET_CONFIG
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

          Serial.print("Setting WiFi: ");
          Serial.println(ssid);

          // Fix "sta is connecting" error
          WiFi.disconnect();
          delay(100);
          WiFi.begin(ssid, pass);

          Serial.print("Parsed Pass: ");
          Serial.println(pass);

          Serial.println("Connecting WiFi...");
        }
      } else if (cmd == 0x03) { // CMD_GET_IP
        Serial.println("Got Get-IP Command!");
        memset(sendbuf, 0, MAX_SPI_BUF); // Clear TX
        sendbuf[0] = 0x01;
        sendbuf[1] = 0x00;
        sendbuf[2] = 0x04;
        IPAddress myIP = WiFi.localIP();
        sendbuf[3] = myIP[0];
        sendbuf[4] = myIP[1];
        sendbuf[5] = myIP[2];
        sendbuf[6] = myIP[3];
        dataPending = true;
      } else if (cmd == 0x04) { // CMD_DNS_LOOKUP
        Serial.println("Got DNS Lookup Command!");
        uint8_t hostnameLen = (uint8_t)recvbuf[1];
        if (hostnameLen > 0 && hostnameLen < 64) {
          char hostname[64] = {0};
          memcpy(hostname, &recvbuf[2], hostnameLen);

          Serial.print("Resolving: ");
          Serial.println(hostname);

          IPAddress resolvedIP;
          bool success = WiFi.hostByName(hostname, resolvedIP);

          memset(sendbuf, 0, MAX_SPI_BUF); // Clear TX
          sendbuf[0] = 0x01;               // STATUS_HAS_DATA
          sendbuf[1] = 0x00;               // Length high byte
          sendbuf[2] = 0x04;               // Length low byte (4 bytes for IP)

          if (success && resolvedIP != IPAddress(0, 0, 0, 0)) {
            sendbuf[3] = resolvedIP[0];
            sendbuf[4] = resolvedIP[1];
            sendbuf[5] = resolvedIP[2];
            sendbuf[6] = resolvedIP[3];
            Serial.print("Resolved to: ");
            Serial.println(resolvedIP);
          } else {
            // Return 0.0.0.0 on failure
            sendbuf[3] = 0;
            sendbuf[4] = 0;
            sendbuf[5] = 0;
            sendbuf[6] = 0;
            Serial.println("DNS resolution failed");
          }
          dataPending = true;
        }
      } else if (cmd == 0x05) { // CMD_GET_DNS
        Serial.println("Got Get-DNS Command!");
        memset(sendbuf, 0, MAX_SPI_BUF);
        sendbuf[0] = 0x01;
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