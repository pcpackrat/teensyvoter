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
WORD_ALIGNED_ATTR char sendbuf[MAX_SPI_BUF] = "";
WORD_ALIGNED_ATTR char recvbuf[MAX_SPI_BUF] = "";
spi_slave_transaction_t t;

void setup() {
  Serial.begin(115200);
  delay(2000);

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
  spi_slave_interface_config_t slvcfg = {.spics_io_num = GPIO_CS,
                                         .flags = 0,
                                         .queue_size = 3,
                                         .mode = 0,
                                         .post_setup_cb = NULL,
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
  memset(recvbuf, 0, MAX_SPI_BUF);
  t.length = MAX_SPI_BUF * 8;
  t.tx_buffer = sendbuf;
  t.rx_buffer = recvbuf;

  // Timeout 100ms
  esp_err_t ret = spi_slave_transmit(RCV_HOST, &t, pdMS_TO_TICKS(100));

  if (ret == ESP_OK) {
    size_t bytesTransferred = t.trans_len / 8;

    // DEBUG: Print Any Transfer
    if (bytesTransferred > 0) {
      // Serial.print("RX: "); Serial.print(bytesTransferred); Serial.print(" B.
      // Cmd: 0x"); Serial.println((uint8_t)recvbuf[0], HEX);

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
        // Try to parse SSID
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

          // FORCE HARDCODED CREDENTIALS FOR TEST
          // To verify if connection works at all.
          // Serial.println("Using HARDCODED Credentials for Test...");
          // WiFi.disconnect();
          // delay(100);
          // WiFi.begin("ImWatchinYou", "n0Password");
          WiFi.begin(ssid, pass); // Restore Original Logic
          Serial.println("Connecting WiFi...");
        }
      } else if (cmd == 0x03) { // CMD_GET_IP
        Serial.println("Got Get-IP Command!");
        sendbuf[0] = 0x01;
        sendbuf[1] = 0x00;
        sendbuf[2] = 0x04;
        IPAddress myIP = WiFi.localIP();
        sendbuf[3] = myIP[0];
        sendbuf[4] = myIP[1];
        sendbuf[5] = myIP[2];
        sendbuf[6] = myIP[3];
        digitalWrite(GPIO_READY, HIGH);
      }
    }
  }

  // --- UDP RECEIVE LOGIC ---
  // Only if we aren't already waiting to send something
  if (sendbuf[0] == 0x00) {
    int packetSize = udp.parsePacket();
    if (packetSize > 0) {
      // Protocol:
      // Byte 0: STATUS_HAS_DATA (0x01)
      // Byte 1: High Byte Length
      // Byte 2: Low Byte Length
      // Byte 3..N: Payload

      // Cap size to fit in buffer (Max 4096 - 4 header bytes)
      if (packetSize > 4000)
        packetSize = 4000;

      sendbuf[0] = 0x01; // STATUS_HAS_DATA
      sendbuf[1] = (packetSize >> 8) & 0xFF;
      sendbuf[2] = (packetSize & 0xFF);

      udp.read((char *)&sendbuf[3], packetSize);

      // Debug
      // Serial.print("UDP RX: "); Serial.print(packetSize); Serial.println("
      // bytes. Queued for SPI.");

      digitalWrite(GPIO_READY, HIGH);
    }
  }

  // Heartbeat
  static uint32_t lastHb = 0;
  if (millis() - lastHb > 2000) {
    lastHb = millis();
    Serial.print("Heartbeat (IP: ");
    Serial.print(WiFi.localIP());
    Serial.println(")");
  }

  // Reset Send Buffer if used
  if (ret == ESP_OK && sendbuf[0] != 0x00) {
    digitalWrite(GPIO_READY, LOW);
    sendbuf[0] = 0x00;
  }
}