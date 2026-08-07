// Minimal ETH-only isolation test. No WiFi, no SPI slave, no web server, no
// failover logic - just ETH.begin() and link-event logging, to check
// whether the ~2000ms CONNECTED/DISCONNECTED flap seen in Spirit.cpp is a
// property of the hardware/PHY itself or something in the rest of that
// firmware's init/runtime. Same pin config as Spirit.cpp's ETH section.

#include <ETH.h>
#include <WiFi.h> // needed for the ARDUINO_EVENT_* / WiFi.onEvent() plumbing

#define ETH_PHY_ADDR     1
#define ETH_PHY_POWER    16
#define ETH_PHY_MDC      23
#define ETH_PHY_MDIO     18
#define ETH_PHY_TYPE     ETH_PHY_LAN8720
#define ETH_PHY_CLK_MODE ETH_CLOCK_GPIO0_IN

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("EthTest - minimal ETH-only isolation sketch");

  WiFi.onEvent([](arduino_event_id_t event, arduino_event_info_t info) {
    switch (event) {
      case ARDUINO_EVENT_ETH_START:        Serial.printf("[ETH-EVT] %lu START\r\n", millis()); break;
      case ARDUINO_EVENT_ETH_CONNECTED:    Serial.printf("[ETH-EVT] %lu CONNECTED (link up)\r\n", millis()); break;
      case ARDUINO_EVENT_ETH_DISCONNECTED: Serial.printf("[ETH-EVT] %lu DISCONNECTED (link down)\r\n", millis()); break;
      case ARDUINO_EVENT_ETH_GOT_IP:       Serial.printf("[ETH-EVT] %lu GOT_IP: %s\r\n", millis(), ETH.localIP().toString().c_str()); break;
      case ARDUINO_EVENT_ETH_LOST_IP:      Serial.printf("[ETH-EVT] %lu LOST_IP\r\n", millis()); break;
      case ARDUINO_EVENT_ETH_STOP:         Serial.printf("[ETH-EVT] %lu STOP\r\n", millis()); break;
      default: break;
    }
  });

  // Force fixed 100/Full to match the switch port instead of relying on
  // autonegotiation - must be called before begin(), these just set flags
  // that begin() applies internally.
  ETH.setAutoNegotiation(false);
  ETH.setFullDuplex(true);
  ETH.setLinkSpeed(100);

  ETH.begin(ETH_PHY_TYPE, ETH_PHY_ADDR, ETH_PHY_MDC, ETH_PHY_MDIO, ETH_PHY_POWER, ETH_PHY_CLK_MODE);
  Serial.println("ETH.begin() called (forced 100/Full, autoneg off)");
}

void loop() {
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint > 2000) {
    lastPrint = millis();
    Serial.printf("[POLL] %lu linkUp=%d speed=%d full_duplex=%d IP=%s\r\n",
                  millis(), ETH.linkUp(), ETH.linkSpeed(), ETH.fullDuplex(),
                  ETH.localIP().toString().c_str());
  }
}
