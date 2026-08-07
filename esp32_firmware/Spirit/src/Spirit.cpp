#include <Arduino.h>
#include <WiFi.h>      // ETH.hostByName() (interface-agnostic DNS); also WiFi failover + AP setup mode
#include <ETH.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <DNSServer.h> // captive-portal redirect while in AP setup mode
#include "driver/spi_slave.h"
#include "soc/gpio_struct.h" // GPIO.out_w1ts/out_w1tc register access
#include "SpiProtocol.h"

// SPI slave pins, reassigned off the fixed/reserved LAN8720 pins (this
// board's actual header breakout - confirmed from silkscreen, NOT the
// generic WT32-ETH01 pinout docs/esp32_network_migration_plan.md assumed:
// this variant does not expose IO32/IO33 at all). Free header pins after
// reserving IO0/TXO/RXO for ETH clock + UART0 flashing: IO12/14/15 (SPI,
// used below), IO35/36/39 (input-only), IO4 (bidirectional), IO2
// (bidirectional but a boot-strap pin - avoid driving it continuously).
#define GPIO_MOSI 14
#define GPIO_MISO 15
#define GPIO_SCLK 12
// CS is an ESP32 input (Teensy drives it as SPI master) - forced onto an
// input-only pin since no other free bidirectional GPIO remains on this
// header. Native hardware CS (spics_io_num) may need IOMUX routing that
// input-only pins don't support - unconfirmed, bench-verify on real
// hardware; IO36/IO39 are free spares to try if IO35 doesn't work.
#define GPIO_CS   35
#define GPIO_READY 4  // Signal to Teensy that we have data (ESP32 output - only free bidirectional pin left on this header)

// LAN8720 PHY wiring fixed by the WT32-ETH01 module itself (not
// reassignable - these are the pins listed as reserved above).
#define ETH_PHY_ADDR   1
#define ETH_PHY_POWER  16
#define ETH_PHY_MDC    23
#define ETH_PHY_MDIO   18
#define ETH_PHY_TYPE   ETH_PHY_LAN8720
#define ETH_PHY_CLK_MODE ETH_CLOCK_GPIO0_IN

// Onboard green LED, active-LOW (per board silkscreen: IO17/"TX2", the
// unused RS485 UART TX pin this project doesn't use for RS485 - free to
// drive directly). Lit solid while the WiFi setup hotspot is running.
#define LED_AP_SETUP 17

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

// NOTE: multi-slot (queue_size-deep, round-robin) SPI was attempted three
// separate times this session, each fixing a real, verified bug found on
// hardware (single-slot re-arm race -> requeue-failure tracking -> global
// dataPending/cfgCmdPending -> READY ISR only checking the active slot),
// and each time still ending up broken in a NEW way. The third attempt's
// failure mode was different again: UDPcmd/UDPfwd sat at flat 0 through
// an entire voter auth handshake attempt (uplink relay not registering
// at all), despite GET_IP/boot working fine. Three distinct failure modes
// from three reasoned fixes is a strong signal this architecture (N
// independently-rotating slots sharing one READY GPIO) is more fragile
// than it looks on paper, and needs a logic analyzer to actually see the
// SPI bus rather than more blind attempts. Back to single-buffer - proven
// solid for both the auth handshake and real bidirectional audio. Its one
// known flaw (the re-arm race, ~15-20% loss under sustained bursts) is
// now precisely measurable via the CMD_SEND_UDP transport sequence
// numbers (see EspSpiDriver.cpp's sendPacketTo() and the [SEQ] Gap log
// below), so any future fix attempt here can be verified directly instead
// of judged indirectly - that tooling stays regardless of this revert.
WORD_ALIGNED_ATTR char sendbuf[MAX_SPI_BUF];
WORD_ALIGNED_ATTR char recvbuf[MAX_SPI_BUF];
spi_slave_transaction_t t;

volatile bool dataPending = false;
volatile uint32_t cbCount = 0;
SysConfigMirror cachedConfig;
bool configReceived = false;
GpsStatus cachedGpsStatus;
bool gpsStatusReceived = false;
volatile bool cfgCmdPending = false;

// Phase 0: staging buffer for config commands. queueConfigCmd() used to
// write straight into sendbuf, which was only safe because the old main
// loop blocked on spi_slave_transmit() so nothing else ever ran while a
// transaction was in flight. Now that transactions are queued/polled
// non-blockingly (webServer.handleClient() etc. can run while one is
// outstanding), sendbuf must only be touched in the single safe window
// right after a transaction completes and before the next is queued -
// so config commands stage here and get copied into sendbuf there.
//
// This is a real queue, not a single slot - it used to be one slot that
// queueConfigCmd() just overwrote on every call, which was fine when
// each web-form save only ever sent one command. Every settings page
// now sends several (one SET_PARAM per field, then SAVE_REBOOT) back to
// back, all before the next SPI transaction ever drains anything - with
// a single slot, only the last call before that drain survived and
// everything before it was silently clobbered. Confirmed on hardware:
// WiFi SSID/password changes were never actually reaching the Teensy
// because SAVE_REBOOT (sent last) kept overwriting them first.
#define CFG_CMD_QUEUE_SIZE 8
uint8_t cfgCmdQueue[CFG_CMD_QUEUE_SIZE][64];
uint8_t cfgCmdQueueLen[CFG_CMD_QUEUE_SIZE];
uint8_t cfgCmdQueueHead = 0; // next slot to send
uint8_t cfgCmdQueueTail = 0; // next free slot to fill
uint8_t cfgCmdQueueCount = 0;

// Phase 0 instrumentation: transaction/forward counters, reported in the
// 5s heartbeat alongside the existing IP/config/callback status.
volatile uint32_t txnCompletedCount = 0;
volatile uint32_t udpForwardedCount = 0;
volatile uint32_t udpSendCmdSeenCount = 0;

// Direct measurement of the SPI slave's unarmed window - the CPU time from
// dequeuing a completed transaction to the requeue call returning (which
// re-arms the slave). This is the actual re-arm race window; loss % is a
// confounded proxy for it (also moves with network conditions/auth churn).
// Measuring this directly lets us tell whether a given change narrows the
// window itself, independent of that noise.
uint32_t maxRearmWindowUs = 0;

// Corruption-vs-absence discriminator - a transaction that arrives with a
// glitched cmd byte or truncated trans_len falls through completely
// silently today (no counter, no log), indistinguishable from a fully
// missed transaction at the SPI transport level. If these climb roughly
// in step with [SEQ] Gap, the loss is corrupted-in-flight data, not the
// slave being unarmed - relevant since two independent re-arm-window
// timing fixes measured zero effect on the gap rate.
uint32_t unknownCmdCount = 0;
uint32_t shortSendCmdCount = 0;

// Payload checksum mismatches - catches corruption WITHIN an otherwise
// valid, sequential transaction (wrong audio samples, right SEQ number),
// which nothing else measures. See the CMD_SEND_UDP handler for how the
// checksum itself is computed and compared.
uint32_t cksumMismatchCount = 0;

// Transport-layer sequence tracking for CMD_SEND_UDP - see the loop()
// handler for why (directly detects dropped Teensy->ESP32 transactions
// instead of only inferring loss indirectly from app-layer symptoms).
uint16_t lastUdpSeq = 0;
bool udpSeqInit = false;
uint32_t udpSeqDropCount = 0;

// Deferred uplink-relay send staging - see the CMD_SEND_UDP handler in
// loop() for why. rb gets memset/reused as the next receive buffer before
// the requeue, so the payload has to be copied out before then if the
// actual udp.write() is going to happen after re-arming instead of before.
WORD_ALIGNED_ATTR uint8_t udpSendStagingBuf[MAX_SPI_BUF];
uint16_t udpSendStagingLen = 0;
IPAddress udpSendStagingIP;
uint16_t udpSendStagingPort = 0;
bool udpSendPending = false;

// ETH is always preferred; WiFi is only started as a fallback, using
// credentials that arrive with the rest of the config (see CMD_PUSH_CONFIG
// below - ETH bring-up itself is deliberately delayed until config
// arrives, since static IP settings have to be known before ETH.begin()
// can apply them).
//
// Two different failover thresholds, not one: ETH.linkUp() reads the
// LAN8720 PHY's hardware link-detect bit directly (autonegotiation
// carrier sense) - it goes false within about a second of a cable
// actually being pulled, independent of DHCP. That's an unambiguous
// signal and should fail over fast. A link that's up but just hasn't
// finished DHCP yet is a different, more likely-transient case and
// deserves more patience before yanking over to WiFi.
enum class NetMode { NONE, ETH_ACTIVE, WIFI_ACTIVE, AP_SETUP };
NetMode netMode = NetMode::NONE;
bool ethStarted = false;
uint32_t lastEthUp = 0;       // millis() this interface last had link+IP; 0 counts as "never"
uint32_t linkDownSince = 0;   // millis() ETH.linkUp() last went false (0 = currently up)
uint32_t phyLinkUpSince = 0;  // millis() ETH.linkUp() became continuously true (0 = currently down)
uint32_t lastWifiAttempt = 0;
bool wifiAttempted = false; // lastWifiAttempt==0 is indistinguishable from "just tried at boot" - track explicitly
const uint32_t ETH_LINK_DOWN_FAILOVER_MS = 1000; // PHY link physically down this long -> try WiFi
const uint32_t ETH_NO_IP_FAILOVER_MS = 15000;    // link up but no IP (DHCP pending) this long -> try WiFi
const uint32_t WIFI_RETRY_MS = 30000;            // don't hammer WiFi.begin()
const uint32_t ETH_LINK_UP_DEBOUNCE_MS = 500;    // PHY link must be back this long before dropping WiFi (just flicker debounce, not a DHCP wait)

// Software self-heal for a flapping PHY link that the reset-pulse fix
// above doesn't fully catch (belt and suspenders - the real fix is giving
// the LAN8720 a properly-timed reset pulse before ETH.begin(), see there).
// This exists so a stuck-flapping unit can recover on its own instead of
// needing someone to physically power-cycle it - the whole point of the
// question that prompted this: trusting a remote firmware update not to
// strand the device.
volatile uint32_t ethFlapCountInWindow = 0;
uint32_t ethFlapWindowStart = 0;
const uint32_t ETH_FLAP_WINDOW_MS = 20000; // count DISCONNECTED events in this window
const uint8_t ETH_FLAP_THRESHOLD = 5;      // this many in the window -> genuinely flapping, not one normal transition
uint32_t ethReinitCount = 0;
const uint32_t ETH_REINIT_MAX = 3; // cap auto-recovery attempts so a truly dead PHY doesn't reinit forever
volatile bool ethReinitRequested = false;

bool ethUp() { return ETH.linkUp() && (uint32_t)ETH.localIP() != 0; }

// Shared by the initial bring-up and the flap-triggered auto-recovery
// (see ethReinitRequested) so both get the same properly-timed PHY reset
// pulse - see the call site comment for why that pulse is needed at all.
void startEth() {
  if (cachedConfig.useStaticIP) {
    ETH.config(IPAddress(cachedConfig.staticIP), IPAddress(cachedConfig.gateway),
               IPAddress(cachedConfig.subnetMask), IPAddress(cachedConfig.staticDNS));
  }
  pinMode(ETH_PHY_POWER, OUTPUT);
  digitalWrite(ETH_PHY_POWER, LOW);
  delay(50);
  digitalWrite(ETH_PHY_POWER, HIGH);
  delay(10);

  // Autonegotiation stays on - forcing fixed 100/Full requires the switch
  // port to be manually forced to match too (a real duplex mismatch
  // otherwise, worse than flapping - confirmed on hardware: with only
  // this side forced and the switch left on auto, the link never came up
  // at all, even across a full power cycle). Not doing that at the switch
  // level right now, by choice - WiFi fallback covers the gap when ETH
  // flaps. If this needs revisiting, the forced-100/Full config is proven
  // to work (120+ seconds, zero flaps, multiple reboots) - it just needs
  // the switch side set to match, not just this side.
  ETH.begin(ETH_PHY_TYPE, ETH_PHY_ADDR, ETH_PHY_MDC, ETH_PHY_MDIO, ETH_PHY_POWER, ETH_PHY_CLK_MODE);
}

// Used both for the human-facing status display AND as the Teensy-facing
// CMD_GET_IP response, which the Teensy treats as "we have real network
// connectivity" the moment it's non-zero. AP_SETUP deliberately reports
// 0.0.0.0 here even though the SoftAP itself has an address (10.5.5.5) -
// that address is an isolated local-only hotspot with no path to the
// actual voter host, and reporting it as a real IP was making the Teensy
// believe it was online and start trying to connect through it. Use
// WiFi.softAPIP() directly (not this function) anywhere that needs the
// AP's own address for display purposes.
IPAddress currentIP() {
  if (netMode == NetMode::ETH_ACTIVE) return ETH.localIP();
  if (netMode == NetMode::WIFI_ACTIVE) return WiFi.localIP();
  return IPAddress(0, 0, 0, 0);
}

// AP setup mode: if there's no network at all (no ETH, and WiFi either
// isn't configured or can't connect) for AP_TRIGGER_MS, spin up a
// no-network-required hotspot so WiFi can be configured from a phone/
// laptop with no other access to the device. One-shot per boot (like the
// reference implementation this is modeled on, in the Analog Meter Clock
// project) - if AP_TIMEOUT_MS passes with nobody connecting, give up and
// go back to normal ETH/WiFi retry cycling; a fresh Teensy reboot (which
// happens on every settings save anyway) is what re-arms it.
#define AP_SSID "TeensyVoter_Config"
DNSServer dnsServer;
IPAddress apIP(10, 5, 5, 5);
bool apGivenUp = false;
uint32_t noNetworkSince = 0;
uint32_t apStartTime = 0;
const uint32_t AP_TRIGGER_MS = 15000;  // no network at all for this long -> offer the setup hotspot
const uint32_t AP_TIMEOUT_MS = 300000; // 5 minutes, then give up

void startApSetup() {
  Serial.println("[NET] No network for a while - starting WiFi setup hotspot");
  netMode = NetMode::AP_SETUP;
  apStartTime = millis();

  WiFi.persistent(false); // avoid NVS wear from a mode that toggles often
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
  WiFi.softAP(AP_SSID, "", 1, false, 4); // open, channel 1, not hidden, max 4 clients

  dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
  dnsServer.start(53, "*", apIP); // wildcard - any hostname resolves to us, for the captive-portal popup

  digitalWrite(LED_AP_SETUP, LOW); // on (active-low)
}

void stopApSetup(NetMode nextMode) {
  dnsServer.stop();
  WiFi.softAPdisconnect(true);
  if (nextMode != NetMode::WIFI_ACTIVE) {
    WiFi.mode(WIFI_OFF);
  }
  netMode = nextMode;
  digitalWrite(LED_AP_SETUP, HIGH); // off
}

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
  if (cfgCmdQueueCount >= CFG_CMD_QUEUE_SIZE) {
    Serial.println("[CFG] Command queue full, dropping command!");
    return;
  }
  memcpy(cfgCmdQueue[cfgCmdQueueTail], data, len);
  cfgCmdQueueLen[cfgCmdQueueTail] = len;
  cfgCmdQueueTail = (cfgCmdQueueTail + 1) % CFG_CMD_QUEUE_SIZE;
  cfgCmdQueueCount++;
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

void sendSetParamU8(uint8_t paramId, uint8_t val) {
  sendSetParam(paramId, &val, 1);
}

void sendSetParamIP(uint8_t paramId, IPAddress ip) {
  uint8_t b[4] = {ip[0], ip[1], ip[2], ip[3]};
  sendSetParam(paramId, b, 4);
}

void sendSaveReboot() {
  uint8_t b = CFG_CMD_SAVE_REBOOT;
  queueConfigCmd(&b, 1);
}

// Post/Redirect/Get: send a redirect to a plain GET page instead of
// rendering the "Saved" response directly in the POST reply. Otherwise
// the "Saved" page IS the POST response, so refreshing it resubmits the
// same POST and triggers another save+reboot.
void sendRedirectToRebooting() {
  webServer.sendHeader("Location", "/rebooting", true);
  webServer.send(302, "text/plain", "");
}

String htmlHeader(const char* title) {
  String html = "<!DOCTYPE html><html><head><meta charset=\"UTF-8\">";
  html += "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">";
  html += "<title>Voter - " + String(title) + "</title>";
  html += HTML_STYLE;
  html += "<script>function toggleShow(id){var e=document.getElementById(id);e.type=(e.type==='password')?'text':'password';}</script>";
  html += "</head><body><div class=\"container\">";
  html += "<h1>TeensyVoter</h1>";
  html += "<div class=\"nav\"><a href=\"/\">Status</a><a href=\"/voter\">Voter</a><a href=\"/radio\">Radio</a><a href=\"/network\">Network</a><a href=\"/wifi\">WiFi</a></div>";
  return html;
}

String htmlFooter() {
  return "</div></body></html>";
}

void handleRebootingPage() {
  String html = htmlHeader("Rebooting");
  html += "<div class=\"card\"><h2>Saved</h2><p>Rebooting Teensy...</p><a href=\"/\">Back to Status</a></div>";
  html += htmlFooter();
  webServer.send(200, "text/html", html);
}

void handleRoot() {
  String html = htmlHeader("Status");
  html += "<div class=\"card\"><h2>System Status</h2>";
  if (!configReceived) {
    html += "<p><span class=\"status warn\">Waiting for Teensy...</span></p>";
  } else {
    if (netMode == NetMode::WIFI_ACTIVE) {
      html += "<p><strong>Interface:</strong> <span class=\"status warn\">WiFi (Ethernet fallback)</span></p>";
      html += "<p><strong>IP Address:</strong> " + WiFi.localIP().toString() + "</p>";
      html += "<p><strong>WiFi RSSI:</strong> " + String(WiFi.RSSI()) + " dBm</p>";
      html += "<p><strong>Gateway:</strong> " + WiFi.gatewayIP().toString() + "</p>";
      html += "<p><strong>DNS:</strong> " + WiFi.dnsIP().toString() + "</p>";
    } else if (netMode == NetMode::ETH_ACTIVE) {
      html += "<p><strong>Interface:</strong> <span class=\"status ok\">Ethernet</span></p>";
      html += "<p><strong>IP Address:</strong> " + ETH.localIP().toString() + "</p>";
      html += "<p><strong>Link:</strong> " + String(ETH.linkSpeed()) + "Mbps</p>";
      html += "<p><strong>Gateway:</strong> " + ETH.gatewayIP().toString() + "</p>";
      html += "<p><strong>DNS:</strong> " + ETH.dnsIP().toString() + "</p>";
    } else if (netMode == NetMode::AP_SETUP) {
      html += "<p><strong>Interface:</strong> <span class=\"status warn\">WiFi Setup Hotspot</span></p>";
      html += "<p>Connect to \"" AP_SSID "\" and go to <a href=\"/wifi\">WiFi setup</a>.</p>";
    } else {
      html += "<p><span class=\"status warn\">No network link yet...</span></p>";
    }
    String hostDisplay = (cachedConfig.hostname[0] != '\0') ? String(cachedConfig.hostname) : IPAddress(cachedConfig.hostIP).toString();
    html += "<p><strong>Voter Host:</strong> " + hostDisplay + ":" + String(cachedConfig.hostPort) + "</p>";
  }
  html += "</div>";

  html += "<div class=\"card\"><h2>GPS</h2>";
  if (!gpsStatusReceived) {
    html += "<p><span class=\"status warn\">No GPS data yet...</span></p>";
  } else {
    html += String("<p><strong>Lock:</strong> <span class=\"status ") + (cachedGpsStatus.locked ? "ok\">Locked" : "warn\">No Lock") + "</span></p>";
    html += "<p><strong>Satellites:</strong> " + String(cachedGpsStatus.satellites) + "</p>";
    html += String("<p><strong>Time Set:</strong> ") + (cachedGpsStatus.timeSet ? "Yes" : "No") + "</p>";
    html += "<p><strong>PPS Jitter:</strong> " + String(cachedGpsStatus.ppsJitterUs) + " us</p>";
    html += "<p><strong>Location:</strong> " + String(cachedGpsStatus.lat) + ", " + String(cachedGpsStatus.lon) +
            " (Elev: " + String(cachedGpsStatus.elev) + ")</p>";
  }
  html += "</div>" + htmlFooter();
  webServer.send(200, "text/html", html);
}

void handleVoter() {
  // hostname takes display priority when set (matches the Teensy serial
  // CLI's own "Host Addr" logic) - fall back to the numeric hostIP when
  // it's empty, otherwise the field just looked blank even though a host
  // was actually configured.
  String hostDisplay = (cachedConfig.hostname[0] != '\0')
                            ? String(cachedConfig.hostname)
                            : IPAddress(cachedConfig.hostIP).toString();

  String html = htmlHeader("Voter");
  html += "<div class=\"card\"><h2>Voter Settings</h2><form method=\"POST\" action=\"/voter\">";
  html += "<label>Host Address (IP or hostname):</label><input type=\"text\" name=\"host\" value=\"" + hostDisplay + "\">";
  html += "<label>Host Port:</label><input type=\"number\" name=\"port\" value=\"" + String(cachedConfig.hostPort) + "\">";
  html += "<label>Client Password:</label><input type=\"password\" id=\"clientPwd\" name=\"clientPwd\" value=\"" + String(cachedConfig.clientPwd) + "\">";
  html += "<label><input type=\"checkbox\" onclick=\"toggleShow('clientPwd')\"> Show</label>";
  html += "<label>Host Password:</label><input type=\"password\" id=\"hostPwd\" name=\"hostPwd\" value=\"" + String(cachedConfig.hostPwd) + "\">";
  html += "<label><input type=\"checkbox\" onclick=\"toggleShow('hostPwd')\"> Show</label>";
  html += "<input type=\"submit\" value=\"Save\" class=\"btn\"></form></div>";
  html += htmlFooter();
  webServer.send(200, "text/html", html);
}

void handleVoterPost() {
  // Mirror the serial CLI's "Host Addr" behavior: an IP-literal entry
  // sets hostIP and clears hostname (hostname takes display/resolution
  // priority when non-empty, so a stale hostname would otherwise keep
  // overriding a freshly-entered IP).
  IPAddress hostIp;
  String hostArg = webServer.arg("host");
  if (hostIp.fromString(hostArg)) {
    sendSetParamIP(PARAM_HOST_IP, hostIp);
    sendSetParamStr(PARAM_HOSTNAME, "");
  } else {
    sendSetParamStr(PARAM_HOSTNAME, hostArg);
  }
  sendSetParamU16(PARAM_HOST_PORT, webServer.arg("port").toInt());
  sendSetParamStr(PARAM_CLIENT_PWD, webServer.arg("clientPwd"));
  sendSetParamStr(PARAM_HOST_PWD, webServer.arg("hostPwd"));
  sendSaveReboot();
  sendRedirectToRebooting();
}

void handleRadio() {
  String html = htmlHeader("Radio");
  html += "<div class=\"card\"><h2>Radio Settings</h2><form method=\"POST\" action=\"/radio\">";

  html += "<label>COS Mode:</label><select name=\"cosMode\">";
  html += String("<option value=\"0\"") + (cachedConfig.cosMode == 0 ? " selected" : "") + ">Always On</option>";
  html += String("<option value=\"1\"") + (cachedConfig.cosMode == 1 ? " selected" : "") + ">Hardware</option>";
  html += String("<option value=\"2\"") + (cachedConfig.cosMode == 2 ? " selected" : "") + ">DSP</option>";
  html += "</select>";
  html += String("<label><input type=\"checkbox\" name=\"cosInvert\"") + (cachedConfig.cosInvert ? " checked" : "") + "> Invert COS</label>";
  html += "<label>DSP Squelch Threshold (0-255):</label><input type=\"number\" min=\"0\" max=\"255\" name=\"dspSquelch\" value=\"" + String(cachedConfig.dspSquelchThresh) + "\">";

  html += "<label>RX Analog Gain (0-15):</label><input type=\"number\" min=\"0\" max=\"15\" name=\"rxAnalogGain\" value=\"" + String(cachedConfig.radioRxAnalogGain) + "\">";
  html += "<label>RX Digital Gain (%):</label><input type=\"number\" min=\"0\" max=\"500\" name=\"rxDigitalGain\" value=\"" + String(cachedConfig.radioRxDigitalGainPct) + "\">";
  html += "<label>Input Source:</label><select name=\"inputSource\">";
  html += String("<option value=\"0\"") + (cachedConfig.inputSource == 0 ? " selected" : "") + ">Line In</option>";
  html += String("<option value=\"1\"") + (cachedConfig.inputSource == 1 ? " selected" : "") + ">Mic</option>";
  html += "</select>";
  html += String("<label><input type=\"checkbox\" name=\"deemp\"") + (cachedConfig.enableDeemp ? " checked" : "") + "> De-emphasis</label>";
  html += String("<label><input type=\"checkbox\" name=\"plFilter\"") + (cachedConfig.enablePLFilter ? " checked" : "") + "> PL Filter</label>";

  html += String("<label><input type=\"checkbox\" name=\"useHwRssi\"") + (cachedConfig.useHwRSSI ? " checked" : "") + "> Use Hardware RSSI (ADC)</label>";
  html += "<label>RSSI Min:</label><input type=\"number\" name=\"rssiMin\" value=\"" + String(cachedConfig.rssiMin) + "\">";
  html += "<label>RSSI Max:</label><input type=\"number\" name=\"rssiMax\" value=\"" + String(cachedConfig.rssiMax) + "\">";

  html += "<label>TX Master Gain (%):</label><input type=\"number\" min=\"0\" max=\"100\" name=\"txGain\" value=\"" + String(cachedConfig.radioTxMasterGainPct) + "\">";
  html += String("<label><input type=\"checkbox\" name=\"pttInvert\"") + (cachedConfig.pttInvert ? " checked" : "") + "> PTT Active LOW</label>";
  html += "<label>PTT Tail (ms):</label><input type=\"number\" name=\"pttTail\" value=\"" + String(cachedConfig.pttTailMs) + "\">";

  html += "<input type=\"submit\" value=\"Save\" class=\"btn\"></form></div>";
  html += htmlFooter();
  webServer.send(200, "text/html", html);
}

void handleRadioPost() {
  sendSetParamU8(PARAM_COS_MODE, (uint8_t)webServer.arg("cosMode").toInt());
  sendSetParamU8(PARAM_COS_INVERT, webServer.hasArg("cosInvert") ? 1 : 0);
  sendSetParamU8(PARAM_DSP_SQUELCH, (uint8_t)webServer.arg("dspSquelch").toInt());

  sendSetParamU8(PARAM_RX_GAIN, (uint8_t)webServer.arg("rxAnalogGain").toInt());
  sendSetParamU8(PARAM_RX_DIGITAL_GAIN_PCT, (uint8_t)webServer.arg("rxDigitalGain").toInt());
  sendSetParamU8(PARAM_INPUT_SOURCE, (uint8_t)webServer.arg("inputSource").toInt());
  sendSetParamU8(PARAM_DEEMP, webServer.hasArg("deemp") ? 1 : 0);
  sendSetParamU8(PARAM_PL_FILTER, webServer.hasArg("plFilter") ? 1 : 0);

  sendSetParamU8(PARAM_USE_HW_RSSI, webServer.hasArg("useHwRssi") ? 1 : 0);
  sendSetParamU16(PARAM_RSSI_MIN, (uint16_t)webServer.arg("rssiMin").toInt());
  sendSetParamU16(PARAM_RSSI_MAX, (uint16_t)webServer.arg("rssiMax").toInt());

  sendSetParamU8(PARAM_TX_GAIN_PCT, (uint8_t)webServer.arg("txGain").toInt());
  sendSetParamU8(PARAM_PTT_INVERT, webServer.hasArg("pttInvert") ? 1 : 0);
  sendSetParamU16(PARAM_PTT_TAIL_MS, (uint16_t)webServer.arg("pttTail").toInt());

  sendSaveReboot();
  sendRedirectToRebooting();
}

void handleNetwork() {
  String html = htmlHeader("Network");
  html += "<div class=\"card\"><h2>Ethernet Addressing</h2><form method=\"POST\" action=\"/network\">";
  html += String("<label><input type=\"checkbox\" name=\"useStaticIP\"") + (cachedConfig.useStaticIP ? " checked" : "") + "> Use Static IP (unchecked = DHCP)</label>";
  html += "<label>Static IP:</label><input type=\"text\" name=\"staticIP\" value=\"" + IPAddress(cachedConfig.staticIP).toString() + "\">";
  html += "<label>Subnet Mask:</label><input type=\"text\" name=\"subnetMask\" value=\"" + IPAddress(cachedConfig.subnetMask).toString() + "\">";
  html += "<label>Gateway:</label><input type=\"text\" name=\"gateway\" value=\"" + IPAddress(cachedConfig.gateway).toString() + "\">";
  html += "<label>DNS (used with Static IP):</label><input type=\"text\" name=\"staticDNS\" value=\"" + IPAddress(cachedConfig.staticDNS).toString() + "\">";
  html += "<label>DNS Override (used with DHCP; 0.0.0.0 = use DHCP default):</label><input type=\"text\" name=\"dns\" value=\"" + IPAddress(cachedConfig.dnsServerIP).toString() + "\">";
  html += "<h2>Syslog</h2>";
  html += String("<label><input type=\"checkbox\" name=\"useSyslog\"") + (cachedConfig.useSyslog ? " checked" : "") + "> Enable Syslog</label>";
  html += "<label>Syslog Host (IP or hostname):</label><input type=\"text\" name=\"syslogHost\" value=\"" + String(cachedConfig.syslogHostname) + "\">";
  html += "<label>Syslog Port:</label><input type=\"number\" name=\"syslogPort\" value=\"" + String(cachedConfig.syslogPort) + "\">";
  html += "<input type=\"submit\" value=\"Save\" class=\"btn\"></form></div>";
  html += htmlFooter();
  webServer.send(200, "text/html", html);
}

void handleNetworkPost() {
  sendSetParamU8(PARAM_USE_STATIC_IP, webServer.hasArg("useStaticIP") ? 1 : 0);

  IPAddress ip;
  if (ip.fromString(webServer.arg("staticIP"))) sendSetParamIP(PARAM_STATIC_IP, ip);
  if (ip.fromString(webServer.arg("subnetMask"))) sendSetParamIP(PARAM_SUBNET_MASK, ip);
  if (ip.fromString(webServer.arg("gateway"))) sendSetParamIP(PARAM_GATEWAY, ip);
  if (ip.fromString(webServer.arg("staticDNS"))) sendSetParamIP(PARAM_STATIC_DNS, ip);
  if (ip.fromString(webServer.arg("dns"))) sendSetParamIP(PARAM_DNS_SERVER_IP, ip);

  sendSetParamU8(PARAM_USE_SYSLOG, webServer.hasArg("useSyslog") ? 1 : 0);
  sendSetParamStr(PARAM_SYSLOG_HOSTNAME, webServer.arg("syslogHost"));
  sendSetParamU16(PARAM_SYSLOG_PORT, webServer.arg("syslogPort").toInt());
  sendSaveReboot();
  sendRedirectToRebooting();
}

void handleWiFi() {
  String html = htmlHeader("WiFi");
  if (netMode == NetMode::AP_SETUP) {
    html += "<div class=\"card\"><h2><span class=\"status warn\">Setup Hotspot Active</span></h2>";
    html += "<p>No Ethernet or working WiFi was found, so this device is broadcasting its own \"" AP_SSID "\" network so you can configure WiFi from here. "
            "Enter your network below and Save - the device will reboot and try to connect.</p>";
    html += "<p>Gives up in " + String((AP_TIMEOUT_MS - (millis() - apStartTime)) / 1000) + "s if nothing is saved.</p></div>";
  }
  html += "<div class=\"card\"><h2>WiFi Fallback</h2>";
  html += "<p>Used only if Ethernet has no cable link for " + String(ETH_LINK_DOWN_FAILOVER_MS / 1000) +
          "s, or has link but no IP for " + String(ETH_NO_IP_FAILOVER_MS / 1000) +
          "s. Ethernet is always preferred.</p>";
  html += "<form method=\"POST\" action=\"/wifi\">";
  html += "<button type=\"button\" onclick=\"scanWifi()\">Scan for Networks</button>";
  html += "<div id=\"scanStatus\"></div>";
  html += "<select id=\"scanResults\" style=\"display:none\" onchange=\"document.getElementById('ssidInput').value=this.value\"></select>";
  html += "<label>SSID:</label><input type=\"text\" id=\"ssidInput\" name=\"ssid\" value=\"" + String(cachedConfig.wifiSSID) + "\">";
  html += "<label>Password:</label><input type=\"password\" id=\"wifiPass\" name=\"pass\" value=\"" + String(cachedConfig.wifiPass) + "\">";
  html += "<label><input type=\"checkbox\" onclick=\"toggleShow('wifiPass')\"> Show</label>";
  html += "<input type=\"submit\" value=\"Save\" class=\"btn\"></form></div>";
  html += "<script>";
  html += "function scanWifi(){document.getElementById('scanStatus').innerText='Scanning...';fetch('/wifi/scan',{method:'POST'}).then(pollScan);}";
  html += "function pollScan(){fetch('/wifi/scan_results').then(function(r){return r.text();}).then(function(text){"
          "if(text==='running'){setTimeout(pollScan,1000);return;}"
          "var networks=text.split('\\n').filter(function(s){return s.length>0;});"
          "var sel=document.getElementById('scanResults');"
          "sel.innerHTML='<option value=\"\">-- Select a network --</option>';"
          "networks.forEach(function(ssid){var opt=document.createElement('option');opt.value=ssid;opt.text=ssid;sel.appendChild(opt);});"
          "sel.style.display=networks.length?'block':'none';"
          "document.getElementById('scanStatus').innerText=networks.length?(networks.length+' networks found'):'No networks found';"
          "});}";
  html += "</script>";
  html += htmlFooter();
  webServer.send(200, "text/html", html);
}

void handleWifiScanStart() {
  if (netMode == NetMode::AP_SETUP) {
    WiFi.mode(WIFI_AP_STA); // keep the setup hotspot alive while also scanning as STA
  } else if (WiFi.getMode() == WIFI_OFF) {
    WiFi.mode(WIFI_STA);
  }
  WiFi.scanNetworks(true); // async - must not block loop(), which is audio-timing-critical
  webServer.send(200, "text/plain", "started");
}

void handleWifiScanResults() {
  int n = WiFi.scanComplete();
  if (n == WIFI_SCAN_RUNNING) {
    webServer.send(200, "text/plain", "running");
    return;
  }

  String out;
  if (n > 0) {
    for (int i = 0; i < n; i++) {
      String ssid = WiFi.SSID(i);
      if (ssid.length() == 0) continue; // hidden network, nothing to show
      out += ssid + "\n";
    }
  }
  WiFi.scanDelete();
  if (netMode == NetMode::AP_SETUP) {
    WiFi.mode(WIFI_AP); // scan done - drop the STA side, keep just the setup hotspot
  }
  webServer.send(200, "text/plain", out);
}

void handleWiFiPost() {
  sendSetParamStr(PARAM_WIFI_SSID, webServer.arg("ssid"));
  sendSetParamStr(PARAM_WIFI_PASS, webServer.arg("pass"));
  sendSaveReboot();
  sendRedirectToRebooting();
}

void initSpiSlave();

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Spirit Firmware 3.4 (Sync Enabled)");
#ifdef BUILD_TIMESTAMP
  Serial.print("Build Date: ");
  Serial.println(BUILD_TIMESTAMP);
#endif

  // Event-driven ETH link logging - the HB heartbeat only samples
  // ETH.linkUp() once every 5s, too coarse to tell a PHY that's genuinely
  // flapping from one that came up once and never retried. These fire
  // immediately as the core sees them.
  WiFi.onEvent([](arduino_event_id_t event, arduino_event_info_t info) {
    switch (event) {
      case ARDUINO_EVENT_ETH_START:        Serial.printf("[ETH-EVT] %lu START\r\n", millis()); break;
      case ARDUINO_EVENT_ETH_CONNECTED:    Serial.printf("[ETH-EVT] %lu CONNECTED (link up)\r\n", millis()); break;
      case ARDUINO_EVENT_ETH_DISCONNECTED: {
        Serial.printf("[ETH-EVT] %lu DISCONNECTED (link down)\r\n", millis());
        uint32_t now = millis();
        if (now - ethFlapWindowStart > ETH_FLAP_WINDOW_MS) {
          ethFlapWindowStart = now;
          ethFlapCountInWindow = 0;
        }
        ethFlapCountInWindow++;
        if (ethFlapCountInWindow >= ETH_FLAP_THRESHOLD) {
          ethReinitRequested = true;
        }
        break;
      }
      case ARDUINO_EVENT_ETH_GOT_IP:       Serial.printf("[ETH-EVT] %lu GOT_IP\r\n", millis()); break;
      case ARDUINO_EVENT_ETH_LOST_IP:      Serial.printf("[ETH-EVT] %lu LOST_IP\r\n", millis()); break;
      case ARDUINO_EVENT_ETH_STOP:         Serial.printf("[ETH-EVT] %lu STOP\r\n", millis()); break;
      default: break;
    }
  });

  pinMode(LED_AP_SETUP, OUTPUT);
  digitalWrite(LED_AP_SETUP, HIGH); // off (active-low)

  pinMode(GPIO_READY, OUTPUT);
  digitalWrite(GPIO_READY, LOW);

  initSpiSlave();

  // ETH.begin() is deliberately NOT called here - it's deferred until
  // config arrives from the Teensy (see loop()), since static IP settings
  // have to be known before ETH.begin() can apply them.
  Serial.println("SPI Slave Started");
}

// Shared by setup() and resyncSpiSlave() - brings the SPI slave peripheral
// up from scratch and arms the one always-queued transaction.
void initSpiSlave() {
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

  // Keep a transaction perpetually queued/armed instead of calling
  // blocking spi_slave_transmit() once per loop iteration.
  memset(sendbuf, 0, MAX_SPI_BUF);
  memset(recvbuf, 0, MAX_SPI_BUF);
  t.length = MAX_SPI_BUF * 8;
  t.tx_buffer = sendbuf;
  t.rx_buffer = recvbuf;
  spi_slave_queue_trans(RCV_HOST, &t, portMAX_DELAY);
}

void loop() {
  // Diagnostic only, not a fix: flag any single loop() iteration that
  // takes abnormally long. This ESP32 loop() gained a lot this session
  // (WiFi/AP state machine every iteration, DNS server, web server, LED,
  // GPS-status capture, the config queue) that wasn't there before - if
  // any of that is delaying how promptly the SPI slave gets re-armed for
  // the Teensy's real-time audio traffic, this is where it would show up
  // as a stall, without necessarily showing up as a stall on the
  // Teensy's own (already-checked, clean) main loop timing.
  static uint32_t lastLoopStart = 0;
  uint32_t loopStartUs = micros();
  if (lastLoopStart != 0) {
    uint32_t iterDuration = loopStartUs - lastLoopStart;
    if (iterDuration > 20000) {
      Serial.printf("[TIMING] loop() iteration took %lu us\r\n", (unsigned long)iterDuration);
    }
  }
  lastLoopStart = loopStartUs;

  // Bring up ETH once config has arrived - needed to know static IP
  // settings before ETH.begin() can apply them. Safety net: if config
  // hasn't shown up yet (e.g. the boot-time pushConfig() SPI transaction
  // was missed), keep asking for it every second.
  if (!ethStarted) {
    if (configReceived) {
      // Explicit PHY reset pulse (inside startEth()) before ETH.begin().
      // The ESP-IDF LAN87xx driver does its own reset pulse via the same
      // pin internally, but hardcodes it to 150us
      // (LAN87XX_PHY_RESET_ASSERTION_TIME_US) - far short of the LAN8720's
      // real datasheet-required reset time (tens of ms). That's enough
      // margin on a cold power-on (the power rail's own rise time covers
      // the gap) but not on a warm/soft reset (e.g. after a firmware
      // upload, which only resets the ESP32 - the PHY chip's own power
      // never drops, so it can still be mid-autonegotiation from before
      // and this tiny pulse doesn't force a clean reset). Bench-confirmed:
      // flapping link after every soft-reset reflash, fixed by a full
      // power cycle. A real, properly-timed pulse here makes the
      // library's own short one redundant instead of load-bearing.
      startEth();
      ethStarted = true;
      lastEthUp = millis(); // failover grace period starts here, not at power-on
      Serial.println("[NET] Ethernet started");
    } else {
      static uint32_t lastCfgReq = 0;
      if (millis() - lastCfgReq > 1000) {
        lastCfgReq = millis();
        uint8_t req = CFG_CMD_REQUEST_CONFIG;
        queueConfigCmd(&req, 1);
      }
    }
  } else if (ethReinitRequested) {
    // Software self-heal: the link has been flapping (ETH_FLAP_THRESHOLD+
    // DISCONNECTED events inside ETH_FLAP_WINDOW_MS) even with the proper
    // reset pulse above, or something else knocked the PHY into a bad
    // state mid-session. Full de-init/re-init cycle rather than hoping
    // the next natural retry works - this is the actual answer to "can we
    // trust a remote firmware update not to strand the device": the
    // firmware recovers itself instead of needing a physical power cycle.
    // Capped at ETH_REINIT_MAX so a genuinely dead PHY doesn't loop
    // forever burning cycles.
    ethReinitRequested = false;
    ethFlapCountInWindow = 0;
    if (ethReinitCount < ETH_REINIT_MAX) {
      ethReinitCount++;
      Serial.printf("[NET] ETH link flapping (#%lu) - re-initializing PHY\r\n",
                    (unsigned long)ethReinitCount);
      ETH.end();
      delay(200);
      startEth();
      lastEthUp = millis();
    } else {
      Serial.println("[NET] ETH flap recovery attempts exhausted - leaving as-is, WiFi failover will take over if configured");
    }
  }

  // ETH/WiFi failover - ETH always preferred.
  if (ethStarted) {
    bool phyLinkUp = ETH.linkUp();
    if (phyLinkUp) {
      if (phyLinkUpSince == 0) phyLinkUpSince = millis();
      linkDownSince = 0;
    } else {
      phyLinkUpSince = 0;
      if (linkDownSince == 0) linkDownSince = millis();
    }

    bool eUp = phyLinkUp && (uint32_t)ETH.localIP() != 0;
    if (eUp) lastEthUp = millis();

    // Drop WiFi the moment the physical ETH link is confirmed back
    // (briefly debounced against flicker) - deliberately NOT waiting for
    // ETH to finish DHCP and get an IP first. Waiting for a stable IP
    // meant both the WiFi radio and the ETH PHY could be fully active
    // (both with live IP sessions) for several seconds during failback,
    // which correlated with intermittent ESP32 reboots on real hardware
    // - most likely a brownout from both radios drawing power at once,
    // though not confirmed without a crash dump. This trades a few
    // seconds of no network (while ETH does its own DHCP) for avoiding
    // that dual-active window entirely.
    if (netMode == NetMode::WIFI_ACTIVE && phyLinkUp &&
        (millis() - phyLinkUpSince > ETH_LINK_UP_DEBOUNCE_MS)) {
      WiFi.disconnect(true);
      WiFi.mode(WIFI_OFF);
      netMode = NetMode::NONE;
      lastEthUp = millis(); // give ETH a fresh DHCP grace period - otherwise
                             // this timestamp is stale (from boot, or from
                             // whenever ETH last actually had an IP) and the
                             // noIpFailover check below fires on the very
                             // next loop(), bouncing straight back to WiFi
                             // before DHCP has any chance to complete
      Serial.println("[NET] Ethernet link restored - dropped WiFi fallback (not waiting for DHCP)");
    }

    // Same idea for the WiFi setup hotspot - ETH always wins, even mid-setup.
    if (netMode == NetMode::AP_SETUP && eUp) {
      Serial.println("[NET] Ethernet available - exiting WiFi setup hotspot");
      stopApSetup(NetMode::ETH_ACTIVE);
    }

    if (eUp) {
      netMode = NetMode::ETH_ACTIVE;
    } else if (netMode != NetMode::WIFI_ACTIVE) {
      if (WiFi.status() == WL_CONNECTED) {
        netMode = NetMode::WIFI_ACTIVE;
      }

      // Fast path: PHY link is physically down (cable pulled) - an
      // unambiguous signal, fail over quickly. Slow path: link is up but
      // DHCP hasn't completed yet - more likely transient, be patient.
      bool linkDownFailover = !phyLinkUp && (millis() - linkDownSince > ETH_LINK_DOWN_FAILOVER_MS);
      bool noIpFailover = phyLinkUp && (millis() - lastEthUp > ETH_NO_IP_FAILOVER_MS);

      // Not while AP_SETUP is running - retrying the same known-bad
      // credentials would call WiFi.mode(WIFI_STA) every WIFI_RETRY_MS,
      // which tears down the setup hotspot itself (observed on hardware:
      // the AP's own IP disappeared on the very next retry). AP_SETUP is
      // the terminal "give up on auto-reconnect, wait for a human"
      // state - it shouldn't be silently undone by the same logic that
      // led to it.
      if (netMode != NetMode::WIFI_ACTIVE && netMode != NetMode::AP_SETUP &&
          configReceived && cachedConfig.wifiSSID[0] != '\0' &&
          (linkDownFailover || noIpFailover) &&
          (!wifiAttempted || millis() - lastWifiAttempt > WIFI_RETRY_MS)) {
        wifiAttempted = true;
        lastWifiAttempt = millis();
        Serial.println("[NET] Ethernet down, attempting WiFi fallback...");
        // Disconnect before every attempt, including retries - calling
        // begin() again on top of a still-lingering failed attempt (not
        // fully cleared) is a known ESP32 WiFi driver trigger for a
        // "STA clear config failed" internal error, observed on hardware
        // right before a retry.
        WiFi.persistent(false); // failover can call begin() many times a
                                 // session - without this every call writes
                                 // SSID+password to NVS, wearing/risking
                                 // corruption of the flash WiFi partition
        WiFi.disconnect(true);
        WiFi.mode(WIFI_STA);
        WiFi.setSleep(false); // modem-sleep power save otherwise re-enables on
                               // every WIFI_STA re-init, adding beacon-interval
                               // RX/TX latency that shows up as audio jitter
        WiFi.begin(cachedConfig.wifiSSID, cachedConfig.wifiPass);
      }
    }
  }

  // Offer a WiFi setup hotspot if there's truly no network at all (no ETH,
  // and WiFi isn't configured or can't connect) for a while. One-shot per
  // boot - see startApSetup()/stopApSetup() comment above.
  if (ethStarted && !apGivenUp) {
    if (netMode == NetMode::NONE) {
      if (noNetworkSince == 0) {
        noNetworkSince = millis();
      } else if (millis() - noNetworkSince > AP_TRIGGER_MS) {
        startApSetup();
      }
    } else if (netMode != NetMode::AP_SETUP) {
      noNetworkSince = 0;
    }

    if (netMode == NetMode::AP_SETUP) {
      dnsServer.processNextRequest();
      if (millis() - apStartTime > AP_TIMEOUT_MS) {
        Serial.println("[NET] WiFi setup hotspot timed out, giving up");
        stopApSetup(NetMode::NONE);
        apGivenUp = true;
      }
    }
  }

  static uint32_t lastHb = 0;
  if (millis() - lastHb > 5000) {
    lastHb = millis();
    const char *modeStr = netMode == NetMode::ETH_ACTIVE ? "ETH" : netMode == NetMode::WIFI_ACTIVE ? "WIFI" : netMode == NetMode::AP_SETUP ? "AP_SETUP" : "NONE";
    // Display only - currentIP() deliberately returns 0.0.0.0 in AP_SETUP
    // (see its comment); show the SoftAP's real address here for
    // debugging visibility without affecting what the Teensy is told.
    String displayIP = (netMode == NetMode::AP_SETUP) ? WiFi.softAPIP().toString() : currentIP().toString();
    Serial.printf("HB Mode:%s IP:%s Cfg:%d CBs:%u Txns:%u UDPcmd:%u UDPfwd:%u MaxRearmUs:%u UnknownCmd:%u ShortSendCmd:%u CksumBad:%u Heap:%u MinHeap:%u PhyLink:%d SSID:%s WiFiTried:%d WiFiStatus:%d RSSI:%d\r\n",
                  modeStr, displayIP.c_str(), configReceived, cbCount,
                  txnCompletedCount, udpSendCmdSeenCount, udpForwardedCount, maxRearmWindowUs,
                  unknownCmdCount, shortSendCmdCount, cksumMismatchCount, ESP.getFreeHeap(),
                  ESP.getMinFreeHeap(), ETH.linkUp(),
                  cachedConfig.wifiSSID[0] != '\0' ? cachedConfig.wifiSSID : "(empty)",
                  wifiAttempted, (int)WiFi.status(),
                  netMode == NetMode::WIFI_ACTIVE ? WiFi.RSSI() : 0);
    txnCompletedCount = 0;
    udpForwardedCount = 0;
    udpSendCmdSeenCount = 0;
    cksumMismatchCount = 0;
    maxRearmWindowUs = 0;
    unknownCmdCount = 0;
    shortSendCmdCount = 0;
  }

  // Reverted (2026-08-07) - this and NET_SETTLE_MS/resyncSpiSlave() were
  // all added tonight chasing the WiFi-failover hang/desync. User then
  // reported the transmitted (uplink) audio itself sounding badly
  // corrupted - "60%+ loss" - even on a plain ETH boot with no failover
  // involved, which this evening's changes hadn't been present for during
  // the last confirmed-good test. Bisecting: reverted all three back to
  // exactly the pre-WiFi-work state (this comment marks where the rebind
  // block was) to test whether tonight's ESP32 changes are responsible,
  // since the Teensy firmware itself hasn't changed since the last
  // confirmed-good result. NET_SETTLE_MS, netModeSince, and
  // resyncSpiSlave() were all fully removed, not just disabled - see
  // memory/git history for the WiFi-failover-hang/desync context if this
  // needs revisiting later.
  static bool wasConnected = false;
  if (netMode != NetMode::NONE && !wasConnected) {
    wasConnected = true;
    udp.begin(8888);
    webServer.on("/", handleRoot);
    webServer.on("/voter", HTTP_GET, handleVoter);
    webServer.on("/voter", HTTP_POST, handleVoterPost);
    webServer.on("/radio", HTTP_GET, handleRadio);
    webServer.on("/radio", HTTP_POST, handleRadioPost);
    webServer.on("/network", HTTP_GET, handleNetwork);
    webServer.on("/network", HTTP_POST, handleNetworkPost);
    webServer.on("/wifi", HTTP_GET, handleWiFi);
    webServer.on("/wifi", HTTP_POST, handleWiFiPost);
    webServer.on("/rebooting", HTTP_GET, handleRebootingPage);
    webServer.on("/wifi/scan", HTTP_POST, handleWifiScanStart);
    webServer.on("/wifi/scan_results", HTTP_GET, handleWifiScanResults);

    // Captive-portal probes various OSes send when joining a new network -
    // answering these makes phones/laptops auto-popup the WiFi setup page
    // instead of the user having to know to browse to 10.5.5.5 themselves.
    // Harmless (just extra registered routes) outside AP_SETUP mode.
    webServer.on("/generate_204", []() { webServer.sendHeader("Location", "http://10.5.5.5/wifi", true); webServer.send(302); });
    webServer.on("/gen_204", []() { webServer.sendHeader("Location", "http://10.5.5.5/wifi", true); webServer.send(302); });
    webServer.on("/hotspot-detect.html", []() { webServer.sendHeader("Location", "http://10.5.5.5/wifi", true); webServer.send(302); });
    webServer.on("/library/test/success.html", []() { webServer.sendHeader("Location", "http://10.5.5.5/wifi", true); webServer.send(302); });
    webServer.on("/connecttest.txt", []() { webServer.send(200, "text/plain", "Microsoft Connect Test"); });
    webServer.on("/ncsi.txt", []() { webServer.send(200, "text/plain", "Microsoft NCSI"); });
    webServer.onNotFound([]() {
      if (netMode == NetMode::AP_SETUP) {
        webServer.sendHeader("Location", "http://10.5.5.5/wifi", true);
        webServer.send(302);
      } else {
        webServer.send(404, "text/plain", "Not found");
      }
    });

    webServer.begin();
    Serial.println("Network Connected, Web Server Started");
  }

  uint32_t webSvcStartUs = micros();
  if (wasConnected) webServer.handleClient();
  uint32_t webSvcDurUs = micros() - webSvcStartUs;
  if (webSvcDurUs > 5000) {
    Serial.printf("[TIMING] webServer.handleClient() took %lu us\r\n", (unsigned long)webSvcDurUs);
  }

  // Poll for a completed SPI transaction (non-blocking). Ruled out as an
  // audio-buzz cause via isolated A/B testing (see
  // docs/esp32_network_migration_plan.md) - restored for its real
  // benefit (frees the ESP32 to service webServer/heartbeat while a
  // transaction is outstanding instead of blocking up to 50ms). The 8-
  // tick ceiling only bounds idle wait; a real Teensy-initiated
  // transaction wakes this immediately regardless (FreeRTOS semaphore,
  // not interval polling). sendbuf/recvbuf must not be touched anywhere
  // else while a transaction is queued/in flight - queueConfigCmd()
  // stages into a separate buffer for exactly this reason.
  spi_slave_transaction_t *doneTrans = nullptr;
  esp_err_t ret = spi_slave_get_trans_result(RCV_HOST, &doneTrans, pdMS_TO_TICKS(8));

  if (ret == ESP_OK) {
    uint32_t rearmWindowStartUs = micros();
    txnCompletedCount++;
    char *sb = sendbuf;
    char *rb = recvbuf;
    // Clear READY pin immediately after transaction finishes
    GPIO.out_w1tc = (1 << GPIO_READY);
    dataPending = false;
    cfgCmdPending = false;
    // Deferred [SEQ] Gap reporting - populated below if a gap is detected,
    // but the actual Serial.printf() happens AFTER the requeue at the
    // bottom of this function, not inline. A gap print is relatively slow
    // (UART transmission isn't free), and printing it inside the critical
    // unarmed window - right when a loss just happened - was making the
    // very next transaction more likely to also be missed, a small
    // self-reinforcing cost on top of the real gap.
    bool seqGapPending = false;
    uint16_t seqGapExpected = 0, seqGapGot = 0, seqGapMissing = 0;
    const char *seqGapType = "";
    // Set after a CMD_SEND_UDP uplink send below - gates the downlink poll
    // further down so the two don't stack into the same pass. See that
    // gate's comment for why.
    bool justSentUplink = false;

    size_t bytesTransferred = doneTrans->trans_len / 8;
      if (bytesTransferred > 0) {
        uint8_t cmd = (uint8_t)rb[0];

        // Remove verbose [SPI] CMD logs as they cause timing jitter

        if (cmd == CMD_SEND_UDP) {
          // Counted here, before any filtering below, so a gap against
          // udpForwardedCount pinpoints whether the netUsable/bytesTransferred
          // gate is silently eating legitimate sends.
          udpSendCmdSeenCount++;
          // Only touch the socket when an interface is actually up. Right
          // after a netMode transition (e.g. WiFi.mode(WIFI_OFF) during
          // ETH failback) the WiFi/lwIP netif is mid-teardown for a moment;
          // calling into the UDP socket in that window was hitting a freed
          // netif structure and crashing (InstrFetchProhibited, PC=0) -
          // bench-confirmed by pairing this exact code path with the crash
          // in the serial log.
          bool netUsable = (netMode == NetMode::ETH_ACTIVE || netMode == NetMode::WIFI_ACTIVE);
          if (bytesTransferred < 13) {
            // A real CMD_SEND_UDP transaction is always MAX_SPI_BUF bytes
            // (Teensy always sends the full padded buffer) - a short
            // trans_len here means this transaction was truncated/glitched
            // in flight, not a normal skip. Counted separately from the
            // netUsable skip below (that one's an expected, non-corrupt
            // condition during net transitions).
            shortSendCmdCount++;
          }
          if (bytesTransferred >= 13 && netUsable) {
            uint16_t seq = ((uint8_t)rb[1] << 8) | (uint8_t)rb[2];
            uint16_t payloadLen = ((uint8_t)rb[3] << 8) | (uint8_t)rb[4];
            IPAddress ip((uint8_t)rb[5], (uint8_t)rb[6], (uint8_t)rb[7], (uint8_t)rb[8]);
            uint16_t port = ((uint8_t)rb[9] << 8) | (uint8_t)rb[10];
            uint16_t recvCksum = ((uint8_t)rb[11] << 8) | (uint8_t)rb[12];

            // Corruption check, independent of the SEQ check below - SEQ
            // only proves a transaction arrived, not that its content
            // survived intact. Added after clean SEQ/loss numbers still
            // came with reports of corrupted-sounding transmitted audio
            // at the server - a bit error that doesn't land on the SEQ
            // bytes is otherwise completely invisible.
            uint16_t cksumLen = (payloadLen > (uint16_t)(MAX_SPI_BUF - 13)) ? (MAX_SPI_BUF - 13) : payloadLen;
            uint16_t calcCksum = 0;
            for (uint16_t i = 0; i < cksumLen; i++) calcCksum += (uint8_t)rb[13 + i];
            if (calcCksum != recvCksum) {
              cksumMismatchCount++;
            }

            // Transport-layer sequence check - this counter increments on
            // EVERY sendPacketTo() call regardless of content (audio, GPS
            // status pushes, auth/keepalive, syslog), so a gap here does
            // NOT necessarily mean an audio frame was lost - it means
            // *something* on this leg was. Label using the arriving
            // packet's own Voter protocol payload_type field (bytes 22-23
            // of VOTER_PACKET_HEADER, present on port 1667 traffic) so the
            // log is honest about what's actually showing up, instead of
            // blaming "audio" for what might be a GPS push or keepalive.
            // Confirmed on hardware: gaps were still firing during
            // completely quiet channel periods with no real audio being
            // sent at all.
            const char *pktTypeStr = "other";
            if (port == 1667 && payloadLen >= 24) {
              uint16_t payloadType = ((uint8_t)rb[13 + 22] << 8) | (uint8_t)rb[13 + 23];
              switch (payloadType) {
                case 0: pktTypeStr = "AUTH"; break;
                case 1: pktTypeStr = "ULAW(audio)"; break;
                case 2: pktTypeStr = "GPS"; break;
                case 3: pktTypeStr = "ADPCM"; break;
                case 5: pktTypeStr = "PING"; break;
                default: pktTypeStr = "unknown-voter"; break;
              }
            }
            if (udpSeqInit) {
              uint16_t expected = (uint16_t)(lastUdpSeq + 1);
              if (seq != expected) {
                uint16_t gap = (uint16_t)(seq - expected);
                udpSeqDropCount += gap;
                // Stashed, not printed here - see the declaration comment
                // for why. pktTypeStr points at a string literal (static
                // storage), safe to carry the pointer past this scope.
                seqGapPending = true;
                seqGapExpected = expected;
                seqGapGot = seq;
                seqGapMissing = gap;
                seqGapType = pktTypeStr;
              }
            }
            lastUdpSeq = seq;
            udpSeqInit = true;

            // Debug non-audio packets (Syslog, etc) - rare (not the 50pps
            // audio path), left inline rather than deferred too.
            bool isAdminPacket = (port != 1667);

            // Stage the payload and defer the actual udp.beginPacket/
            // write/endPacket() until after the SPI slave is re-armed
            // (see the deferred-send block after the requeue below).
            // CMD_SEND_UDP is fire-and-forget - the Teensy never waits for
            // a same-transaction response to an audio send - so moving
            // the actual socket I/O out of this window doesn't touch any
            // synchronous request/response pairing the way the earlier,
            // reverted "delay every response by one transaction" attempt
            // did. This was the single largest remaining cost in the
            // critical unarmed window: individually never slow enough to
            // trip the 5ms alarm, but paid on every single audio
            // transaction, 50 times a second.
            uint16_t stageLen = (payloadLen > (uint16_t)(MAX_SPI_BUF - 13)) ? (MAX_SPI_BUF - 13) : payloadLen;
            memcpy(udpSendStagingBuf, &rb[13], stageLen);
            udpSendStagingLen = stageLen;
            udpSendStagingIP = ip;
            udpSendStagingPort = port;
            udpSendPending = true;
            if (isAdminPacket) {
              Serial.printf("[UDP] Sending Admin Packet to %s:%d (Len: %d)\r\n",
                            ip.toString().c_str(), port, payloadLen);
            }
            udpForwardedCount++;
            justSentUplink = true;
          }
        } else if (cmd == CMD_GET_IP) {
          memset(sb, 0, MAX_SPI_BUF);
          sb[0] = STATUS_HAS_DATA;
          sb[1] = 0x00;
          sb[2] = 0x04;
          IPAddress myIP = currentIP();
          sb[3] = myIP[0];
          sb[4] = myIP[1];
          sb[5] = myIP[2];
          sb[6] = myIP[3];
          dataPending = true;
          GPIO.out_w1ts = (1 << GPIO_READY);
          Serial.printf("[SPI] Reporting Local IP: %s\r\n", myIP.toString().c_str());
        } else if (cmd == CMD_PUSH_CONFIG) {
          uint16_t cfgLen = ((uint8_t)rb[1] << 8) | (uint8_t)rb[2];
          if (cfgLen <= sizeof(SysConfigMirror)) {
            memcpy(&cachedConfig, &rb[3], cfgLen);
            configReceived = true;
            Serial.printf("Config received! Host:%s\r\n", cachedConfig.hostname);
          }
        } else if (cmd == CMD_PUSH_GPS_STATUS) {
          uint16_t gpsLen = ((uint8_t)rb[1] << 8) | (uint8_t)rb[2];
          if (gpsLen == sizeof(GpsStatus)) {
            memcpy(&cachedGpsStatus, &rb[3], gpsLen);
            gpsStatusReceived = true;
          }
        } else if (cmd == CMD_DNS_LOOKUP) {
          uint8_t hostnameLen = (uint8_t)rb[1];
          if (hostnameLen > 0 && hostnameLen < 64) {
            char hostname[64] = {0};
            memcpy(hostname, &rb[2], hostnameLen);
            Serial.printf("[DNS] Request: %s\r\n", hostname);
            IPAddress resolvedIP;
            bool success = WiFi.hostByName(hostname, resolvedIP);

            memset(sb, 0, MAX_SPI_BUF);
            sb[0] = STATUS_HAS_DATA;
            sb[1] = 0x00;
            sb[2] = 0x04;
            if (success && resolvedIP != IPAddress(0,0,0,0)) {
              sb[3] = resolvedIP[0];
              sb[4] = resolvedIP[1];
              sb[5] = resolvedIP[2];
              sb[6] = resolvedIP[3];
              Serial.printf("[DNS] Resolved %s to %s\r\n", hostname, resolvedIP.toString().c_str());
            } else {
              sb[3] = 0; sb[4] = 0; sb[5] = 0; sb[6] = 0;
              Serial.printf("[DNS] Failed to resolve %s\r\n", hostname);
            }
            dataPending = true;
            GPIO.out_w1ts = (1 << GPIO_READY);
          }
        } else if (cmd == CMD_GET_DNS) {
          memset(sb, 0, MAX_SPI_BUF);
          sb[0] = STATUS_HAS_DATA;
          sb[1] = 0x00;
          sb[2] = 0x04;
          IPAddress dnsIP = (netMode == NetMode::WIFI_ACTIVE) ? WiFi.dnsIP() : ETH.dnsIP();
          sb[3] = dnsIP[0];
          sb[4] = dnsIP[1];
          sb[5] = dnsIP[2];
          sb[6] = dnsIP[3];
          dataPending = true;
          GPIO.out_w1ts = (1 << GPIO_READY);
          Serial.printf("[SPI] Reporting DNS: %s\r\n", dnsIP.toString().c_str());
        } else {
          // cmd byte didn't match any known command - a transaction that
          // physically arrived (bytesTransferred > 0) but with corrupted
          // content, silently dropped with no visibility until now.
          unknownCmdCount++;
        }
      }

    // Safe window: the previous transaction is confirmed complete and the
    // next hasn't been queued yet, so it's OK to prepare sb here.
    // Staged config commands take priority over UDP forwarding; some of
    // the branches above (GET_IP/DNS_LOOKUP/GET_DNS) already populated
    // sb and set dataPending directly, so don't clobber those.
    if (!dataPending && cfgCmdQueueCount > 0) {
      memset(sb, 0, MAX_SPI_BUF);
      sb[0] = STATUS_CONFIG_CMD;
      sb[1] = 0;
      sb[2] = cfgCmdQueueLen[cfgCmdQueueHead];
      memcpy(&sb[3], cfgCmdQueue[cfgCmdQueueHead], cfgCmdQueueLen[cfgCmdQueueHead]);
      cfgCmdQueueHead = (cfgCmdQueueHead + 1) % CFG_CMD_QUEUE_SIZE;
      cfgCmdQueueCount--;
      cfgCmdPending = true;
      GPIO.out_w1ts = (1 << GPIO_READY);
    } else if (!dataPending && !cfgCmdPending && !justSentUplink && wasConnected &&
               (netMode == NetMode::ETH_ACTIVE || netMode == NetMode::WIFI_ACTIVE)) {
      // Same netMode guard as the CMD_SEND_UDP path above - don't touch the
      // socket while the netif is mid-teardown/mid-bringup. (NET_SETTLE_MS
      // reverted 2026-08-07, see the wasConnected block's comment.)
      //
      // !justSentUplink: don't also poll for downlink data in the same
      // pass that just handled an uplink send. Every CMD_SEND_UDP
      // transaction was stacking a udp.beginPacket/write/endPacket() AND
      // a udp.parsePacket() into the same processing window - individually
      // both were fast enough to never trip the 5ms alarm on either one,
      // but paying that combined cost on literally every audio transaction
      // (not occasionally) is what was widening the SPI re-arm window
      // enough to lose frames constantly. Bench-confirmed via the
      // transport sequence numbers: ~47% loss during sustained audio
      // before this fix. Skipping the poll here just defers it to the
      // next transaction, which arrives within ~20ms during active audio
      // anyway - negligible for downlink timing.
      uint32_t udpParseStartUs = micros();
      int pktSize = udp.parsePacket();
      uint32_t udpParseDurUs = micros() - udpParseStartUs;
      if (udpParseDurUs > 5000) {
        Serial.printf("[TIMING] udp.parsePacket() took %lu us\r\n", (unsigned long)udpParseDurUs);
      }
      if (pktSize > 0) {
        memset(sb, 0, MAX_SPI_BUF);
        sb[0] = STATUS_HAS_DATA;
        sb[1] = (pktSize >> 8) & 0xFF;
        sb[2] = (pktSize & 0xFF);
        udp.read((uint8_t*)&sb[3], (pktSize > MAX_SPI_BUF-3) ? MAX_SPI_BUF-3 : pktSize);
        dataPending = true;
        GPIO.out_w1ts = (1 << GPIO_READY);
      } else {
        memset(sb, 0, MAX_SPI_BUF);
      }
    } else if (!dataPending && !cfgCmdPending) {
      // Neither branch above applied this pass (most commonly: pre-network
      // boot, wasConnected still false) - sb must still be reset back to
      // "no data" here. Without this, sb silently keeps whatever it last
      // held (e.g. the one-shot CFG_CMD_REQUEST_CONFIG retry) and that
      // stale STATUS_CONFIG_CMD content gets requeued and redelivered on
      // every subsequent transaction, since nothing else ever clears it -
      // the Teensy reads it fresh each time and queues it as a "new"
      // command, flooding its own RX ring buffer far faster than the real
      // 1-per-second retry rate. Confirmed on hardware: ~118 consecutive
      // "[CFG-RX] Queue full" drops during early boot before ETH came up.
      memset(sb, 0, MAX_SPI_BUF);
    }

    memset(rb, 0, MAX_SPI_BUF);
    t.length = MAX_SPI_BUF * 8;
    t.tx_buffer = sb;
    t.rx_buffer = rb;
    uint32_t spiQueueStartUs = micros();
    spi_slave_queue_trans(RCV_HOST, &t, portMAX_DELAY);
    uint32_t spiQueueDurUs = micros() - spiQueueStartUs;
    if (spiQueueDurUs > 5000) {
      Serial.printf("[TIMING] spi_slave_queue_trans() took %lu us\r\n", (unsigned long)spiQueueDurUs);
    }
    uint32_t rearmWindowUs = micros() - rearmWindowStartUs;
    if (rearmWindowUs > maxRearmWindowUs) {
      maxRearmWindowUs = rearmWindowUs;
    }

    // Deferred uplink relay send - see the CMD_SEND_UDP handler above for
    // why. The slave is already re-armed by this point, so the actual
    // socket I/O no longer sits inside the window that determines whether
    // the *next* transaction gets missed.
    if (udpSendPending) {
      udpSendPending = false;
      uint32_t udpSendStartUs = micros();
      // Reverted the raw-socket/SO_SNDTIMEO version (2026-08-07) - it
      // introduced audible jitter in normal operation, worse trade than
      // the rare multi-hundred-ms hang it was meant to fix. Back to plain
      // WiFiUDP - the multi-hundred-ms hang during a WiFi failover is
      // currently unmitigated again (NET_SETTLE_MS/resyncSpiSlave() were
      // also reverted the same night, bisecting a separate "transmitted
      // audio sounds corrupted" report that traced to tonight's ESP32
      // changes in general - see git history/memory for the full story).
      udp.beginPacket(udpSendStagingIP, udpSendStagingPort);
      udp.write(udpSendStagingBuf, udpSendStagingLen);
      udp.endPacket();
      uint32_t udpSendDurUs = micros() - udpSendStartUs;
      if (udpSendDurUs > 5000) {
        Serial.printf("[TIMING] udp send (uplink relay) took %lu us\r\n", (unsigned long)udpSendDurUs);
      }
    }

    // Deferred from the gap-detection point above - the slave is already
    // re-armed by now, so this print's UART time no longer eats into the
    // window that determines whether the *next* transaction gets missed.
    if (seqGapPending) {
      Serial.printf("[SEQ] Gap in Teensy->ESP32 SPI sends: expected %u got %u (missing %u, total missing %lu, arriving packet type=%s)\r\n",
                    seqGapExpected, seqGapGot, seqGapMissing, (unsigned long)udpSeqDropCount, seqGapType);
    }
  }
}