#include "WebServer.h"
#include <Audio.h>

// HTML stored in PROGMEM to save RAM
const char HTML_STYLE[] PROGMEM = R"rawliteral(
<style>
body{font-family:Arial,sans-serif;margin:0;padding:20px;background:#2c3e50;color:#ecf0f1}
.container{max-width:800px;margin:0 auto}
.card{background:#34495e;padding:20px;margin:10px 0;border-radius:8px}
h1{color:#3498db;margin-top:0}
h2{color:#ecf0f1;border-bottom:2px solid #3498db;padding-bottom:10px}
.status{display:inline-block;padding:5px 10px;border-radius:4px;margin:5px}
.status.ok{background:#27ae60;color:#fff}
.status.error{background:#e74c3c;color:#fff}
.btn{background:#3498db;color:#fff;border:none;padding:10px 20px;border-radius:4px;cursor:pointer;font-size:16px;margin:5px}
.btn:hover{background:#2980b9}
  label {
    display: block;
    margin: 15px 0 5px 0;
    font-weight: bold;
    color: #bdc3c7;
  }
  label.checkbox {
    display: flex;
    align-items: center;
    font-weight: normal;
    margin: 10px 0;
    color: #ecf0f1;
    cursor: pointer;
  }
  label.checkbox input[type="checkbox"] {
    width: auto;
    margin: 0 10px 0 0;
  }
  input[type="number"], input[type="text"], input[type="password"], select {
    width: 100%;
    padding: 10px;
    margin: 5px 0;
    border: 1px solid #7f8c8d;
    border-radius: 4px;
    box-sizing: border-box;
    background: #ecf0f1;
    color: #2c3e50;
    font-size: 14px;
  }
  h3 {
    color: #3498db;
    border-bottom: 1px solid #7f8c8d;
    padding-bottom: 5px;
    margin-top: 25px;
  }
  p small {
    color: #95a5a6;
    display: block;
    margin-top: -5px;
    margin-bottom: 15px;
  }
.nav a{color:#3498db;text-decoration:none;margin:0 15px;font-weight:bold}
.nav a:hover{color:#ecf0f1}
</style>
)rawliteral";

const char HTML_NAV[] PROGMEM = R"rawliteral(
<div class="nav">
<a href="/">Dashboard</a>
<a href="/network">Network</a>
<a href="/voter">Voter</a>
<a href="/audio">Audio</a>
<a href="/api/export">Export</a>
<a href="/import">Import</a>
<a href="/system">System</a>
</div>
)rawliteral";

WebServer::WebServer()
    : _server(80), _config(nullptr), _netMgr(nullptr), _voter(nullptr),
      _gps(nullptr) {}

void WebServer::begin() {
  _server.begin();
  Serial.println("[Web] Server started on port 80");
}

void WebServer::setConfig(ConfigManager *cfg) { _config = cfg; }

void WebServer::setSystemObjects(NetworkManager *netMgr, VoterClient *voter,
                                 GPSManager *gps) {
  _netMgr = netMgr;
  _voter = voter;
  _gps = gps;
}

void WebServer::handleClient() {
  EthernetClient client = _server.available();
  if (client) {
    Serial.println("[Web] New client connected");
    handleRequest(client);
    client.stop();
    Serial.println("[Web] Client disconnected");
  }
}

void WebServer::handleRequest(EthernetClient &client) {
  // Read request line
  String requestLine = readRequestLine(client);
  if (requestLine.length() == 0) {
    return;
  }

  // Read headers and look for Content-Length
  int contentLength = 0;
  while (client.available()) {
    String line = client.readStringUntil('\n');
    line.trim();
    if (line.length() == 0)
      break; // Empty line = end of headers

    // Check for Content-Length header
    if (line.startsWith("Content-Length:")) {
      contentLength = line.substring(15).toInt();
    }
  }

  // Parse request
  String method, path;
  parseRequest(requestLine, method, path);

  Serial.printf("[Web] %s %s\n", method.c_str(), path.c_str());

  // Read POST body if present
  String body = "";
  if (method == "POST" && contentLength > 0) {
    body = readRequestBody(client, contentLength);
  }

  // Route to handlers
  if (method == "GET") {
    if (path == "/" || path == "/index.html") {
      handleRoot(client);
    } else if (path == "/network") {
      handleNetwork(client);
    } else if (path == "/voter") {
      handleVoter(client);
    } else if (path == "/audio") {
      handleAudio(client);
    } else if (path == "/system") {
      handleSystem(client);
    } else if (path == "/api/status") {
      handleApiStatus(client);
    } else if (path == "/api/export") {
      handleExport(client);
    } else if (path == "/import") {
      handleImport(client);
    } else {
      handleNotFound(client);
    }
  } else if (method == "POST") {
    if (path == "/api/config/network") {
      handleNetworkPost(client, body);
    } else if (path == "/api/config/voter") {
      handleVoterPost(client, body);
    } else if (path == "/api/config/audio") {
      handleAudioPost(client, body);
    } else if (path == "/api/import") {
      handleImportPost(client, body);
    } else {
      handleNotFound(client);
    }
  } else {
    handleNotFound(client);
  }
}

String WebServer::readRequestLine(EthernetClient &client) {
  String line = "";
  unsigned long timeout = millis() + 1000;

  while (client.connected() && millis() < timeout) {
    if (client.available()) {
      char c = client.read();
      if (c == '\n') {
        break;
      } else if (c != '\r') {
        line += c;
      }
    }
  }

  return line;
}

void WebServer::parseRequest(const String &request, String &method,
                             String &path) {
  int firstSpace = request.indexOf(' ');
  int secondSpace = request.indexOf(' ', firstSpace + 1);

  if (firstSpace > 0 && secondSpace > firstSpace) {
    method = request.substring(0, firstSpace);
    path = request.substring(firstSpace + 1, secondSpace);
  }
}

void WebServer::sendHeader(EthernetClient &client, int code,
                           const char *contentType) {
  client.print("HTTP/1.1 ");
  client.print(code);
  client.println(code == 200 ? " OK" : " Not Found");
  client.print("Content-Type: ");
  client.println(contentType);
  client.println("Connection: close");
  client.println();
}

void WebServer::sendHtmlHeader(EthernetClient &client) {
  sendHeader(client, 200, "text/html");
  client.println("<!DOCTYPE html><html><head>");
  client.println("<meta name=\"viewport\" "
                 "content=\"width=device-width,initial-scale=1\">");
  client.println("<title>TeensyVoter</title>");
  client.print(reinterpret_cast<const __FlashStringHelper *>(HTML_STYLE));
  client.println("</head><body><div class=\"container\">");
  client.println("<h1>TeensyVoter Configuration</h1>");
  client.print(reinterpret_cast<const __FlashStringHelper *>(HTML_NAV));
}

void WebServer::sendHtmlFooter(EthernetClient &client) {
  client.println("</div></body></html>");
}

void WebServer::handleRoot(EthernetClient &client) {
  sendHtmlHeader(client);

  client.println("<div class=\"card\">");
  client.println("<h2>System Status</h2>");
  client.println("<div id=\"status\">Loading...</div>");
  client.println("</div>");

  // JavaScript for AJAX updates
  client.println("<script>");
  client.println("function updateStatus() {");
  client.println("  fetch('/api/status')");
  client.println("    .then(r => r.json())");
  client.println("    .then(data => {");
  client.println("      let html = '<p><strong>Network:</strong> ' + "
                 "data.network.type + '</p>';");
  client.println("      html += '<p><strong>IP Address:</strong> ' + "
                 "data.network.ip + '</p>';");
  client.println("      html += '<p><strong>Connection:</strong> <span "
                 "class=\"status ' + (data.network.connected ? 'ok' : 'error') "
                 "+ '\">' + (data.network.connected ? 'CONNECTED' : "
                 "'DISCONNECTED') + '</span></p>';");
  client.println("      html += '<hr>';");
  client.println("      html += '<p><strong>Voter Host:</strong> ' + "
                 "data.voter.host + ':' + data.voter.port + '</p>';");
  client.println(
      "      html += '<p><strong>Voter Status:</strong> <span class=\"status ' "
      "+ (data.voter.connected ? 'ok' : 'error') + '\">' + "
      "(data.voter.connected ? 'CONNECTED' : 'DISCONNECTED') + '</span></p>';");
  client.println("      html += '<hr>';");
  client.println(
      "      html += '<p><strong>RSSI:</strong> ' + data.audio.rssi + '</p>';");
  client.println("      html += '<p><strong>Audio Peak:</strong> ' + "
                 "data.audio.peak.toFixed(2) + '</p>';");
  client.println("      html += '<p><strong>COS:</strong> ' + (data.audio.cos "
                 "? 'ACTIVE' : 'INACTIVE') + '</p>';");
  client.println("      html += '<hr>';");
  client.println("      html += '<p><strong>GPS Locked:</strong> ' + "
                 "(data.gps.locked ? 'YES' : 'NO') + '</p>';");
  client.println("      html += '<p><strong>Satellites:</strong> ' + "
                 "data.gps.satellites + '</p>';");
  client.println("      html += '<p><strong>GPS Time:</strong> ' + "
                 "data.gps.time + '</p>';");
  client.println("      html += '<hr>';");
  client.println(
      "      html += '<p><strong>Uptime:</strong> ' + Math.floor(data.uptime / "
      "3600) + 'h ' + Math.floor((data.uptime % 3600) / 60) + 'm ' + "
      "(data.uptime % 60) + 's</p>';");
  client.println("      document.getElementById('status').innerHTML = html;");
  client.println("    })");
  client.println("    .catch(e => document.getElementById('status').innerHTML "
                 "= 'Error loading status');");
  client.println("}");
  client.println("updateStatus();");
  client.println("setInterval(updateStatus, 2000);"); // Update every 2 seconds
  client.println("</script>");

  sendHtmlFooter(client);
}

void WebServer::handleNetwork(EthernetClient &client) {
  sendHtmlHeader(client);

  client.println("<div class=\"card\">");
  client.println("<h2>Network Configuration</h2>");
  client.println("<form method=\"POST\" action=\"/api/config/network\">");

  // DHCP vs Static IP
  client.println("<label>Network Mode:</label>");
  client.println("<select name=\"mode\">");
  client.print("<option value=\"dhcp\"");
  if (!_config->data.useStaticIP)
    client.print(" selected");
  client.println(">DHCP</option>");
  client.print("<option value=\"static\"");
  if (_config->data.useStaticIP)
    client.print(" selected");
  client.println(">Static IP</option>");
  client.println("</select>");

  // Static IP Address
  client.println("<label>IP Address:</label>");
  client.print("<input type=\"text\" name=\"ip\" value=\"");
  IPAddress staticIP(_config->data.staticIP);
  client.printf("%u.%u.%u.%u", staticIP[0], staticIP[1], staticIP[2],
                staticIP[3]);
  client.println("\">");

  // Subnet Mask
  client.println("<label>Subnet Mask:</label>");
  client.print("<input type=\"text\" name=\"subnet\" value=\"");
  IPAddress subnet(_config->data.subnetMask);
  client.printf("%u.%u.%u.%u", subnet[0], subnet[1], subnet[2], subnet[3]);
  client.println("\">");

  // Gateway
  client.println("<label>Gateway:</label>");
  client.print("<input type=\"text\" name=\"gateway\" value=\"");
  IPAddress gw(_config->data.gateway);
  client.printf("%u.%u.%u.%u", gw[0], gw[1], gw[2], gw[3]);
  client.println("\">");

  // DNS Server
  client.println("<label>DNS Server:</label>");
  client.print("<input type=\"text\" name=\"dns\" value=\"");
  IPAddress dns(_config->data.staticDNS);
  client.printf("%u.%u.%u.%u", dns[0], dns[1], dns[2], dns[3]);
  client.println("\">");

  // Hostname
  client.println("<label>Hostname (optional):</label>");
  client.print("<input type=\"text\" name=\"hostname\" value=\"");
  client.print(_config->data.hostname);
  client.println("\" placeholder=\"voter.example.com\">");

  client.println(
      "<button type=\"submit\" class=\"btn\">Save & Reboot</button>");
  client.println("</form>");
  client.println(
      "<p><small>Note: Changes require reboot to take effect</small></p>");
  client.println("</div>");

  sendHtmlFooter(client);
}

void WebServer::handleVoter(EthernetClient &client) {
  sendHtmlHeader(client);

  client.println("<div class=\"card\">");
  client.println("<h2>Voter Configuration</h2>");
  client.println("<form method=\"POST\" action=\"/api/config/voter\">");

  // Host Address (IP or hostname)
  client.println("<label>Host Address (IP or hostname):</label>");
  client.print("<input type=\"text\" name=\"host\" value=\"");
  if (_config->data.hostname[0] != '\0') {
    client.print(_config->data.hostname);
  } else {
    IPAddress hostIP(_config->data.hostIP);
    client.printf("%u.%u.%u.%u", hostIP[0], hostIP[1], hostIP[2], hostIP[3]);
  }
  client.println("\">");

  // Port
  client.println("<label>Port:</label>");
  client.print("<input type=\"number\" name=\"port\" value=\"");
  client.print(_config->data.hostPort);
  client.println("\">");

  // Client Password
  client.println("<label>Client Password:</label>");
  client.print("<input type=\"password\" name=\"clientpwd\" value=\"");
  client.print(_config->data.clientPwd);
  client.println("\">");

  // Host Password
  client.println("<label>Host Password:</label>");
  client.print("<input type=\"password\" name=\"hostpwd\" value=\"");
  client.print(_config->data.hostPwd);
  client.println("\">");

  client.println(
      "<button type=\"submit\" class=\"btn\">Save & Reboot</button>");
  client.println("</form>");
  client.println(
      "<p><small>Note: Changes require reboot to take effect</small></p>");
  client.println("</div>");

  sendHtmlFooter(client);
}

void WebServer::handleAudio(EthernetClient &client) {
  sendHtmlHeader(client);

  client.println("<div class=\"card\">");
  client.println("<h2>Radio & Audio Configuration</h2>");
  client.println("<form method=\"POST\" action=\"/api/config/audio\">");

  // RX Gain (Hardware)
  client.println("<h3>Hardware Gain</h3>");
  client.println("<label>RX Gain (0-15):</label>");
  client.print(
      "<input type=\"number\" name=\"rxgain\" min=\"0\" max=\"15\" value=\"");
  client.print(_config->data.rxGain);
  client.println("\">");
  client.println("<p><small>SGTL5000 Line Input Level</small></p>");

  // RSSI Calibration (DSP)
  client.println("<label>RSSI Calibration Factor (DSP):</label>");
  client.print("<input type=\"number\" name=\"dspgain\" min=\"0\" max=\"50\" "
               "step=\"0.1\" value=\"");
  client.print(_config->data.dspCalib);
  client.println("\">");
  client.println(
      "<p><small>Adjusts RSSI sensitivity (Default: 13.0)</small></p>");

  // Input Source
  client.println("<label>Input Source:</label>");
  client.println("<select name=\"inputsource\">");
  client.print("<option value=\"0\"");
  if (_config->data.inputSource == 0)
    client.print(" selected");
  client.println(">Line In</option>");
  client.print("<option value=\"1\"");
  if (_config->data.inputSource == 1)
    client.print(" selected");
  client.println(">Microphone</option>");
  client.println("</select>");

  // COS Configuration
  client.println("<h3>COS (Carrier Operated Squelch)</h3>");
  client.println("<label>COS Mode:</label>");
  client.println("<select name=\"cosmode\">");
  client.print("<option value=\"0\"");
  if (_config->data.cosMode == 0)
    client.print(" selected");
  client.println(">Always On</option>");
  client.print("<option value=\"1\"");
  if (_config->data.cosMode == 1)
    client.print(" selected");
  client.println(">Hardware</option>");
  client.print("<option value=\"2\"");
  if (_config->data.cosMode == 2)
    client.print(" selected");
  client.println(">DSP</option>");
  client.println("</select>");

  client.println("<label class=\"checkbox\">");
  client.print("<input type=\"checkbox\" name=\"cosinvert\" value=\"1\"");
  if (_config->data.cosInvert)
    client.print(" checked");
  client.println(">Invert COS Polarity</label>");

  // DSP Squelch (for DSP COS mode)
  client.println("<label>DSP Squelch Threshold (0-255):</label>");
  client.print(
      "<input type=\"number\" name=\"squelch\" min=\"0\" max=\"255\" value=\"");
  client.print(_config->data.dspSquelchThresh);
  client.println("\">");
  client.println("<p><small>Used when COS Mode = DSP</small></p>");

  // RSSI Configuration
  client.println("<h3>RSSI</h3>");
  client.println("<label class=\"checkbox\">");
  client.print("<input type=\"checkbox\" name=\"hwrssi\" value=\"1\"");
  if (_config->data.useHwRSSI)
    client.print(" checked");
  client.println(">Use Hardware RSSI</label>");

  client.println("<label>RSSI Min (ADC):</label>");
  client.print("<input type=\"number\" name=\"rssimin\" min=\"0\" max=\"1023\" "
               "value=\"");
  client.print(_config->data.rssiMin);
  client.println("\">");

  client.println("<label>RSSI Max (ADC):</label>");
  client.print("<input type=\"number\" name=\"rssimax\" min=\"0\" max=\"1023\" "
               "value=\"");
  client.print(_config->data.rssiMax);
  client.println("\">");
  client.println(
      "<p><small>Calibration range for analog RSSI input</small></p>");

  // Audio Filtering
  client.println("<h3>Audio Filtering</h3>");
  client.println("<label class=\"checkbox\">");
  client.print("<input type=\"checkbox\" name=\"plfilter\" value=\"1\"");
  if (_config->data.enablePLFilter)
    client.print(" checked");
  client.println(">Enable PL Filter (300Hz HPF)</label>");

  client.println("<label class=\"checkbox\">");
  client.print("<input type=\"checkbox\" name=\"deemph\" value=\"1\"");
  if (_config->data.enableDeemp)
    client.print(" checked");
  client.println(">Enable De-Emphasis</label>");

  client.println("<button type=\"submit\" class=\"btn\">Save & Apply</button>");
  client.println("</form>");
  client.println("</div>");

  sendHtmlFooter(client);
}

void WebServer::handleSystem(EthernetClient &client) {
  sendHtmlHeader(client);

  client.println("<div class=\"card\">");
  client.println("<h2>System</h2>");
  client.println("<p>System page - Coming soon</p>");
  client.println("</div>");

  sendHtmlFooter(client);
}

void WebServer::handleApiStatus(EthernetClient &client) {
  sendHeader(client, 200, "application/json");

  // Get network info
  IPAddress ip = Ethernet.localIP();
  String ipStr = String(ip[0]) + "." + String(ip[1]) + "." + String(ip[2]) +
                 "." + String(ip[3]);
  const char *netType =
      (_netMgr && _netMgr->getType() == DRIVER_ETHERNET) ? "Ethernet" : "WiFi";
  bool netConnected = _netMgr ? _netMgr->isConnected() : false;

  // Get voter info
  IPAddress hostIP(_config ? _config->data.hostIP : 0);
  String hostStr;
  if (_config && _config->data.hostname[0] != '\0') {
    hostStr = String(_config->data.hostname);
  } else {
    hostStr = String(hostIP[0]) + "." + String(hostIP[1]) + "." +
              String(hostIP[2]) + "." + String(hostIP[3]);
  }
  uint16_t hostPort = _config ? _config->data.hostPort : 0;
  bool voterConnected = _voter ? _voter->isConnected() : false;

  // Get GPS info
  bool gpsLocked = _gps ? _gps->isLocked() : false;
  int gpsSats = _gps ? _gps->getSatellites() : 0;
  String gpsTime = "N/A";
  if (_gps && gpsLocked) {
    uint32_t epoch = _gps->getEpoch();
    // Simple time formatting from epoch
    uint32_t hours = (epoch / 3600) % 24;
    uint32_t minutes = (epoch / 60) % 60;
    uint32_t seconds = epoch % 60;
    char timeStr[16];
    snprintf(timeStr, sizeof(timeStr), "%02lu:%02lu:%02lu UTC", hours, minutes,
             seconds);
    gpsTime = String(timeStr);
  }

  // Get audio/RSSI info (access global objects from main.cpp)
  extern AudioAnalyzePeak peak1;

  int rssiRaw = analogRead(A14);                 // RSSI_PIN from main.cpp
  int rssiValue = map(rssiRaw, 0, 1023, 0, 255); // Map to 0-255 range
  float audioPeak = 0.0;
  if (peak1.available()) {
    audioPeak = peak1.read();
  }
  bool cosActive = false; // TODO: Add COS status if needed

  // Build JSON response
  client.println("{");
  client.println("  \"network\": {");
  client.print("    \"type\": \"");
  client.print(netType);
  client.println("\",");
  client.print("    \"ip\": \"");
  client.print(ipStr);
  client.println("\",");
  client.print("    \"connected\": ");
  client.println(netConnected ? "true" : "false");
  client.println("  },");
  client.println("  \"voter\": {");
  client.print("    \"host\": \"");
  client.print(hostStr);
  client.println("\",");
  client.print("    \"port\": ");
  client.print(hostPort);
  client.println(",");
  client.print("    \"connected\": ");
  client.println(voterConnected ? "true" : "false");
  client.println("  },");
  client.println("  \"audio\": {");
  client.print("    \"rssi\": ");
  client.print(rssiValue);
  client.println(",");
  client.print("    \"peak\": ");
  client.print(audioPeak, 3);
  client.println(",");
  client.print("    \"cos\": ");
  client.println(cosActive ? "true" : "false");
  client.println("  },");
  client.println("  \"gps\": {");
  client.print("    \"locked\": ");
  client.print(gpsLocked ? "true" : "false");
  client.println(",");
  client.print("    \"satellites\": ");
  client.print(gpsSats);
  client.println(",");
  client.print("    \"time\": \"");
  client.print(gpsTime);
  client.println("\"");
  client.println("  },");
  client.print("  \"uptime\": ");
  client.println(millis() / 1000);
  client.println("}");
}

void WebServer::handleNotFound(EthernetClient &client) {
  sendHeader(client, 404, "text/html");
  client.println("<html><body><h1>404 Not Found</h1></body></html>");
}

// Utility Functions

String WebServer::readRequestBody(EthernetClient &client, int contentLength) {
  String body = "";
  body.reserve(contentLength);

  unsigned long timeout = millis() + 2000;
  while (body.length() < contentLength && millis() < timeout) {
    if (client.available()) {
      body += (char)client.read();
    }
  }

  return body;
}

String WebServer::getFormValue(const String &body, const String &key) {
  // Find key in URL-encoded form data
  String searchKey = key + "=";
  int startIdx = body.indexOf(searchKey);

  if (startIdx == -1) {
    return ""; // Key not found
  }

  startIdx += searchKey.length();
  int endIdx = body.indexOf('&', startIdx);

  if (endIdx == -1) {
    endIdx = body.length();
  }

  String value = body.substring(startIdx, endIdx);

  // URL decode (basic - handle %20 for spaces, etc.)
  value.replace("+", " ");
  value.replace("%20", " ");
  value.replace("%2B", "+");
  value.replace("%3D", "=");

  return value;
}

bool WebServer::parseIPAddress(const String &str, uint32_t &ip) {
  IPAddress addr;
  if (addr.fromString(str)) {
    ip = (uint32_t)addr;
    return true;
  }
  return false;
}

void WebServer::sendRedirect(EthernetClient &client, const String &location) {
  client.println("HTTP/1.1 302 Found");
  client.print("Location: ");
  client.println(location);
  client.println("Connection: close");
  client.println();
}

void WebServer::sendJsonResponse(EthernetClient &client, bool success,
                                 const String &message) {
  sendHeader(client, 200, "application/json");
  client.println("{");
  client.print("  \"success\": ");
  client.println(success ? "true" : "false");
  client.print("  \"message\": \"");
  client.print(message);
  client.println("\"");
  client.println("}");
}

// POST Handlers

void WebServer::handleNetworkPost(EthernetClient &client, const String &body) {
  Serial.println("[Web] Processing network config POST");

  // Parse form data
  String mode = getFormValue(body, "mode");
  String ipStr = getFormValue(body, "ip");
  String subnetStr = getFormValue(body, "subnet");
  String gatewayStr = getFormValue(body, "gateway");
  String dnsStr = getFormValue(body, "dns");
  String hostname = getFormValue(body, "hostname");

  // Update config
  _config->data.useStaticIP = (mode == "static");

  if (!parseIPAddress(ipStr, _config->data.staticIP)) {
    sendJsonResponse(client, false, "Invalid IP address");
    return;
  }

  if (!parseIPAddress(subnetStr, _config->data.subnetMask)) {
    sendJsonResponse(client, false, "Invalid subnet mask");
    return;
  }

  if (!parseIPAddress(gatewayStr, _config->data.gateway)) {
    sendJsonResponse(client, false, "Invalid gateway");
    return;
  }

  if (!parseIPAddress(dnsStr, _config->data.staticDNS)) {
    sendJsonResponse(client, false, "Invalid DNS server");
    return;
  }

  hostname.toCharArray(_config->data.hostname, sizeof(_config->data.hostname));

  // Save to EEPROM
  _config->save();

  Serial.println("[Web] Network config saved, rebooting...");

  // Send response
  sendJsonResponse(client, true, "Configuration saved. Rebooting...");

  delay(100); // Let response send

  // Reboot Teensy
  SCB_AIRCR = 0x05FA0004;
}

void WebServer::handleVoterPost(EthernetClient &client, const String &body) {
  Serial.println("[Web] Processing voter config POST");

  // Parse form data
  String host = getFormValue(body, "host");
  String portStr = getFormValue(body, "port");
  String clientPwd = getFormValue(body, "clientpwd");
  String hostPwd = getFormValue(body, "hostpwd");

  // Update config
  host.toCharArray(_config->data.hostname, sizeof(_config->data.hostname));
  _config->data.hostPort = portStr.toInt();
  clientPwd.toCharArray(_config->data.clientPwd,
                        sizeof(_config->data.clientPwd));
  hostPwd.toCharArray(_config->data.hostPwd, sizeof(_config->data.hostPwd));

  // Try to parse as IP, otherwise leave as hostname
  uint32_t ip;
  if (parseIPAddress(host, ip)) {
    _config->data.hostIP = ip;
  }

  // Save to EEPROM
  _config->save();

  Serial.println("[Web] Voter config saved, rebooting...");

  // Send response
  sendJsonResponse(client, true, "Configuration saved. Rebooting...");

  delay(100);

  // Reboot
  SCB_AIRCR = 0x05FA0004;
}

void WebServer::handleAudioPost(EthernetClient &client, const String &body) {
  Serial.println("[Web] Processing audio config POST");

  // Parse form data
  String rxGainStr = getFormValue(body, "rxgain");
  String dspGainStr = getFormValue(body, "dspgain");
  String inputSourceStr = getFormValue(body, "inputsource");
  String cosModeStr = getFormValue(body, "cosmode");
  String squelchStr = getFormValue(body, "squelch");
  String rssiMinStr = getFormValue(body, "rssimin");
  String rssiMaxStr = getFormValue(body, "rssimax");

  // Checkboxes (only present if checked)
  String cosInvert = getFormValue(body, "cosinvert");
  String hwRssi = getFormValue(body, "hwrssi");
  String plFilter = getFormValue(body, "plfilter");
  String deemph = getFormValue(body, "deemph");

  // Update config
  _config->data.rxGain = rxGainStr.toInt();
  _config->data.dspCalib = dspGainStr.toFloat();
  _config->data.inputSource = inputSourceStr.toInt();
  _config->data.cosMode = cosModeStr.toInt();
  _config->data.dspSquelchThresh = squelchStr.toInt();
  _config->data.rssiMin = rssiMinStr.toInt();
  _config->data.rssiMax = rssiMaxStr.toInt();

  _config->data.cosInvert = (cosInvert == "1");
  _config->data.useHwRSSI = (hwRssi == "1");
  _config->data.enablePLFilter = (plFilter == "1");
  _config->data.enableDeemp = (deemph == "1");

  // Save to EEPROM
  _config->save();

  Serial.println("[Web] Audio config saved");

  // Send redirect back to audio page
  sendRedirect(client, "/audio");
}

// Export/Import Handlers

void WebServer::handleExport(EthernetClient &client) {
  sendHeader(client, 200, "application/json");
  client.println(
      "Content-Disposition: attachment; filename=\"voter_config.json\"");
  client.println();

  // Generate JSON export
  client.println("{");
  client.println("  \"version\": \"1.0\",");

  // Network config
  client.println("  \"network\": {");
  client.print("    \"useStaticIP\": ");
  client.println(_config->data.useStaticIP ? "true," : "false,");

  IPAddress staticIP(_config->data.staticIP);
  client.printf("    \"staticIP\": \"%u.%u.%u.%u\",\n", staticIP[0],
                staticIP[1], staticIP[2], staticIP[3]);

  IPAddress subnet(_config->data.subnetMask);
  client.printf("    \"subnetMask\": \"%u.%u.%u.%u\",\n", subnet[0], subnet[1],
                subnet[2], subnet[3]);

  IPAddress gw(_config->data.gateway);
  client.printf("    \"gateway\": \"%u.%u.%u.%u\",\n", gw[0], gw[1], gw[2],
                gw[3]);

  IPAddress dns(_config->data.staticDNS);
  client.printf("    \"staticDNS\": \"%u.%u.%u.%u\",\n", dns[0], dns[1], dns[2],
                dns[3]);

  client.print("    \"hostname\": \"");
  client.print(_config->data.hostname);
  client.println("\"");
  client.println("  },");

  // Voter config
  client.println("  \"voter\": {");
  IPAddress hostIP(_config->data.hostIP);
  client.printf("    \"hostIP\": \"%u.%u.%u.%u\",\n", hostIP[0], hostIP[1],
                hostIP[2], hostIP[3]);
  client.print("    \"hostPort\": ");
  client.println(_config->data.hostPort);
  client.println("  },");

  // Audio config
  client.println("  \"audio\": {");
  client.print("    \"rxGain\": ");
  client.println(_config->data.rxGain);
  client.println(",");
  client.print("    \"dspCalib\": ");
  client.println(_config->data.dspCalib);
  client.println(",");
  client.print("    \"inputSource\": ");
  client.println(_config->data.inputSource);
  client.println(",");
  client.print("    \"cosMode\": ");
  client.println(_config->data.cosMode);
  client.println(",");
  client.print("    \"cosInvert\": ");
  client.println(_config->data.cosInvert ? "true," : "false,");
  client.print("    \"dspSquelchThresh\": ");
  client.println(_config->data.dspSquelchThresh);
  client.println(",");
  client.print("    \"useHwRSSI\": ");
  client.println(_config->data.useHwRSSI ? "true," : "false,");
  client.print("    \"rssiMin\": ");
  client.println(_config->data.rssiMin);
  client.println(",");
  client.print("    \"rssiMax\": ");
  client.println(_config->data.rssiMax);
  client.println(",");
  client.print("    \"enablePLFilter\": ");
  client.println(_config->data.enablePLFilter ? "true," : "false,");
  client.print("    \"enableDeemp\": ");
  client.println(_config->data.enableDeemp ? "true" : "false");
  client.println("  }");

  client.println("}");
}

void WebServer::handleImport(EthernetClient &client) {
  sendHtmlHeader(client);

  client.println("<div class=\"card\">");
  client.println("<h2>Import Configuration</h2>");
  client.println("<p>Upload a previously exported configuration JSON file to "
                 "restore settings.</p>");
  client.println("<form method=\"POST\" action=\"/api/import\" "
                 "enctype=\"multipart/form-data\">");
  client.println("<label>Configuration File:</label>");
  client.println(
      "<input type=\"file\" name=\"config\" accept=\".json\" required>");
  client.println(
      "<button type=\"submit\" class=\"btn\">Import & Reboot</button>");
  client.println("</form>");
  client.println("</div>");

  sendHtmlFooter(client);
}

void WebServer::handleImportPost(EthernetClient &client, const String &body) {
  // TODO: Implement JSON parsing and config restoration
  // This is complex due to multipart/form-data parsing
  // For now, send not implemented response
  sendJsonResponse(client, false, "Import not yet implemented");
}
