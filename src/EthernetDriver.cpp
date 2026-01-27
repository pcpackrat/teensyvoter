#include "EthernetDriver.h"

EthernetDriver::EthernetDriver() {
  _linkStatus = false;
  _targetPort = 0;
  _udp.stop();
}

bool EthernetDriver::begin(uint8_t *mac) {
  _mac = mac;
  Serial.println("[Ethernet] Initializing NativeEthernet...");

  // Check for hardware first (optional but good)
  Serial.println("[Ethernet] Checking for hardware...");
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("[Ethernet] No Hardware Found (Shield missing?)");
    return false;
  }
  Serial.println("[Ethernet] Hardware detected!");

  Serial.println("[Ethernet] Checking link status...");
  if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("[Ethernet] Link status reports OFF.");
    Serial.println("[Ethernet] NOTE: Header-based adapters may not report link "
                   "correctly.");
    Serial.println("[Ethernet] Attempting DHCP anyway (5s timeout)...");
    // Don't return false - try DHCP anyway for header-based adapters
  } else {
    Serial.println("[Ethernet] Link is UP!");
  }

  // Try DHCP with timeout (10 seconds instead of default 60)
  Serial.println("[Ethernet] Requesting DHCP (10s timeout)...");
  if (Ethernet.begin(_mac, 10000) == 0) {
    Serial.println("[Ethernet] DHCP Failed!");
    // Check for link again
    if (Ethernet.linkStatus() != LinkOFF) {
      Serial.println("[Ethernet] Link is UP but DHCP failed. Configuring "
                     "Static IP 192.168.1.177");
      Ethernet.begin(_mac, IPAddress(192, 168, 1, 177));
    } else {
      return false;
    }
  }

  _linkStatus = true;
  Serial.print("[Ethernet] Connected! IP: ");
  Serial.println(Ethernet.localIP());

  _udp.begin(8888); // Local Port
  return true;
}

// Static IP mode
bool EthernetDriver::begin(uint8_t *mac, IPAddress ip, IPAddress subnet,
                           IPAddress gateway, IPAddress dns) {
  _mac = mac;
  Serial.println("[Ethernet] Initializing NativeEthernet (Static IP)...");

  // Check for hardware first
  Serial.println("[Ethernet] Checking for hardware...");
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("[Ethernet] No Hardware Found (Shield missing?)");
    return false;
  }
  Serial.println("[Ethernet] Hardware detected!");

  // Configure static IP
  Serial.print("[Ethernet] Configuring Static IP: ");
  Serial.println(ip);

  // Use DNS if provided, otherwise use gateway
  IPAddress dnsServer = (dns != IPAddress(0, 0, 0, 0)) ? dns : gateway;

  Ethernet.begin(_mac, ip, dnsServer, gateway, subnet);

  _linkStatus = true;
  Serial.print("[Ethernet] Connected! IP: ");
  Serial.println(Ethernet.localIP());
  Serial.printf("[Ethernet] Subnet: %u.%u.%u.%u\r\n", subnet[0], subnet[1],
                subnet[2], subnet[3]);
  Serial.printf("[Ethernet] Gateway: %u.%u.%u.%u\r\n", gateway[0], gateway[1],
                gateway[2], gateway[3]);
  Serial.printf("[Ethernet] DNS: %u.%u.%u.%u\r\n", dnsServer[0], dnsServer[1],
                dnsServer[2], dnsServer[3]);

  _udp.begin(8888); // Local Port
  return true;
}

void EthernetDriver::update() {
  // Maintain DHCP lease
  Ethernet.maintain();

  // Check Link
  // auto status = Ethernet.linkStatus();
  // if(status == LinkOFF && _linkStatus) {
  //    Serial.println("[Ethernet] Link Lost!");
  //    _linkStatus = false;
  // }
  // else if(status == LinkON && !_linkStatus) {
  //     Serial.println("[Ethernet] Link Restored!");
  //    _linkStatus = true;
  // }
}

bool EthernetDriver::isConnected() {
  return (Ethernet.linkStatus() != LinkOFF) &&
         (Ethernet.localIP() != IPAddress(0, 0, 0, 0));
}

IPAddress EthernetDriver::getLocalIP() { return Ethernet.localIP(); }

void EthernetDriver::setTarget(IPAddress ip, uint16_t port) {
  _targetIP = ip;
  _targetPort = port;
}

void EthernetDriver::sendPacket(const uint8_t *data, uint16_t len) {
  if (!isConnected())
    return;

  // NativeEthernet UDP
  if (_udp.beginPacket(_targetIP, _targetPort)) {
    _udp.write(data, len);
    _udp.endPacket();
  }
}

int EthernetDriver::parsePacket() { return _udp.parsePacket(); }

int EthernetDriver::read(uint8_t *buffer, size_t maxLen) {
  return _udp.read(buffer, maxLen);
}

IPAddress EthernetDriver::resolveHostname(const char *hostname) {
  Serial.printf("[Ethernet] Resolving hostname: %s\r\n", hostname);

  // Get DNS server from Ethernet configuration
  IPAddress dnsServer = Ethernet.dnsServerIP();
  if (dnsServer == IPAddress(0, 0, 0, 0)) {
    Serial.println("[Ethernet] No DNS server configured!");
    return IPAddress(0, 0, 0, 0);
  }

  Serial.printf("[Ethernet] Using DNS server: %u.%u.%u.%u\r\n", dnsServer[0],
                dnsServer[1], dnsServer[2], dnsServer[3]);

  // Build DNS query packet
  uint8_t dnsQuery[512];
  uint16_t queryLen = 0;

  // DNS Header (12 bytes)
  dnsQuery[queryLen++] = 0x00;
  dnsQuery[queryLen++] = 0x01; // Transaction ID
  dnsQuery[queryLen++] = 0x01;
  dnsQuery[queryLen++] = 0x00; // Flags: standard query
  dnsQuery[queryLen++] = 0x00;
  dnsQuery[queryLen++] = 0x01; // Questions: 1
  dnsQuery[queryLen++] = 0x00;
  dnsQuery[queryLen++] = 0x00; // Answer RRs: 0
  dnsQuery[queryLen++] = 0x00;
  dnsQuery[queryLen++] = 0x00; // Authority RRs: 0
  dnsQuery[queryLen++] = 0x00;
  dnsQuery[queryLen++] = 0x00; // Additional RRs: 0

  // Question section: convert hostname to DNS format
  // e.g., "voter.example.com" -> 5voter7example3com0
  const char *label = hostname;
  while (*label) {
    const char *dot = strchr(label, '.');
    uint8_t labelLen = dot ? (dot - label) : strlen(label);
    dnsQuery[queryLen++] = labelLen;
    memcpy(&dnsQuery[queryLen], label, labelLen);
    queryLen += labelLen;
    label += labelLen;
    if (dot)
      label++; // Skip the dot
    else
      break;
  }
  dnsQuery[queryLen++] = 0x00; // End of hostname

  // Query type: A (IPv4 address)
  dnsQuery[queryLen++] = 0x00;
  dnsQuery[queryLen++] = 0x01;
  // Query class: IN (Internet)
  dnsQuery[queryLen++] = 0x00;
  dnsQuery[queryLen++] = 0x01;

  // Send DNS query
  EthernetUDP dnsUdp;
  dnsUdp.begin(0); // Use random local port

  if (!dnsUdp.beginPacket(dnsServer, 53)) {
    Serial.println("[Ethernet] Failed to begin DNS packet");
    dnsUdp.stop();
    return IPAddress(0, 0, 0, 0);
  }

  dnsUdp.write(dnsQuery, queryLen);
  dnsUdp.endPacket();

  // Wait for response (timeout: 5 seconds)
  unsigned long startTime = millis();
  while (millis() - startTime < 5000) {
    int packetSize = dnsUdp.parsePacket();
    if (packetSize > 0) {
      uint8_t dnsResponse[512];
      int len = dnsUdp.read(dnsResponse, sizeof(dnsResponse));

      // Parse DNS response
      // Skip header (12 bytes) and question section
      int pos = 12;

      // Skip question section (same format as query)
      while (pos < len && dnsResponse[pos] != 0) {
        pos += dnsResponse[pos] + 1;
      }
      pos += 5; // Skip null terminator + type + class

      // Parse answer section
      if (pos + 12 <= len) {
        // Skip name (usually compressed pointer)
        if ((dnsResponse[pos] & 0xC0) == 0xC0) {
          pos += 2; // Compressed name pointer
        }

        uint16_t type = (dnsResponse[pos] << 8) | dnsResponse[pos + 1];
        pos += 8; // Skip type, class, TTL

        uint16_t dataLen = (dnsResponse[pos] << 8) | dnsResponse[pos + 1];
        pos += 2;

        if (type == 1 && dataLen == 4 && pos + 4 <= len) {
          // Type A (IPv4 address)
          IPAddress resolvedIP(dnsResponse[pos], dnsResponse[pos + 1],
                               dnsResponse[pos + 2], dnsResponse[pos + 3]);

          Serial.printf("[Ethernet] DNS resolved to: %u.%u.%u.%u\r\n",
                        resolvedIP[0], resolvedIP[1], resolvedIP[2],
                        resolvedIP[3]);

          dnsUdp.stop();
          return resolvedIP;
        }
      }
    }
    delay(10);
  }

  Serial.println("[Ethernet] DNS query timeout");
  dnsUdp.stop();
  return IPAddress(0, 0, 0, 0);
}
