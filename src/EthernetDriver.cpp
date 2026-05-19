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

  Serial.println("[Ethernet] Waiting for Link (up to 2s)...");
  for (int i = 0; i < 20; i++) {
    if (Ethernet.linkStatus() != LinkOFF) break;
    delay(100);
  }

  if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("[Ethernet] Link status reports OFF.");
    Serial.println("[Ethernet] NOTE: Header-based adapters may not report link correctly.");
    Serial.println("[Ethernet] Attempting DHCP anyway...");
  } else {
    Serial.println("[Network] ✓ Ethernet Link is UP!");
  }

  // Log MAC address being used
  Serial.printf("[Ethernet] MAC Address: %02X:%02X:%02X:%02X:%02X:%02X\r\n", 
                _mac[0], _mac[1], _mac[2], _mac[3], _mac[4], _mac[5]);

  // Try DHCP with timeout (15 seconds)
  Serial.println("[Ethernet] Requesting DHCP...");
  if (Ethernet.begin(_mac, 15000) == 0) {
    Serial.println("[Ethernet] DHCP Failed!");
    return false;
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

IPAddress EthernetDriver::getSubnetMask() { return Ethernet.subnetMask(); }

IPAddress EthernetDriver::getGateway() { return Ethernet.gatewayIP(); }

IPAddress EthernetDriver::getDNS() { return Ethernet.dnsServerIP(); }

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

void EthernetDriver::sendPacketTo(IPAddress ip, uint16_t port,
                                 const uint8_t *data, uint16_t len) {
  if (!isConnected())
    return;

  // Optimized for NativeEthernet - Avoid redundant calls
  if (_udp.beginPacket(ip, port)) {
    _udp.write(data, len);
    if (!_udp.endPacket()) {
        // Serial.println("[Ethernet] UDP Send Failed");
    }
  } else {
      // Serial.println("[Ethernet] UDP Begin Failed");
  }
}

int EthernetDriver::parsePacket() { return _udp.parsePacket(); }

int EthernetDriver::read(uint8_t *buffer, size_t maxLen) {
  return _udp.read(buffer, maxLen);
}

IPAddress EthernetDriver::resolveHostname(const char *hostname) {
  if (!hostname || hostname[0] == '\0') return IPAddress(0, 0, 0, 0);
  
  // If it's already an IP, return it
  IPAddress result;
  if (result.fromString(hostname)) return result;

  Serial.printf("[Ethernet] Resolving hostname: %s\r\n", hostname);
  
  IPAddress dnsServer = Ethernet.dnsServerIP();
  if (dnsServer == IPAddress(0, 0, 0, 0)) {
    Serial.println("[Ethernet] No DNS server configured");
    return IPAddress(0, 0, 0, 0);
  }

  EthernetUDP dnsUdp;
  dnsUdp.begin(1024 + (millis() % 1000)); // Random local port

  uint8_t packet[512];
  memset(packet, 0, 12);
  packet[1] = 0x01; // ID
  packet[5] = 0x01; // 1 Question
  packet[2] = 0x01; // RD = 1 (Recursion Desired)

  int pos = 12;
  const char *p = hostname;
  while (*p) {
    const char *dot = strchr(p, '.');
    int len = dot ? (dot - p) : strlen(p);
    packet[pos++] = (uint8_t)len;
    memcpy(&packet[pos], p, len);
    pos += len;
    p += len;
    if (dot) p++;
  }
  packet[pos++] = 0; // End of name
  packet[pos++] = 0; packet[pos++] = 1; // Type A
  packet[pos++] = 0; packet[pos++] = 1; // Class IN

  if (dnsUdp.beginPacket(dnsServer, 53)) {
    dnsUdp.write(packet, pos);
    dnsUdp.endPacket();
  }

  uint32_t start = millis();
  while (millis() - start < 3000) {
    _yield(); // Allow GPS and other tasks to run
    int len = dnsUdp.parsePacket();
    if (len >= 12) {
      dnsUdp.read(packet, sizeof(packet)); // Actually read the data!
      if (packet[0] == 0x00 && packet[1] == 0x01) { // Check ID
        int qCount = (packet[4] << 8) | packet[5];
        int aCount = (packet[6] << 8) | packet[7];
        
        int cur = 12;
      // Skip Questions with bounds safety
      for (int i = 0; i < qCount && cur < len - 5; i++) {
        while (cur < len && packet[cur] != 0) {
          if ((packet[cur] & 0xC0) == 0xC0) { cur += 2; break; }
          cur += packet[cur] + 1;
        }
        if (cur < len && packet[cur] == 0) cur++;
        cur += 4; // type + class
      }
      
      // Parse Answers with bounds safety
      for (int i = 0; i < aCount && cur < len - 10; i++) {
        // Skip name
        if ((packet[cur] & 0xC0) == 0xC0) cur += 2;
        else {
            while (cur < len && packet[cur] != 0) cur += packet[cur] + 1; 
            if (cur < len) cur++;
        }
        
        if (cur > len - 10) break;
        uint16_t type = (packet[cur] << 8) | packet[cur+1];
        uint16_t rdlen = (packet[cur+8] << 8) | packet[cur+9];
        cur += 10;
        
        if (type == 1 && rdlen == 4 && cur <= len - 4) { // Type A
          IPAddress resolved(packet[cur], packet[cur+1], packet[cur+2], packet[cur+3]);
          dnsUdp.stop();
          Serial.printf("[Ethernet] Resolved to: %u.%u.%u.%u\r\n", resolved[0], resolved[1], resolved[2], resolved[3]);
          return resolved;
        }
        cur += rdlen;
      }
      }
    }
    delay(10);
  }

  dnsUdp.stop();
  Serial.println("[Ethernet] DNS Timeout/Failure");
  return IPAddress(0, 0, 0, 0);
}
