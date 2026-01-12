#include "WebInterface.h"

// Minimal SCB AIRCR define to perform software reset without pulling in CMSIS
#ifndef SCB_AIRCR
#define SCB_AIRCR (*((volatile uint32_t*)0xE000ED0C))
#endif

WebInterface::WebInterface() {
    _server = new EthernetServer(80);
}

void WebInterface::begin(ConfigManager* cfg, GPSManager* gps, VoterClient* voter) {
    _cfg = cfg;
    _gps = gps;
    _voter = voter;
    _server->begin();
}

void WebInterface::update() {
    EthernetClient client = _server->available();
    if (client) {
        // Read Request Buffer
        String request = "";
        while (client.connected()) {
            if (client.available()) {
                char c = client.read();
                request += c;
                if (c == '\n' && request.endsWith("\r\n\r\n")) {
                    break;
                }
            }
        }
        
        // Parse GET/POST
        if (request.length() > 0) {
            if (request.indexOf("POST /save") >= 0) {
                // Parse Body
                String body = "";
                while (client.available()) {
                    body += (char)client.read();
                }
                _parseParams(body);
                // Redirect home
                client.println("HTTP/1.1 303 See Other");
                client.println("Location: /");
                client.println();
            }
            else {
                // Serve Page
                _handleRequest(client);
            }
        }
        client.stop();
    }
}

void WebInterface::_handleRequest(EthernetClient& client) {
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/html");
    client.println("Connection: close");
    client.println();

    client.println(F("<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width, initial-scale=1'><title>TeensyVoter</title><style>"));
    client.println(F("body{font-family:sans-serif;margin:0;padding:20px;background:#f0f2f5}.card{background:#fff;padding:20px;margin-bottom:20px;border-radius:8px;box-shadow:0 2px 4px rgba(0,0,0,0.1)}"));
    client.println(F("h2{margin-top:0}input{width:100%;padding:8px;margin:5px 0 15px;border:1px solid #ccc;border-radius:4px}.btn{background:#007bff;color:#fff;padding:10px 20px;border:none;border-radius:4px;cursor:pointer}.btn:hover{background:#0056b3}"));
    client.println(F(".stat{font-weight:bold;color:#333}.ok{color:green}.bad{color:red}"));
    client.println(F("</style></head><body>"));

    // Status Card
    client.println(F("<div class='card'><h2>Status</h2>"));
    
    // GPS
    client.print(F("GPS: <span class='stat'>"));
    if (_gps->isLocked()) client.print(F("<span class='ok'>LOCKED</span>"));
    else client.print(F("<span class='bad'>SEARCHING</span>"));
    client.println(F("</span><br>"));

    // Voter
    client.print(F("Host: <span class='stat'>"));
    if (_voter->isConnected()) client.print(F("<span class='ok'>CONNECTED</span>"));
    else client.print(F("<span class='bad'>DISCONNECTED</span>"));
    client.println(F("</span><br>"));
    
    // Config
    IPAddress ip = Ethernet.localIP();
    client.printf("Device IP: %d.%d.%d.%d<br>", ip[0], ip[1], ip[2], ip[3]);
    client.println(F("</div>"));

    // Config Card
    client.println(F("<div class='card'><h2>Settings</h2><form action='/save' method='POST'>"));
    
    // Host IP
    IPAddress host = _cfg->getHostIP();
    client.println(F("Host IP:<br><input name='ip' value='"));
    client.printf("%d.%d.%d.%d", host[0], host[1], host[2], host[3]);
    client.println(F("' required>"));
    
    // Port
    client.println(F("Port:<br><input name='port' type='number' value='"));
    client.print(_cfg->data.hostPort);
    client.println(F("' required>"));
    
    // RSSI Mode
    client.println(F("RSSI Mode:<br><select name='rssiam' style='width:100%;padding:8px;margin-bottom:15px'>"));
    client.print(F("<option value='0'")); if(!_cfg->data.useHwRSSI) client.print(" selected"); client.println(F(">Software (DSP)</option>"));
    client.print(F("<option value='1'")); if(_cfg->data.useHwRSSI) client.print(" selected"); client.println(F(">Hardware (Analog)</option>"));
    client.println(F("</select>"));
    
    // Passwords
    client.println(F("Client Password:<br><input name='cpwd' value='"));
    client.print(_cfg->data.clientPwd);
    client.println(F("'>"));

    client.println(F("Host Password:<br><input name='hpwd' value='"));
    client.print(_cfg->data.hostPwd);
    client.println(F("'>"));

    client.println(F("<input type='submit' value='Save & Reboot' class='btn'>"));
    client.println(F("</form></div></body></html>"));
}

void WebInterface::_parseParams(String& body) {
    // Example: ip=192.168.1.100&port=667&rssiam=0
    // Extremely basic parsing
    
    int idx;
    
    // IP
    idx = body.indexOf("ip=");
    if (idx != -1) {
        String val = body.substring(idx + 3);
        int end = val.indexOf('&');
        if (end != -1) val = val.substring(0, end);
        val = _urlDecode(val);
        
        IPAddress newIP;
        newIP.fromString(val);
        _cfg->setHostIP(newIP);
    }
    
    // Port
    idx = body.indexOf("port=");
    if (idx != -1) {
        String val = body.substring(idx + 5);
        int end = val.indexOf('&');
        if (end != -1) val = val.substring(0, end);
        _cfg->data.hostPort = val.toInt();
    }
    
    // RSSI
    idx = body.indexOf("rssiam=");
    if (idx != -1) {
        String val = body.substring(idx + 7);
        int end = val.indexOf('&');
        if (end != -1) val = val.substring(0, end);
        _cfg->data.useHwRSSI = (val.toInt() == 1);
    }
    
    // Pwd
    idx = body.indexOf("cpwd=");
    if (idx != -1) {
        String val = body.substring(idx + 5);
        int end = val.indexOf('&');
        if (end != -1) val = val.substring(0, end);
        val = _urlDecode(val);
        strncpy(_cfg->data.clientPwd, val.c_str(), 19);
    }
    
     idx = body.indexOf("hpwd=");
    if (idx != -1) {
        String val = body.substring(idx + 5);
        int end = val.indexOf('&');
        if (end != -1) val = val.substring(0, end);
        val = _urlDecode(val);
        strncpy(_cfg->data.hostPwd, val.c_str(), 19);
    }
    
    _cfg->save();
    delay(500);
    // Reboot (Teensy Specific) via AIRCR write
    SCB_AIRCR = 0x05FA0004;
}

String WebInterface::_urlDecode(String str) {
    String ret = "";
    char c;
    for (unsigned int i = 0; i < str.length(); i++) {
        c = str.charAt(i);
        if (c == '+') {
            ret += ' ';
        }
        else if (c == '%') {
            char code0 = str.charAt(i + 1);
            char code1 = str.charAt(i + 2);
            c = (strtol(&code0, NULL, 16) << 4) | strtol(&code1, NULL, 16);
            ret += c;
            i += 2;
        }
        else {
            ret += c;
        }
    }
    return ret;
}
