#ifndef WEB_INTERFACE_H
#define WEB_INTERFACE_H

#include <Arduino.h>
#include <NativeEthernet.h>
#include "ConfigManager.h"
#include "GPSManager.h"
#include "VoterClient.h"

// Forward Declaration
class WebInterface {
public:
    WebInterface();
    void begin(ConfigManager* cfg, GPSManager* gps, VoterClient* voter);
    void update(); // Call in loop()

private:
    EthernetServer* _server;
    ConfigManager* _cfg;
    GPSManager* _gps;
    VoterClient* _voter;

    void _handleRequest(EthernetClient& client);
    void _sendHeader(EthernetClient& client);
    void _sendFooter(EthernetClient& client);
    void _parseParams(String& req);
    String _urlDecode(String str);
};

#endif
