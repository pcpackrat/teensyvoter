#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include "ConfigManager.h"
#include "GPSManager.h"
#include "NetworkManager.h"
#include "VoterClient.h"
#include <Arduino.h>
#include <NativeEthernet.h>

class WebServer {
public:
  WebServer();

  void begin();
  void handleClient();
  void setConfig(ConfigManager *cfg);
  void setSystemObjects(NetworkManager *netMgr, VoterClient *voter,
                        GPSManager *gps);

private:
  EthernetServer _server;
  ConfigManager *_config;
  NetworkManager *_netMgr;
  VoterClient *_voter;
  GPSManager *_gps;

  // HTTP request parsing
  void handleRequest(EthernetClient &client);
  String readRequestLine(EthernetClient &client);
  void parseRequest(const String &request, String &method, String &path);

  // Route handlers
  void handleRoot(EthernetClient &client);
  void handleNetwork(EthernetClient &client);
  void handleVoter(EthernetClient &client);
  void handleAudio(EthernetClient &client);
  void handleSystem(EthernetClient &client);
  void handleApiStatus(EthernetClient &client);
  void handleNotFound(EthernetClient &client);

  // Response helpers
  void sendHeader(EthernetClient &client, int code, const char *contentType);
  void sendHtmlHeader(EthernetClient &client);
  void sendHtmlFooter(EthernetClient &client);
  void sendRedirect(EthernetClient &client, const String &location);
  void sendJsonResponse(EthernetClient &client, bool success,
                        const String &message);

  // POST handlers
  void handleNetworkPost(EthernetClient &client, const String &body);
  void handleVoterPost(EthernetClient &client, const String &body);
  void handleAudioPost(EthernetClient &client, const String &body);

  // Export/Import
  void handleExport(EthernetClient &client);
  void handleImport(EthernetClient &client);
  void handleImportPost(EthernetClient &client, const String &body);

  // Utilities
  String getFormValue(const String &body, const String &key);
  bool parseIPAddress(const String &str, uint32_t &ip);
  String readRequestBody(EthernetClient &client, int contentLength);
};

#endif
