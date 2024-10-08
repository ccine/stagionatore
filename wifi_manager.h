#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <Arduino.h>
#include "WiFiS3.h"

#include "arduino_secrets.h"
#include "chamber_process.h"

class WiFiManager {
public:
  WiFiManager(Stagionatore* stagionatore);
  void begin();
  void handleClient();

private:
  char* _ssid = SECRET_SSID;  // your network SSID (name)
  char* _pass = SECRET_PASS;  // your network password (use for WPA, or use as key for WEP)
  int _status = WL_IDLE_STATUS;
  WiFiServer _server;

  Stagionatore* _stagionatore;

  void handleRoot(WiFiClient& client);
  void handleStatus(WiFiClient& client);
  void handleCommand(WiFiClient& client, const String& command);
};

#endif