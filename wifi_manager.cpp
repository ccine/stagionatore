#include "wifi_manager.h"

WiFiManager::WiFiManager(Stagionatore* stagionatore)
  : _stagionatore(stagionatore), _server(80) {}  // Server sulla porta 80

void WiFiManager::begin() {
  // Check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // Don't continue
    while (true)
      ;
  }
  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  IPAddress ip(192, 168, 0, 251);
  IPAddress gateway(192, 168, 0, 1);
  IPAddress subnet(255, 255, 255, 0);
  IPAddress dns(8, 8, 8, 8);

  // Connect to WiFi using static IP address
  WiFi.config(ip, dns, gateway, subnet);

  // Attempt to connect to WiFi network:
  while (_status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(_ssid);
    _status = WiFi.begin(_ssid, _pass);
    // Wait 10 seconds for connection:
    delay(10000);
  }
  Serial.println("Connected to WiFi");
  _server.begin();  // Avvia il server
}

void WiFiManager::handleClient() {
  WiFiClient client = _server.available();  // Controlla se ci sono client connessi
  if (client) {
    Serial.println("New client connected");
    String currentLine = "";
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        Serial.write(c);  // Echo per debug
        if (c == '\n') {
          if (currentLine.length() == 0) {
            handleRoot(client);  // Risposta di default
            break;
          } else {
            if (currentLine.startsWith("GET /status")) {
              handleStatus(client);
              break;
            } else if (currentLine.startsWith("GET /command?")) {
              int pos = currentLine.indexOf("?");
              String command = currentLine.substring(pos + 1);
              handleCommand(client, command);
              break;
            }
            currentLine = "";
          }
        } else if (c != '\r') {
          currentLine += c;
        }
      }
    }
    client.stop();
    Serial.println("Client disconnected");
  }
}

void WiFiManager::handleRoot(WiFiClient& client) {
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");
  client.println("Connection: close");
  client.println();
  client.println("<!DOCTYPE HTML>");
  client.println("<html>");
  client.println("<head><title>Stagionatore</title></head>");
  client.println("<body>");
  client.println("<h1>Arduino Stagionatore Server</h1>");
  client.println("<p>Use /status to get the current status.</p>");
  client.println("<p>Use /command?startAuto or /command?stop to control.</p>");
  client.println("</body></html>");
}

void WiFiManager::handleStatus(WiFiClient& client) {
  chamberStatus status = _stagionatore->getStatus();
  chamberSensorData sensorData = _stagionatore->getSensorData();

  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: application/json");
  client.println("Connection: close");
  client.println();
  client.print("{");
  client.print("\"temperatureUp\":");
  client.print(sensorData.temperatureUp);
  client.print(",\"humidityUp\":");
  client.print(sensorData.humidityUp);
  client.print(",\"temperatureDown\":");
  client.print(sensorData.temperatureDown);
  client.print(",\"humidityDown\":");
  client.print(sensorData.humidityDown);
  client.print(",\"targetTemperature\":");
  client.print(status.targetTemperature);
  client.print(",\"targetHumidity\":");
  client.print(status.targetHumidity);
  client.print(",\"cooling\":");
  client.print(status.cooling ? "true" : "false");
  client.print(",\"heating\":");
  client.print(status.heating ? "true" : "false");
  client.print(",\"fan\":");
  client.print(status.fan ? "true" : "false");
  client.print(",\"dehumidifier\":");
  client.print(status.dehumidifier ? "true" : "false");
  client.print(",\"humidifier\":");
  client.print(status.humidifier ? "true" : "false");
  client.print(",\"isRunning\":");
  client.print(status.isRunning ? "true" : "false");
  client.print(",\"currentStep\":");
  client.print(status.currentStep);
  client.print(",\"nSteps\":");
  client.print(status.nSteps);
  client.print(",\"elapsedTime\":");
  client.print(status.elapsedTime);
  client.print(",\"duration\":");
  client.print(status.duration);
  client.println("}");
}

void WiFiManager::handleCommand(WiFiClient& client, const String& command) {
  if (command.indexOf("startManual") >= 0) {
    // Estrai temperature e umidità dal comando
    int targetTempIndex = command.indexOf("targetTemperature=") + 17;
    int targetHumIndex = command.indexOf("targetHumidity=") + 15;

    float targetTemperature = command.substring(targetTempIndex, command.indexOf("&", targetTempIndex)).toFloat();
    float targetHumidity = command.substring(targetHumIndex, command.length()).toFloat();

    _stagionatore->startManual(targetTemperature, targetHumidity);
  } else if (command.indexOf("startAuto") >= 0) {
    _stagionatore->startProgram();
  } else if (command.indexOf("stop") >= 0) {
    _stagionatore->stop();
  }

  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/plain");
  client.println("Connection: close");
  client.println();
  client.println("Command executed");
}
