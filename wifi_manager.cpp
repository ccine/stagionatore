#include "wifi_manager.h"

WiFiManager::WiFiManager(Stagionatore* stagionatore)
  : _stagionatore(stagionatore), _server(80) {
}

void WiFiManager::begin() {
  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true)
      ;
  }
  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  IPAddress ip(192, 168, 0, 231);
  IPAddress gateway(192, 168, 0, 1);
  IPAddress subnet(255, 255, 255, 0);
  IPAddress dns(8, 8, 8, 8);

  // Connect to WiFi using static IP address
  WiFi.config(ip, dns, gateway, subnet);

  // attempt to connect to WiFi network:
  while (_status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(_ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    _status = WiFi.begin(_ssid, _pass);

    // wait 10 seconds for connection:
    delay(10000);
  }
  Serial.println("Connected");
  _server.begin();
}

void WiFiManager::handleClient() {
  // listen for incoming clients
  WiFiClient client = _server.available();
  bool redirect = false;
  if (client) {
    Serial.println("new client");
    // an HTTP request ends with a blank line
    String currentLine = "";  // make a String to hold incoming data from the client
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        Serial.write(c);


        // Check client request
        if (currentLine.startsWith("GET /start-manual?") && c == '\n') {
          int tempIndex = currentLine.indexOf("temp-set=");
          int umidIndex = currentLine.indexOf("&umid-set=");
          if (tempIndex != -1 && umidIndex != -1) {
            // Extract the values
            String temp = currentLine.substring(tempIndex + 9, umidIndex);
            String umid = currentLine.substring(umidIndex + 10);

            // Convert to float and integer
            float tempValue = temp.toFloat();
            int humValue = umid.toInt();

            _stagionatore->startManual(tempValue, humValue);
          }
          redirect = true;
        } else if (currentLine.endsWith("GET /start-auto")) {
          _stagionatore->startProgram();
          redirect = true;
        } else if (currentLine.endsWith("GET /STOP")) {
          _stagionatore->stop();
          redirect = true;
        }


        // if you've gotten to the end of the line (received a newline
        // character) and the line is blank, the HTTP request has ended,
        // so you can send a reply
        if (c == '\n') {
          if (currentLine.length() == 0) {
            if (!redirect) {
              loadHTMLPage(&client);
            } else {
              // Redirect to root ("/")
              client.print("HTTP/1.1 303 See Other\r\n");
              client.print("Location: /\r\n");
              client.print("Connection: close\r\n");
              client.print("\r\n");
            }
            break;
          } else {  // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }

    // give the web browser time to receive the data
    delay(1);

    // close the connection:
    client.stop();
    Serial.println("client disconnected");
  }
}


void WiFiManager::loadHTMLPage(WiFiClient* client) {
  // send a standard HTTP response header
  client->println("HTTP/1.1 200 OK");
  client->println("Content-Type: text/html");
  client->println("Connection: close");  // the connection will be closed after completion of the response
  client->println("Refresh: 10");         // refresh the page automatically every 5 sec
  client->println();

  // HTML Page
  // Header
  client->println(getHTMLHeader());
  // Body
  client->println("<div class='container mt-3'>");

  chamberStatus status = _stagionatore->getStatus();
  // Alert
  client->println(getHTMLStatusAlert(status.isRunning));
  // Actual values
  client->println(getHTMLSensorData());

  if (status.isRunning) {
    client->println(getHTMLRunningStatus(status));
  } else {
    client->println(getHTMLStartManualMode());
  }


  client->println("</div>");
  // Footer
  client->println(getHTMLFooter());
  // The HTTP response ends with another blank line
  client->println();
}


String WiFiManager::getHTMLStatusAlert(bool isRunning) {
  if (isRunning) {
    return "<div id='stato-stagionatore' class='alert alert-success' role='alert'>\
              Stato: <strong>Acceso</strong>\
              </div>";
  } else {
    return "<div id='stato-stagionatore' class='alert alert-danger' role='alert'>\
              Stato: <strong>Spento</strong>\
              </div>";
  }
}


String WiFiManager::getHTMLSensorData() {
  chamberSensorData sensorData = _stagionatore->getSensorData();
  return String(String("<div class='row'>\
    <div class='col'>\
      <h4>Temperatura Alto: <span id='temp-alto'>")
                + sensorData.temperatureUp + "°C </span > </h4>\
                    </div>\
                    <div class = 'col'>\
                    <h4> Umidità Alto : <span id = 'umid-alto'>"
                + sensorData.humidityUp + "% </span></h4></div></div><div class = 'row'><div class = 'col'>\
                    <h4> Temperatura Basso : <span id = 'temp-basso'>"
                + sensorData.temperatureDown + "°C</span > </h4></div>\
                 <div class = 'col'>\
                 <h4> Umidità Basso : <span id = 'umid-basso'>"
                + sensorData.humidityDown + "% </span></h4></div></div>");
}

String WiFiManager::getHTMLRunningStatus(chamberStatus status) {
  return String("<!-- Controlli se lo stagionatore è acceso -->\
    <div id='controlli-stagionatore' class='my-3'>\
    <h5>Impostazioni Attuali:</h5>\
    <p>Temperatura Impostata: <span id='temp-impostata'>")
         + status.targetTemperature + "°C</span></p>\
    <p>Umidità Impostata: <span id='umid-impostata'>"
         + status.targetHumidity + "%</span></p>\
    <p>Step: <span id='step'>"
         + status.currentStep + "/" + status.nSteps + "</span></p>\
    <p>Tempo(min): <span id='time'>"
         + status.elapsedTime + "/" + status.duration + "</span></p>\
    <!-- Informazioni sulla ventola e sugli altri dispositivi -->\
    <p>Ventola: "
         + getHTMLEquipmentStatus(status.fan) + "</p>\
    <p>Riscaldatore: "
         + getHTMLEquipmentStatus(status.heating) + "</p>\
    <p>Raffreddatore: "
         + getHTMLEquipmentStatus(status.cooling) + "</p>\
    <p>Umidificatore: "
         + getHTMLEquipmentStatus(status.humidifier) + "</p>\
    <p>Deumidificatore: "
         + getHTMLEquipmentStatus(status.dehumidifier) + "</p>\
    <a class='btn btn-danger' href='/STOP' role='button'>Stop</a>\
    </div>";
}

String WiFiManager::getHTMLEquipmentStatus(bool status) {
  if (status) return "<span class='equipment-status text-success'>Acceso</span>";
  else return "<span class='equipment-status text-danger'>Spento</span></p>";
}


String WiFiManager::getHTMLStartManualMode() {
  return "<div id='form-stagionatura' class='my-3'>\
    <h5>Inizia Programma di Stagionatura:</h5>\
    <form action='/start-manual'>\
      <div class='form-group'>\
        <label for='temp-set'>Imposta Temperatura:</label>\
        <input type='number' id='temp-set' name='temp-set' class='form-control' placeholder='Temperatura in °C'>\
      </div>\
      <div class='form-group'>\
        <label for='umid-set'>Imposta Umidità:</label>\
        <input type='number' id='umid-set' name='umid-set' class='form-control' placeholder='Umidità in %'>\
      </div>\
      <button type='submit' class='btn btn-primary'>Inizia</button>\
    </form>\
    </br>\
    <a class='btn btn-info' href='/start-auto' role='button'>Start Auto</a>\
  </div>";
}

String WiFiManager::getHTMLHeader() {
  return "<!DOCTYPE html> <html lang = 'it'> <head>\
    <meta charset = 'UTF-8'>\
    <meta name = 'viewport' content = 'width=device-width, initial-scale=1.0'>\
    <title>Controllo Stagionatore</title>\
    <!--Includi Bootstrap CSS-->\
    <link rel = 'stylesheet' href = 'https://stackpath.bootstrapcdn.com/bootstrap/4.5.2/css/bootstrap.min.css'>\
    <style>\
    /* Personalizzazioni CSS aggiuntive */\
    .row {\
      margin-bottom: 1em;\
    }\
    .equipment-status {\
      font-weight: bold;\
    }\
    </style>\
    </head>";
}

String WiFiManager::getHTMLFooter() {
  return "<!-- Includi Bootstrap JS e dipendenze -->\
    <script src='https://code.jquery.com/jquery-3.5.1.slim.min.js'></script>\
    <script src='https://cdn.jsdelivr.net/npm/@popperjs/core@2.9.3/dist/umd/popper.min.js'></script>\
    <script src='https://stackpath.bootstrapcdn.com/bootstrap/4.5.2/js/bootstrap.min.js'></script>\
    </body>\
    </html>";
}