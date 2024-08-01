#include "chamber_process.h"
#include "wifi_manager.h"

// Dichiarazione delle variabili globali
Stagionatore stagionatore;
WiFiManager wifiManager(&stagionatore);


void setup() {
  Serial.begin(9600);

  // Inizializzazione stagionatore
  stagionatore.begin();

  // Inizializzazione WiFi e server web
  wifiManager.begin();
}

void loop() {
  stagionatore.run();
  wifiManager.handleClient();
}