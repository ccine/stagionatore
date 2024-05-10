// #include <EEPROM.h>
#include "chamber_process.h"
#include "wifi_manager.h"

#define IS_RUNNING_ADDR 0
#define STEP_INDEX_ADDR 1
#define STEP_START_ADDR (sizeof(unsigned int) + STEP_INDEX_ADDR)

// Dichiarazione delle variabili globali
Stagionatore stagionatore;
WiFiManager wifiManager(&stagionatore);

StagionaturaProgram currentProgram("Programma 1");
chamberSensorData currentSensorData;


void setup() {
  Serial.begin(9600);

  // EEPROM.get(IS_RUNNING_ADDR, isRunningSaved);
  // EEPROM.get(STEP_INDEX_ADDR, currentStepIndex);
  // EEPROM.get(STEP_START_ADDR, currentStepStartTime);


  // Inizializzazione stagionatore
  stagionatore.begin();
  
  // Inizializzazione WiFi e server web
  wifiManager.begin();
}

void loop() {
  stagionatore.run();
  wifiManager.handleClient();
}