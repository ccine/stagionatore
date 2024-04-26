//#include <SD.h>
#include <ArduinoJson.h>
#include "display.h"
// #include <EEPROM.h>
#include "chamber_process.h"


#define DISPLAY_PIN 6
#define SD_CS 4
#define IS_RUNNING_ADDR 0
#define STEP_INDEX_ADDR 1
#define STEP_START_ADDR (sizeof(unsigned int) + STEP_INDEX_ADDR)

// Dichiarazione delle variabili globali
Stagionatore stagionatore;
CustomDisplay cd;

StagionaturaProgram currentProgram("Programma 1");
chamberSensorData currentSensorData;


void setup() {
  Serial.begin(9600);



  // Init SD_Card
  pinMode(SD_CS, OUTPUT);


  // Inizializzazione della microSD
  /*if (!SD.begin(SD_CS)) {
    Serial.println("Errore nell'inizializzazione della microSD!");
  } else {
    // Carica il programma automatico da un file JSON sulla microSD
    loadProgramFromFile("auto_program.json");
  }*/
  // currentProgram.addStep(11 * 60, 22, 85);
  // currentProgram.addStep(12 * 60, 20, 75);
  //currentProgram.addStep(24 * 60, 20, 75);
  //currentProgram.addStep(24 * 60, 18, 76);
  //currentProgram.addStep(24 * 60, 16, 76);
  //currentProgram.addStep(24 * 60, 14, 80);
  //currentProgram.addStep(24 * 60, 12, 85);
  currentProgram.addStep(24 * 60, 12, 83);
  currentProgram.addStep(24 * 60, 12, 82);
  currentProgram.addStep(24 * 60, 10, 81);
  currentProgram.addStep(24 * 60, 10, 80);

  // EEPROM.get(IS_RUNNING_ADDR, isRunningSaved);
  // EEPROM.get(STEP_INDEX_ADDR, currentStepIndex);
  // EEPROM.get(STEP_START_ADDR, currentStepStartTime);


  // Inizializzazione stagionatore
  stagionatore.begin();

  // Inizializzazione del display
  cd.initDisplay(&stagionatore);
}

void loop() {
  stagionatore.run();

  cd.update();
}




/*
StagionaturaProgram loadProgramFromFile(String filename)
{
  // Carica un programma da file JSON
  File file = SD.open(filename);

  if (file)
  {
    // Leggi il contenuto del file
    String jsonContent = "";
    while (file.available())
    {
      jsonContent += (char)file.read();
    }
    file.close();

    // Dichiarazione di un documento JSON per memorizzare i dati letti
    DynamicJsonDocument doc(1024); // Specifica la dimensione massima del documento

    // Deserializza il JSON dal contenuto del file
    DeserializationError error = deserializeJson(doc, jsonContent);

    // Controlla se c'è un errore durante la deserializzazione
    if (error)
    {
      Serial.print("Errore durante la deserializzazione JSON: ");
      Serial.println(error.c_str());
      return;
    }

    // Estrai i dati e popola l'oggetto StagionaturaProgram
    StagionaturaProgram autoProgram("Auto Program");
    autoProgram.numSteps = doc["numSteps"];
    JsonArray steps = doc["steps"].as<JsonArray>();

    for (JsonVariant step : steps)
    {
      int duration = step["duration"];
      float temp = step["targetTemperature"];
      float hum = step["targetHumidity"];
      autoProgram.addStep(duration, temp, hum);
    }

    // Utilizza l'oggetto autoProgram nel tuo programma come necessario
    // Ad esempio, assegnalo a currentProgram
    currentProgram = autoProgram;

    Serial.println("Programma automatico caricato con successo!");
  }
  else
  {
    Serial.println("Impossibile aprire il file.");
  }
}
*/