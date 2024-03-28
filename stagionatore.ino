#include "DHT.h"
#include <SD.h>
#include <SPI.h>
#include <ArduinoJson.h>
#include "CustomDisplay.h"
#include "RTClib.h"
// #include <EEPROM.h>
#include "DFRobot_SHT20.h"

#define HUMIDITY_DELTA 2  // Percentuale
#define TEMP_DELTA 1      // Gradi
#define MIN_WAIT_FRIDGE 5 // minuti
#define FAN_OFF_TIME 180  // secondi
#define FAN_ON_TIME 90    // secondi

// Definizione delle costanti per i pin dei relay e sensori
#define COOLING_RELAY_PIN 25      // relay F
#define HEATING_RELAY_PIN 23      // relay C
#define FAN_RELAY_PIN 29          // relay V
#define DEHUMIDIFIER_RELAY_PIN 33 // relay D
#define GREEN_CABLE_PIN 31        // relay U
#define HUMIDIFIER_RELAY_PIN 27   // relay E
#define TEMP_HUM_SENSOR_PIN A0
#define DISPLAY_PIN 6
#define SENSOR_UP_ID 0x29
#define SENSOR_DOWN_ID 0x28
#define SENSOR_READ_INTERVAL 5
#define SD_CS 4
#define IS_RUNNING_ADDR 0
#define STEP_INDEX_ADDR 1
#define STEP_START_ADDR (sizeof(unsigned int) + STEP_INDEX_ADDR)

// Dichiarazione delle variabili globali
StagionaturaProgram currentProgram("Programma 1");
float currentTemperatureUp = 0;
int currentHumidityUp = 0;
float currentTemperatureDown = 0;
int currentHumidityDown = 0;
float currentTemperature = 0;
int currentHumidity = 0;
unsigned long lastSensorReadTime = 10; // Memorizza il tempo dell'ultima lettura dei sensori
unsigned int currentStepIndex = 0;
unsigned long currentStepStartTime = 0;                   // Minuti
unsigned long fanSwitchTime, fridgeOffTime, fridgeOnTime; // Secondi
bool cooling = false, heating = false, fan = false, dehumidifier = false, humidifier = false;
CustomDisplay cd;
RTC_DS3231 rtc;
unsigned long currentSeconds;
DFRobot_SHT20 sht20(&Wire, SHT20_I2C_ADDR);
int isRunningSaved = 0;

void setup()
{
  Serial.begin(9600);

  // Inizializzazione sensori temperatura e umidità
  sht20.initSHT20();

  // Inizializzazione dei pin
  pinMode(COOLING_RELAY_PIN, OUTPUT);
  pinMode(HEATING_RELAY_PIN, OUTPUT);
  pinMode(FAN_RELAY_PIN, OUTPUT);
  pinMode(HUMIDIFIER_RELAY_PIN, OUTPUT);

  // Init SD_Card
  pinMode(SD_CS, OUTPUT);

  if (!rtc.begin())
  {
    Serial.println("Couldn't find RTC");
  }
  // TO RESET rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

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

  RunningStep runningStep = RunningStep();
  /*if (isRunningSaved == 1) {
    currentSeconds = rtc.now().unixtime();
    unsigned long currentMinutes = currentSeconds / 60;  // Converti il tempo Unix in minuti
    unsigned long elapsedMinutes = currentMinutes - currentStepStartTime;
    runningStep = RunningStep(currentProgram.steps[currentStepIndex], elapsedMinutes);
  }*/

  // Inizializzazione del display
  cd.initDisplay(COOLING_RELAY_PIN, HEATING_RELAY_PIN, FAN_RELAY_PIN, DEHUMIDIFIER_RELAY_PIN, HUMIDIFIER_RELAY_PIN, runningStep);
}

void loop()
{
  // Leggi il tempo corrente
  currentSeconds = rtc.now().unixtime();

  // Esegui readSensors ogni SENSOR_READ_INTERVAL millisecondi
  if (currentSeconds - lastSensorReadTime >= SENSOR_READ_INTERVAL)
  {
    // Leggi i sensori
    readSensors();
    lastSensorReadTime = currentSeconds;
  }

  cd.update(currentTemperatureUp, currentHumidityUp, currentTemperatureDown, currentHumidityDown);

  // Se sono in running esegui il programma o la modalità auto
  if (cd.isRunning())
  {
    if (cd.isProgramMode())
    {
      runStagionaturaProgram();
    }
    else
    {
      runProgramStep(cd.getSelectedTemperature(), cd.getSelectedHumidity());
      cd.updateRunning(cooling, heating, fan, dehumidifier, humidifier, cd.getSelectedTemperature(), cd.getSelectedHumidity());
    }
  }
  else
  {
    if (!cooling || currentSeconds - fridgeOnTime >= MIN_WAIT_FRIDGE * 60)
    {
      reachedTargetTemperature();
    }
    reachedTargetHumidity();
    turnOffFan();
    if (isRunningSaved == 1)
    {
      isRunningSaved = 0;
      // EEPROM.put(IS_RUNNING_ADDR, isRunningSaved);
    }
  }
}

void readSensors()
{
  byte aa, bb, cc, dd;
  float temperature = 0.0;
  float humidity = 0.0;

  // Leggi i valori dal sensore di temperatura e umidità superiore sht20
  float h = sht20.readHumidity();
  // Read temperature as Celsius (the default)
  float t = round(sht20.readTemperature() * 10) / 10.0;

  // Leggi sensore superiore
  getdata(&aa, &bb, &cc, &dd, SENSOR_UP_ID);

  // humidity = (rH_High [5:0] x 256 + rH_Low [7:0]) / 16384 x 100
  humidity = (float)(((aa & 0x3F) << 8) + bb) / 16384.0 * 100.0;
  // temperature = (Temp_High [7:0] x 64 + Temp_Low [7:2]/4 ) / 16384 x 165 - 40
  temperature = (float)((unsigned)(cc * 64) + (unsigned)(dd >> 2)) / 16384.0 * 165.0 - 40.0;
  currentTemperatureUp = (temperature + t) / 2.0;
  currentHumidityUp = (humidity + h) / 2.0;

  // Leggi sensore inferiore
  getdata(&aa, &bb, &cc, &dd, SENSOR_DOWN_ID);

  // humidity = (rH_High [5:0] x 256 + rH_Low [7:0]) / 16384 x 100
  humidity = (float)(((aa & 0x3F) << 8) + bb) / 16384.0 * 100.0;
  // temperature = (Temp_High [7:0] x 64 + Temp_Low [7:2]/4 ) / 16384 x 165 - 40
  temperature = (float)((unsigned)(cc * 64) + (unsigned)(dd >> 2)) / 16384.0 * 165.0 - 40.0;
  currentTemperatureDown = temperature;
  currentHumidityDown = round(humidity);

  // Valore media
  currentTemperature = (float)(currentTemperatureUp + currentTemperatureDown) / 2.0;
  currentHumidity = (float)(currentHumidityUp + currentHumidityDown) / 2.0;
  // currentHumidity = currentHumidityUp;
}

// Esegui il programma di stagionatura corrente
void runStagionaturaProgram()
{
  unsigned long currentMinutes = currentSeconds / 60; // Converti il tempo Unix in minuti
  if (isRunningSaved == 0)
  {
    isRunningSaved = 1;
    currentStepIndex = 0;
    currentStepStartTime = currentMinutes;
    // EEPROM.put(IS_RUNNING_ADDR, isRunningSaved);
    // EEPROM.put(STEP_INDEX_ADDR, currentStepIndex);
    // EEPROM.put(STEP_START_ADDR, currentStepStartTime);
  }

  // Ottieni il passo corrente del programma
  ProgramStep currentStep = currentProgram.steps[currentStepIndex];

  runProgramStep(currentStep.targetTemperature, currentStep.targetHumidity);

  // Verifica se il passo corrente è terminato in base al tempo corrente in minuti
  unsigned long elapsedMinutes = currentMinutes - currentStepStartTime;
  if (elapsedMinutes >= currentStep.duration)
  {
    currentStepIndex++;
    currentStepStartTime = currentMinutes; // Resetta il tempo di inizio del passo corrente
    // EEPROM.put(STEP_INDEX_ADDR, currentStepIndex);
    // EEPROM.put(STEP_START_ADDR, currentStepStartTime);
  }

  // Se il programma è terminato, riporta all'inizio
  if (currentStepIndex >= currentProgram.numSteps)
  {
    isRunningSaved = 0;
    // EEPROM.put(IS_RUNNING_ADDR, isRunningSaved);
    currentStepIndex = 0;
    cd.stop();
  }
  else
  {
    cd.updateRunning(cooling, heating, fan, dehumidifier, humidifier, currentStep.targetTemperature, currentStep.targetHumidity, currentStepIndex, currentProgram.numSteps, elapsedMinutes, currentStep.duration);
  }
}

// In base alla temperatura e umidità impostate accendo i relè necessari
void runProgramStep(float targetTemperature, int targetHumidity)
{
  // Attiva il ricircolo dell'aria
  fanController();

  // Se il frigo è acceso, aspetto 5 minuti prima di controllare la temperatura
  if (!cooling || currentSeconds - fridgeOnTime >= MIN_WAIT_FRIDGE * 60)
  {
    // Aggiusta la temperatura
    if (targetTemperature >= 100)
    {
      // Spegni tutto se la temperatura non è impostata
      reachedTargetTemperature();
    }
    else if (currentTemperature >= targetTemperature + TEMP_DELTA)
    {
      // Temperatura troppo alta, accendi il frigo
      decreaseTemperature();
    }
    else if (currentTemperature <= targetTemperature - TEMP_DELTA)
    {
      // Temperatura troppo bassa, accendi i riscaldatori
      increaseTemperature();
    }
    else
    {
      if ((cooling && currentTemperature <= targetTemperature) || (heating && currentTemperature >= targetTemperature))
      {
        // Spegni tutti i relay se la temperatura è giusta
        reachedTargetTemperature();
      }
    }
  }

  // Aggiusta l'umidità
  if (targetHumidity >= 100)
  {
    // Spegni tutto se l'umidità non è impostata
    reachedTargetHumidity();
  }
  else if (currentHumidity >= targetHumidity + HUMIDITY_DELTA)
  {
    // Umidità troppo alta, accendi l'estrattore
    decreaseHumidity();
  }
  else if (currentHumidity <= targetHumidity - HUMIDITY_DELTA)
  {
    // Umidità troppo bassa, accendi umidificatore
    increaseHumidity();
  }
  else
  {
    if ((dehumidifier && currentHumidity <= targetHumidity) || (humidifier && currentHumidity >= targetHumidity))
    {
      // Spegni tutti i relay se la temperatura è giusta
      reachedTargetHumidity();
    }
  }
}

void fanController()
{
  if (humidifier)
  {
    if (!fan)
    {
      digitalWrite(FAN_RELAY_PIN, HIGH);
      fan = true;
      fanSwitchTime = currentSeconds;
    }
  }
  else
  {
    if (fan)
    {
      // Attendi x minuti prima di spegnere la ventola
      if (currentSeconds - fanSwitchTime >= FAN_ON_TIME)
      {
        digitalWrite(FAN_RELAY_PIN, LOW);
        fan = false;
        fanSwitchTime = currentSeconds;
      }
    }
    else
    {
      // Attendi x minuti prima di accendere la ventola
      if (currentSeconds - fanSwitchTime >= FAN_OFF_TIME)
      {
        digitalWrite(FAN_RELAY_PIN, HIGH);
        fan = true;
        fanSwitchTime = currentSeconds;
      }
    }
  }
}

void reachedTargetTemperature()
{
  if (heating || cooling)
  {
    if (cooling)
      fridgeOffTime = currentSeconds;
    heating = false;
    cooling = false;
    digitalWrite(HEATING_RELAY_PIN, LOW);
    digitalWrite(COOLING_RELAY_PIN, LOW);
  }
}

void decreaseTemperature()
{
  if (!cooling)
  {
    heating = false;
    digitalWrite(HEATING_RELAY_PIN, LOW);
    // Frigo è stato spento per almeno MIN_WAIT_FRIDGE minuti
    if (currentSeconds - fridgeOffTime >= MIN_WAIT_FRIDGE * 60)
    {
      cooling = true;
      digitalWrite(COOLING_RELAY_PIN, HIGH);
      fridgeOnTime = currentSeconds;
    }
  }
}

void increaseTemperature()
{
  if (!heating)
  {
    if (cooling)
    {
      cooling = false;
      fridgeOffTime = currentSeconds;
    }
    heating = true;
    digitalWrite(HEATING_RELAY_PIN, HIGH);
    digitalWrite(COOLING_RELAY_PIN, LOW);
  }
}

void reachedTargetHumidity()
{
  if (humidifier || dehumidifier)
  {
    humidifier = false;
    dehumidifier = false;
    digitalWrite(DEHUMIDIFIER_RELAY_PIN, LOW);
    digitalWrite(HUMIDIFIER_RELAY_PIN, LOW);
  }
}

void decreaseHumidity()
{
  if (!dehumidifier)
  {
    dehumidifier = true;
    humidifier = false;
    digitalWrite(DEHUMIDIFIER_RELAY_PIN, HIGH);
    digitalWrite(HUMIDIFIER_RELAY_PIN, LOW);
  }
}

void increaseHumidity()
{
  if (!humidifier)
  {
    humidifier = true;
    dehumidifier = false;
    digitalWrite(HUMIDIFIER_RELAY_PIN, HIGH);
    digitalWrite(DEHUMIDIFIER_RELAY_PIN, LOW);
  }
}

void turnOffFan()
{
  if (fan)
  {
    digitalWrite(FAN_RELAY_PIN, LOW);
    fan = false;
  }
}

void getdata(byte *a, byte *b, byte *c, byte *d, int address)
{
  Wire.beginTransmission(address);
  Wire.write(0);
  Wire.endTransmission();

  Wire.requestFrom(address, 4);
  *a = Wire.read();
  *b = Wire.read();
  *c = Wire.read();
  *d = Wire.read();
}

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
