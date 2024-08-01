#ifndef CHAMBER_H
#define CHAMBER_H

#include "DHT.h"
#include <SPI.h>
#include "RTClib.h"
#include "DFRobot_SHT20.h"
#include <Arduino.h>
#include <Wire.h>
#include "Program.h"
#include <EEPROM.h>

#define HUMIDITY_DELTA 2        // Percentuale
#define TEMP_DELTA 1            // Gradi
#define MIN_WAIT_FRIDGE 5       // minuti
#define FAN_OFF_TIME 180        // secondi
#define FAN_ON_TIME 90          // secondi
#define SENSOR_READ_INTERVAL 5  // secondi

// Definizione delle costanti per i pin e id di relay e sensori
#define COOLING_RELAY_PIN 25       // relay F
#define HEATING_RELAY_PIN 23       // relay C
#define FAN_RELAY_PIN 29           // relay V
#define DEHUMIDIFIER_RELAY_PIN 33  // relay D
#define GREEN_CABLE_PIN 31         // relay U
#define HUMIDIFIER_RELAY_PIN 27    // relay E
#define TEMP_HUM_SENSOR_PIN A0
#define SENSOR_UP_ID 0x29
#define SENSOR_DOWN_ID 0x28

#define IS_RUNNING_ADDR 0
#define LAST_TEMP_ADDR 10
#define LAST_HUM_ADDR 20

struct chamberSensorData {
  float temperatureUp = 0;
  float temperatureDown = 0;
  int humidityUp = 0;
  int humidityDown = 0;
  float temperatureAvg = 0;
  int humidityAvg = 0;
};

struct chamberStatus {
  bool isRunning = false;
  bool cooling = false, heating = false, fan = false, dehumidifier = false, humidifier = false;
  float targetTemperature = 25.0;
  int targetHumidity = 50;
  byte currentStep = 0, nSteps = 1;
  unsigned int elapsedTime = 0, duration = 0;
};

class Stagionatore {
public:
  Stagionatore();
  void begin();
  void run();
  chamberSensorData getSensorData();
  chamberStatus getStatus();
  void stop();
  void startProgram();
  void startManual(float targetTemperature, int targetHumidity);
private:
  void runStagionaturaProgram();
  void readSensors();
  void runProgramStep();
  void fanController();
  void reachedTargetTemperature();
  void decreaseTemperature();
  void increaseTemperature();
  void reachedTargetHumidity();
  void decreaseHumidity();
  void increaseHumidity();
  void turnOffFan();
  void start();
  void getdata(byte *a, byte *b, byte *c, byte *d, int address);


  bool cooling = false, heating = false, fan = false, dehumidifier = false, humidifier = false;

  bool isRunning = false;
  unsigned long elapsedTime = 0;
  byte currentStep;

  StagionaturaProgram currentProgram;
  chamberSensorData currentSensorData;

  unsigned long lastSensorReadTime = 10;                     // Memorizza il tempo dell'ultima lettura dei sensori
  unsigned long currentStepStartTime = 0;                    // Minuti
  unsigned long fanSwitchTime, fridgeOffTime, fridgeOnTime;  // Secondi
  RTC_DS3231 rtc;
  unsigned long currentSeconds;
  DFRobot_SHT20 sht20;

  StagionaturaProgram savedProgram;
};

#endif