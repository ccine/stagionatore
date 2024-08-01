#include "chamber_process.h"

Stagionatore::Stagionatore()
  : sht20(&Wire, SHT20_I2C_ADDR), currentProgram("Programma corrente"), savedProgram("Programma salvato") {
}

void Stagionatore::begin() {
  // Inizializzazione sensori temperatura e umidità
  sht20.initSHT20();

  // Inizializzazione dei pin
  pinMode(COOLING_RELAY_PIN, OUTPUT);
  pinMode(HEATING_RELAY_PIN, OUTPUT);
  pinMode(FAN_RELAY_PIN, OUTPUT);
  pinMode(HUMIDIFIER_RELAY_PIN, OUTPUT);

  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
  }

  // TO RESET RTC
  // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  EEPROM.get(IS_RUNNING_ADDR, isRunning);
  Serial.print("Running? ");
  Serial.print(isRunning);
  if (isRunning) {
    // When arduino power off while running set manual mode with last target values
    float targetTemperature;
    int targetHumidity;
    EEPROM.get(LAST_TEMP_ADDR, targetTemperature);
    EEPROM.get(LAST_HUM_ADDR, targetHumidity);
    if (targetTemperature > 0 && targetTemperature < 100 && targetHumidity > 0 && targetHumidity < 100) {
      startManual(targetTemperature, targetHumidity);
    }
  }

  savedProgram.addStep(11 * 60, 22, 85);
  savedProgram.addStep(12 * 60, 20, 75);
  savedProgram.addStep(24 * 60, 20, 75);
  savedProgram.addStep(24 * 60, 18, 76);
  savedProgram.addStep(24 * 60, 16, 76);
  savedProgram.addStep(24 * 60, 14, 80);
  savedProgram.addStep(24 * 60, 12, 85);
  savedProgram.addStep(24 * 60, 12, 83);
  savedProgram.addStep(24 * 60, 12, 82);
  savedProgram.addStep(24 * 60, 10, 81);
  savedProgram.addStep(24 * 60, 10, 80);
}

void Stagionatore::run() {
  // Leggi il tempo corrente
  currentSeconds = rtc.now().unixtime();

  // Esegui readSensors ogni SENSOR_READ_INTERVAL millisecondi
  if (currentSeconds - lastSensorReadTime >= SENSOR_READ_INTERVAL) {
    // Leggi i sensori
    readSensors();
    lastSensorReadTime = currentSeconds;
  }

  // Se sono in running esegui il programma o la modalità auto
  if (isRunning) {
    runStagionaturaProgram();
  } else {
    if (!cooling || currentSeconds - fridgeOnTime >= MIN_WAIT_FRIDGE * 60) {
      reachedTargetTemperature();
    }
    reachedTargetHumidity();
    turnOffFan();
  }
}

void Stagionatore::startProgram() {
  currentProgram = savedProgram;
  start();
}

void Stagionatore::startManual(float targetTemperature, int targetHumidity) {
  StagionaturaProgram newProgram = StagionaturaProgram("Programma manuale");
  newProgram.addStep(0, targetTemperature, targetHumidity);
  currentProgram = newProgram;
  start();
}

void Stagionatore::start() {
  currentStepStartTime = currentSeconds / 60;
  currentStep = 0;
  EEPROM.update(IS_RUNNING_ADDR, true);
  EEPROM.update(LAST_TEMP_ADDR, currentProgram.steps[currentStep].targetTemperature);
  EEPROM.update(LAST_HUM_ADDR, currentProgram.steps[currentStep].targetHumidity);
  isRunning = true;
}

void Stagionatore::stop() {
  isRunning = false;
  EEPROM.update(IS_RUNNING_ADDR, false);
}

// Esegui il programma di stagionatura corrente
void Stagionatore::runStagionaturaProgram() {
  unsigned long currentMinutes = currentSeconds / 60;  // Converti il tempo Unix in minuti
  unsigned long elapsedMinutes = currentMinutes - currentStepStartTime;

  // Ottieni il passo corrente del programma
  ProgramStep currentStepProgram = currentProgram.steps[currentStep];
  elapsedTime = elapsedMinutes;

  runProgramStep();

  // Verifica se il passo corrente è terminato in base al tempo corrente in minuti
  if (currentStepProgram.duration != 0 && elapsedMinutes >= currentStepProgram.duration) {
    currentStep++;
    currentStepStartTime = currentMinutes;  // Resetta il tempo di inizio del passo corrente
    EEPROM.update(LAST_TEMP_ADDR, currentProgram.steps[currentStep].targetTemperature);
    EEPROM.update(LAST_HUM_ADDR, currentProgram.steps[currentStep].targetHumidity);
  }

  // Se il programma è terminato, riporta all'inizio
  if (currentStep >= currentProgram.numSteps) {
    stop();
  }
}

// In base alla temperatura e umidità impostate accendo i relè necessari
void Stagionatore::runProgramStep() {
  float actualTemperature = currentSensorData.temperatureAvg;
  int actualHumidity = currentSensorData.humidityAvg;

  float targetTemperature = currentProgram.steps[currentStep].targetTemperature;
  int targetHumidity = currentProgram.steps[currentStep].targetHumidity;

  // Attiva il ricircolo dell'aria
  fanController();

  // Se il frigo è acceso, aspetto 5 minuti prima di controllare la temperatura
  if (!cooling || currentSeconds - fridgeOnTime >= MIN_WAIT_FRIDGE * 60) {
    // Aggiusta la temperatura
    if (targetTemperature >= 100) {
      // Spegni tutto se la temperatura non è impostata
      reachedTargetTemperature();
    } else if (actualTemperature >= targetTemperature + TEMP_DELTA) {
      // Temperatura troppo alta, accendi il frigo
      decreaseTemperature();
    } else if (actualTemperature <= targetTemperature - TEMP_DELTA) {
      // Temperatura troppo bassa, accendi i riscaldatori
      increaseTemperature();
    } else {
      if ((cooling && actualTemperature <= targetTemperature) || (heating && actualTemperature >= targetTemperature)) {
        // Spegni tutti i relay se la temperatura è giusta
        reachedTargetTemperature();
      }
    }
  }

  // Aggiusta l'umidità
  if (targetHumidity >= 100) {
    // Spegni tutto se l'umidità non è impostata
    reachedTargetHumidity();
  } else if (actualHumidity >= targetHumidity + HUMIDITY_DELTA) {
    // Umidità troppo alta, accendi l'estrattore
    decreaseHumidity();
  } else if (actualHumidity <= targetHumidity - HUMIDITY_DELTA) {
    // Umidità troppo bassa, accendi umidificatore
    increaseHumidity();
  } else {
    if ((dehumidifier && actualHumidity <= targetHumidity) || (humidifier && actualHumidity >= targetHumidity)) {
      // Spegni tutti i relay se la temperatura è giusta
      reachedTargetHumidity();
    }
  }
}

void Stagionatore::readSensors() {
  byte aa, bb, cc, dd;
  float temperature = 0.0;
  float humidity = 0.0;

  // Leggi i valori dal sensore di temperatura e umidità superiore sht20
  float h = sht20.readHumidity();
  // Read temperature as Celsius (the default)
  float t = round(sht20.readTemperature() * 10) / 10.0;

  // Leggi sensore superiore
  getdata(&aa, &bb, &cc, &dd, SENSOR_UP_ID);

  humidity = (float)(((aa & 0x3F) << 8) + bb) / 16384.0 * 100.0;
  temperature = (float)((unsigned)(cc * 64) + (unsigned)(dd >> 2)) / 16384.0 * 165.0 - 40.0;
  currentSensorData.temperatureUp = (temperature + t) / 2.0;
  currentSensorData.humidityUp = (humidity + h) / 2.0;

  // Leggi sensore inferiore
  getdata(&aa, &bb, &cc, &dd, SENSOR_DOWN_ID);

  humidity = (float)(((aa & 0x3F) << 8) + bb) / 16384.0 * 100.0;
  temperature = (float)((unsigned)(cc * 64) + (unsigned)(dd >> 2)) / 16384.0 * 165.0 - 40.0;
  currentSensorData.temperatureDown = temperature;
  currentSensorData.humidityDown = round(humidity);

  // Valore media
  currentSensorData.temperatureAvg = (float)(currentSensorData.temperatureUp + currentSensorData.temperatureDown) / 2.0;
  currentSensorData.humidityAvg = (float)(currentSensorData.humidityUp + currentSensorData.humidityDown) / 2.0;
  // currentSensorData.humidityAvg = currentSensorData.humidityUp;
}

void Stagionatore::fanController() {
  if (humidifier) {
    if (!fan) {
      digitalWrite(FAN_RELAY_PIN, HIGH);
      fan = true;
      fanSwitchTime = currentSeconds;
    }
  } else {
    if (fan) {
      // Attendi x minuti prima di spegnere la ventola
      if (currentSeconds - fanSwitchTime >= FAN_ON_TIME) {
        digitalWrite(FAN_RELAY_PIN, LOW);
        fan = false;
        fanSwitchTime = currentSeconds;
      }
    } else {
      // Attendi x minuti prima di accendere la ventola
      if (currentSeconds - fanSwitchTime >= FAN_OFF_TIME) {
        digitalWrite(FAN_RELAY_PIN, HIGH);
        fan = true;
        fanSwitchTime = currentSeconds;
      }
    }
  }
}

void Stagionatore::reachedTargetTemperature() {
  if (heating || cooling) {
    if (cooling)
      fridgeOffTime = currentSeconds;
    heating = false;
    cooling = false;
    digitalWrite(HEATING_RELAY_PIN, LOW);
    digitalWrite(COOLING_RELAY_PIN, LOW);
  }
}

void Stagionatore::decreaseTemperature() {
  if (!cooling) {
    heating = false;
    digitalWrite(HEATING_RELAY_PIN, LOW);
    // Frigo è stato spento per almeno MIN_WAIT_FRIDGE minuti
    if (currentSeconds - fridgeOffTime >= MIN_WAIT_FRIDGE * 60) {
      cooling = true;
      digitalWrite(COOLING_RELAY_PIN, HIGH);
      fridgeOnTime = currentSeconds;
    }
  }
}

void Stagionatore::increaseTemperature() {
  if (!heating) {
    if (cooling) {
      cooling = false;
      fridgeOffTime = currentSeconds;
    }
    heating = true;
    digitalWrite(HEATING_RELAY_PIN, HIGH);
    digitalWrite(COOLING_RELAY_PIN, LOW);
  }
}

void Stagionatore::reachedTargetHumidity() {
  if (humidifier || dehumidifier) {
    humidifier = false;
    dehumidifier = false;
    digitalWrite(DEHUMIDIFIER_RELAY_PIN, LOW);
    digitalWrite(HUMIDIFIER_RELAY_PIN, LOW);
  }
}

void Stagionatore::decreaseHumidity() {
  if (!dehumidifier) {
    dehumidifier = true;
    humidifier = false;
    digitalWrite(DEHUMIDIFIER_RELAY_PIN, HIGH);
    digitalWrite(HUMIDIFIER_RELAY_PIN, LOW);
  }
}

void Stagionatore::increaseHumidity() {
  if (!humidifier) {
    humidifier = true;
    dehumidifier = false;
    digitalWrite(HUMIDIFIER_RELAY_PIN, HIGH);
    digitalWrite(DEHUMIDIFIER_RELAY_PIN, LOW);
  }
}

void Stagionatore::turnOffFan() {
  if (fan) {
    digitalWrite(FAN_RELAY_PIN, LOW);
    fan = false;
  }
}

void Stagionatore::getdata(byte* a, byte* b, byte* c, byte* d, int address) {
  Wire.beginTransmission(address);
  Wire.write(0);
  Wire.endTransmission();

  Wire.requestFrom(address, 4);
  *a = Wire.read();
  *b = Wire.read();
  *c = Wire.read();
  *d = Wire.read();
}

chamberSensorData Stagionatore::getSensorData() {
  return currentSensorData;
}


chamberStatus Stagionatore::getStatus() {
  float targetTemperature = 25.0;
  int targetHumidity = 50;
  unsigned int duration = 0;
  if (currentProgram.numSteps > 0) {
    ProgramStep currentStepProgram = currentProgram.steps[currentStep];
    targetTemperature = currentStepProgram.targetTemperature;
    targetHumidity = currentStepProgram.targetHumidity;
    duration = currentStepProgram.duration;
  }
  return {
    isRunning, cooling, heating, fan, dehumidifier, humidifier, targetTemperature, targetHumidity, currentStep, currentProgram.numSteps, elapsedTime, duration
  };
}