#include "chamber_process.h"

Stagionatore::Stagionatore()
  : sht20(&Wire, SHT20_I2C_ADDR), currentProgram("Programma 1") {
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
  // TO RESET rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  /*
  RunningStep runningStep = RunningStep();
  if (isRunningSaved == 1) {
    currentSeconds = rtc.now().unixtime();
    unsigned long currentMinutes = currentSeconds / 60;  // Converti il tempo Unix in minuti
    unsigned long elapsedMinutes = currentMinutes - currentStepStartTime;
    runningStep = RunningStep(currentProgram.steps[currentStepIndex], elapsedMinutes);
  }*/

  currentProgram.addStep(11 * 60, 22, 85);
  currentProgram.addStep(12 * 60, 20, 75);
  currentProgram.addStep(24 * 60, 20, 75);
  currentProgram.addStep(24 * 60, 18, 76);
  currentProgram.addStep(24 * 60, 16, 76);
  currentProgram.addStep(24 * 60, 14, 80);
  currentProgram.addStep(24 * 60, 12, 85);
  currentProgram.addStep(24 * 60, 12, 83);
  currentProgram.addStep(24 * 60, 12, 82);
  currentProgram.addStep(24 * 60, 10, 81);
  currentProgram.addStep(24 * 60, 10, 80);
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
    if (isProgramMode) {
      // Program mode
      runStagionaturaProgram();
    } else {
      // Manual mode
      runProgramStep();
    }
    if (isRunningSaved == 0) {
      isRunningSaved = 1;
      currentStepIndex = 0;
      unsigned long currentMinutes = currentSeconds / 60;
      currentStepStartTime = currentMinutes;
      // EEPROM.put(IS_RUNNING_ADDR, isRunningSaved);
      // EEPROM.put(STEP_INDEX_ADDR, currentStepIndex);
      // EEPROM.put(STEP_START_ADDR, currentStepStartTime);
    }
  } else {
    if (!cooling || currentSeconds - fridgeOnTime >= MIN_WAIT_FRIDGE * 60) {
      reachedTargetTemperature();
    }
    reachedTargetHumidity();
    turnOffFan();
    if (isRunningSaved == 1) {
      isRunningSaved = 0;
      // EEPROM.put(IS_RUNNING_ADDR, isRunningSaved);
    }
  }
}

void Stagionatore::startProgram() {
  currentStepStartTime = currentSeconds / 60;
  currentStepIndex = 0;
  isProgramMode = true;
  isRunning = true;
}

void Stagionatore::startManual(float targetTemperature, int targetHumidity) {
  this->targetTemperature = targetTemperature;
  this->targetHumidity = targetHumidity;
  isProgramMode = false;
  isRunning = true;
}

chamberSensorData Stagionatore::getSensorData() {
  return currentSensorData;
}


chamberStatus Stagionatore::getStatus() {
  return {
    isRunning, isProgramMode, cooling, heating, fan, dehumidifier, humidifier, targetTemperature, targetHumidity, currentStepIndex, nSteps, elapsedTime, duration
  };
}

void Stagionatore::stop() {
  isRunning = false;
}

// Esegui il programma di stagionatura corrente
void Stagionatore::runStagionaturaProgram() {
  unsigned long currentMinutes = currentSeconds / 60;  // Converti il tempo Unix in minuti
  unsigned long elapsedMinutes = currentMinutes - currentStepStartTime;

  // Ottieni il passo corrente del programma
  ProgramStep currentStepProgram = currentProgram.steps[currentStepIndex];
  targetTemperature = currentStepProgram.targetTemperature;
  targetHumidity = currentStepProgram.targetHumidity;
  nSteps = currentProgram.numSteps;
  elapsedTime = elapsedMinutes;
  duration = currentStepProgram.duration;

  runProgramStep();

  // Verifica se il passo corrente è terminato in base al tempo corrente in minuti
  if (elapsedMinutes >= currentStepProgram.duration) {
    currentStepIndex++;
    currentStepStartTime = currentMinutes;  // Resetta il tempo di inizio del passo corrente
    // EEPROM.put(STEP_INDEX_ADDR, currentStepIndex);
    // EEPROM.put(STEP_START_ADDR, currentStepStartTime);
  }

  // Se il programma è terminato, riporta all'inizio
  if (currentStepIndex >= currentProgram.numSteps) {
    isRunningSaved = 0;
    // EEPROM.put(IS_RUNNING_ADDR, isRunningSaved);
    currentStepIndex = 0;
    stop();
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

// In base alla temperatura e umidità impostate accendo i relè necessari
void Stagionatore::runProgramStep() {
  float actualTemperature = currentSensorData.temperatureAvg;
  int actualHumidity = currentSensorData.humidityAvg;

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