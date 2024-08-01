#ifndef PROGRAM_H
#define PROGRAM_H

// Definizione della classe ProgramStep
class ProgramStep {
public:
  unsigned long duration;   // minutes
  float targetTemperature;  // if >=100 not set
  int targetHumidity;       // if >=100 not set

  ProgramStep(unsigned long dur, float temp, int hum)
    : duration(dur), targetTemperature(temp), targetHumidity(hum) {}

  ProgramStep()
    : duration(0), targetTemperature(0), targetHumidity(0) {}
};

// Definizione della classe StagionaturaProgram
class StagionaturaProgram {
public:
  String name;
  ProgramStep steps[20];  // Array di passi di dimensione massima 10
  byte numSteps;          // Numero effettivo di passi

  StagionaturaProgram(String n)
    : name(n), numSteps(0) {}

  void addStep(unsigned long durationMinutes, float temp, int hum) {
    if (numSteps < 10) {
      steps[numSteps] = ProgramStep(durationMinutes, temp, hum);
      numSteps++;
    }
  }
};

#endif