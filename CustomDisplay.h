#include "RelayButton.h"
#include <Adafruit_GFX.h>
#include <TouchScreen.h>
#include "Program.h"

#define TS_MINX 150
#define TS_MINY 120
#define TS_MAXX 920
#define TS_MAXY 940
#define MINPRESSURE 10
#define MAXPRESSURE 1000

#define BLACK 0x0000
#define BLUE 0x001F
#define RED 0xF800
#define GREEN 0x07E0
#define CYAN 0x07FF
#define MAGENTA 0xF81F
#define YELLOW 0xFFE0
#define WHITE 0xFFFF
#define RGB(r, g, b) (((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3))
#define GREY RGB(127, 127, 127)
#define DARKGREY RGB(64, 64, 64)
#define TURQUOISE RGB(0, 128, 128)
#define PINK RGB(255, 128, 192)
#define OLIVE RGB(128, 128, 0)
#define PURPLE RGB(128, 0, 128)
#define AZURE RGB(0, 128, 255)
#define ORANGE RGB(255, 128, 64)


class CustomDisplay {
public:
  CustomDisplay();
  void initDisplay(int coolingRelay, int heatingRelay, int fanRelay, int dehumidifierRelay, int humidifierRelay, RunningStep rs);
  void update(float temperatureUp, int humidityUp, float temperatureDown, int humidityDown);
  void updateRunning(bool cooling, bool heating, bool fan, bool dehumidifier, bool humidifier, int targetTemp, int targetHum, int currentStep = -1, int nSteps = -1, int elapsedTime = 0, int duration = -1);
  float getSelectedTemperature();
  int getSelectedHumidity();
  bool isRunning();
  bool isProgramMode();
  void stop();

private:
  int screen;                                                                                 // 0: Home, 1: Auto, 2: Program, 3: Test, 4: Running
  MCUFRIEND_kbv tft;                                                                          // Display variable
  RelayButton cooling_btn, heating_btn, fan_btn, dehum_btn, hum_btn;                          // Test button to switch relays
  Adafruit_GFX_Button home_screen_btn, auto_screen_btn, program_screen_btn, test_screen_btn;  // Change screen buttons
  Adafruit_GFX_Button add1T_btn, sub1T_btn, add5T_btn, sub5T_btn;                             // Change temperature buttons
  Adafruit_GFX_Button add5H_btn, sub5H_btn, add10H_btn, sub10H_btn;                           // Change humidity buttons
  Adafruit_GFX_Button start_btn, stop_btn;
  int pixel_x, pixel_y;
  //const int XP = 8, XM = A2, YP = A3, YM = 9;  //240x320 ID=0x9341 // DISPLAY GRANDE
  //const int TS_LEFT = 940, TS_RT = 113, TS_TOP = 64, TS_BOT = 900; // DISPLAY GRANDE
  const int XP = 6, XM = A2, YP = A1, YM = 7;                        // DISPLAY PICCOLO
  const int TS_LEFT = 175, TS_RT = 943, TS_TOP = 953, TS_BOT = 203;  // DISPLAY PICCOLO
  TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);
  float lastTUp = -1;
  float lastTDown = -1;
  int lastHUp = -1;
  int lastHDown = -1;
  int testModeClick;
  bool loadedScreen = false;
  int selectedTemp = 20, selectedHum = 60;
  bool changedSelectedAuto = false;
  bool running = false;
  bool programMode = false;
  bool lastCooling = false, lastHeating = false, lastFan = false, lastDehumidifier = false, lastHumidifier = false;
  int lastStep = -1, lastTime = -1;

  // PORTRAIT  CALIBRATION     240 x 320
  //x = map(p.x, LEFT=940, RT=113, 0, 240)
  //y = map(p.y, TOP=64, BOT=900, 0, 320)

  // LANDSCAPE CALIBRATION     320 x 240
  //x = map(p.y, LEFT=64, RT=900, 0, 320)
  //y = map(p.x, TOP=113, BOT=940, 0, 240)

  bool Touch_getXY();
  void showValues(float temperatureUp, int humidityUp, float temperatureDown, int humidityDown);
  void loadHomeScreen();
  void loadAutoScreen();
  void loadProgramScreen();
  void loadTestScreen();
  void loadRunningScreen();
  void handleUserInput();
  void handleHomeScreen(bool down);
  void handleAutoScreen(bool down);
  void handleProgramScreen(bool down);
  void handleTestScreen(bool down);
  void handleRunningScreen(bool down);
};