#ifndef RELAYBUTTON_H
#define RELAYBUTTON_H

#include <MCUFRIEND_kbv.h>

class RelayButton {
public:
  RelayButton();
  void initButton(Adafruit_GFX *gfx, int16_t x, int16_t y, const char *label, uint8_t relayPin);
  void update(bool pressed, int16_t pixel_x, int16_t pixel_y);
  void drawButton();

private:
  MCUFRIEND_kbv *gfx;
  int16_t x, y;
  uint16_t width = 150, height = 50;
  const char *label;
  uint8_t relayPin;
  bool lastState = false;
  bool btn_pressed = false;
  bool justReleased = false;

  bool contains(int16_t x, int16_t y);
};

#endif