#include "RelayButton.h"
#include <Arduino.h>

#define BLACK 0x0000
#define BLUE 0x001F
#define RED 0xF800
#define GREEN 0x07E0
#define CYAN 0x07FF
#define MAGENTA 0xF81F
#define YELLOW 0xFFE0
#define WHITE 0xFFFF

RelayButton::RelayButton() {}

void RelayButton::initButton(MCUFRIEND_kbv *gfx, int16_t x, int16_t y, const char *label, uint8_t relayPin) {
  this->gfx = gfx;
  this->x = x;
  this->y = y;
  this->label = label;
  this->relayPin = relayPin;
  pinMode(relayPin, OUTPUT);
}

bool RelayButton::contains(int16_t x, int16_t y) {
  return (x >= this->x && x <= (this->x + this->width) && y >= this->y && y <= (this->y + this->height));
}

void RelayButton::drawButton() {
  gfx->fillRoundRect(x, y, width, height, 10, lastState ? GREEN : WHITE);
  gfx->drawRoundRect(x, y, width, height, 10, BLACK);
  gfx->setTextSize(2);
  gfx->setTextColor(BLACK);
  gfx->setCursor(x + 20, y + 16);
  gfx->print(label);
}

void RelayButton::update(bool pressed, int16_t x, int16_t y) {
  boolean touched = contains(x, y);
  //press(pressed && touched);

  if (pressed && touched) {
    this->btn_pressed = true;
  } else if (!pressed && this->btn_pressed) {
    this->btn_pressed = false;
    this->justReleased = true;
  }

  if (this->justReleased) {
    lastState = !lastState;
    digitalWrite(relayPin, lastState ? HIGH : LOW);
    drawButton();
    this->justReleased = false;
  }
}
