#include <Arduino.h>
#include <Wire.h>
#include "SSD1306.h"


SSD1306 display(0x3c, 21, 22);

long lastSwitch = 0;
int stage = 0;


void setup() {
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);
}


void loop() {
  long now = millis();

  if (now - lastSwitch > 2000) {
    stage = (stage + 1) % 2;  // переключение между 0 и 1
    lastSwitch = now;
  }

  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);

  if (stage == 0) {
    display.drawString(0, 0, "111111");
  } else {
    display.drawString(0, 0, "222222");
  }

  display.display();
}