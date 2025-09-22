#include "xDelay.h"

xDelay::xDelay() {
  previousMillis = 0;
}

bool xDelay::delay(unsigned long interval) {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    return true;
  }
  return false;
}
