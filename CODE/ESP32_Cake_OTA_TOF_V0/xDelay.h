#ifndef xDelay_h
#define xDelay_h

#include <Arduino.h>

class xDelay {
  public:
    xDelay();
    bool delay(unsigned long interval);
  private:
    unsigned long previousMillis;
};

#endif
