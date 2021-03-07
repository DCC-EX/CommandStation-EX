#ifndef DCCTimer_h
#define DCCTimer_h
#include "Arduino.h"

typedef void (*INTERRUPT_CALLBACK)();

class DCCTimer {
  public:
  static void begin(INTERRUPT_CALLBACK interrupt);
  static void getSimulatedMacAddress(byte mac[6]);
  static bool isPWMPin(byte pin);
  static void setPWM(byte pin, bool high);
  private:
};

#endif
