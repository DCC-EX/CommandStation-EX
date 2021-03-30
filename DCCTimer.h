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
#if (defined(TEENSYDUINO) && !defined(__IMXRT1062__))
  static void read_mac(byte mac[6]);
  static void read(uint8_t word, uint8_t *mac, uint8_t offset);
#endif
  private:
};

#endif
