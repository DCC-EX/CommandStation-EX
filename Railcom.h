#ifndef Railcom_h
#define Railcom_h

#include <Arduino.h>
class Railcom {
  public:
    static  void startCutout();
    static  void interrupt();
  private:
    static byte interruptState;
    static byte bitsReceived;
    static const byte MAX_BUFFER=20;
    static byte buffer[MAX_BUFFER];    
};

#endif
