
#ifndef WifiInterface_h
#define WifiInterface_h
#include "DCCEXParser.h"

class WifiInterface {

 public:
    static void setup();
    static void loop(DCCEXParser & parser);
    
  private:
    static bool setup2();
    static bool checkForOK(const int timeout, char * search);
    static bool connected;
    static const byte MAX_BUFFER=64;
    static byte inboundBuffer[MAX_BUFFER];
};

#endif
