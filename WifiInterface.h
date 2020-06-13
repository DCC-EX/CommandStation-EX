
#ifndef WifiInterface_h
#define WifiInterface_h
#include "DCCEXParser.h"
#include "MemStream.h"

class WifiInterface {

 public:
    static void setup();
    static void loop();
    
  private:
    static DCCEXParser parser;
    static bool setup2();
    static bool checkForOK(const int timeout, char * search);
    static bool connected;
    static byte loopstate;
    static int  datalength;
    static int connectionId;
    static const byte MAX_WIFI_BUFFER=64;
    static byte buffer[MAX_WIFI_BUFFER];
    static MemStream  streamer;
};

#endif
