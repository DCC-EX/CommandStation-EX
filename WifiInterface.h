
#ifndef WifiInterface_h
#define WifiInterface_h
#include <WiFiEsp.h> 
#include "DCCEXParser.h"

class WifiInterface {

 public:
    static void setup();
    static void loop(DCCEXParser & parser);
    
  private:
    static WiFiEspServer server;
    static WiFiEspClient client;
    static bool connected;
    static bool haveClient;
};

#endif
