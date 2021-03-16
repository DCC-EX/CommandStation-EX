#ifndef LCN_h
#define LCN_h
#include <Arduino.h>

class LCN {
  public: 
    static void init(Stream & lcnstream);
    static void loop();
    static void send(char opcode, int id, bool state);
  private :
    static bool firstLoop; 
    static Stream * stream; 
    static int id;
};

#endif
