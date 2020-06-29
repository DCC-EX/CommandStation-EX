#ifndef HTTPParser_h
#define HTTPParser_h
#include <Arduino.h>
class HTTPParser {
  public:  
    static void parse(Print & stream, byte * cmd);  
};
#endif
