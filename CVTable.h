#ifndef CVTable_h
#define CVTable_h
#include <Arduino.h>
class CVTable {
  public:
    static const uint8_t CV_MAX = 255;
    static uint16_t cv[CV_MAX+1];
};

#endif