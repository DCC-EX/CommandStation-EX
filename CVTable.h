/**********************************************************************

The NVS Table class for DCC-EX Command Station Nodes

**********************************************************************/

#ifndef CVTable_h
#define CVTable_h
#include <Arduino.h>
class CVTable {
  public:
    static const uint8_t CV_MAX = 255;
    static uint16_t cv[CV_MAX+1];
    static void load();
    static void save();
    static void dump(Print * stream);
    static void setCV(uint8_t cvNumber, uint16_t value);
};

#endif