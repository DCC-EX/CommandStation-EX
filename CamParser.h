#ifndef CamParser_H
#define CamParser_H
#include <Arduino.h>
#include "IODevice.h"

class CamParser {
   public:
   static bool parseN(Print * stream, byte paramCount, int16_t p[]);
};


#endif