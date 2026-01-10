#ifndef DCCConsist_h
#define DCCConsist_h

#include <Arduino.h>
class DCCConsist {

    public:
      static bool parse(Print * stream, byte paramCount, int16_t p[]);

      // deteles any consist chain containing locoid 
      static void deleteAnyConsist(int16_t locoid);
      
      //add loco anywhere in consist (may create consist) see EXRAIL.
      static bool addLocoToConsist(uint16_t consistId,uint16_t locoid, bool revesed);

};
#endif
