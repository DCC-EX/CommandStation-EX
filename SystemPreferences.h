#ifndef SYSTEMPREFERENCES_H
#define SYSTEMPREFERENCES_H
#include <Arduino.h>
#include "StringFormatter.h"


class SystemPreferences {
public:
  #define SYSPref(name,id) static const int16_t name=id;
  #include "SystemPreferencesList.h"
  #undef SYSPref

  static void streamSystemPreferenceIds(Print * stream) {
    stream->print(F("sysNVS={};"));
    #define SYSPref(name,id) wsp(stream,#name,id);
    #include "SystemPreferencesList.h"
    #undef SYSPref
  }
private:
  static void wsp(Print * stream,const char *name,int16_t value) {
    StringFormatter::send(stream,F("sysNVS[\"%s\"]=%d;"), name, value);
  }
 
};
#endif
