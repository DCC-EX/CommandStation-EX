#include "CVTable.h"
#ifdef ARDUINO_ARCH_ESP32

#include "Preferences.h"

uint16_t CVTable::cv[CVTable::CV_MAX+1];

void CVTable::load() {
  // Load CV values from Preferences
  Preferences prefs;
  prefs.begin("cvtable", true); // Read-only
  prefs.getBytes("CVTable",cv,sizeof(cv)); 
  prefs.end();
}

void CVTable::save() {
  // Save CV values to Preferences
  Preferences prefs;
  prefs.begin("cvtable", false); // Read-write
  prefs.putBytes("CVTable", cv, sizeof(cv));
  prefs.end();
}
#else
void CVTable::load(){};
void CVTable::save(){};
#endif
