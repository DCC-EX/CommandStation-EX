#include "CVTable.h"
#ifdef ARDUINO_ARCH_ESP32

#include "Preferences.h"
#include "StringFormatter.h"

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
void CVTable::dump(Print * stream) {
  // Dump all non-zero CV values to the provided stream
  for (uint8_t i = 1; i <= CV_MAX; i++) {
    stream->print(F("<* All non-zero CV values\n"));
    if (cv[i] != 0) {
      StringFormatter::send(stream,F("CV %3d = %d\n"), i, cv[i]);
    }
    stream->println(F("*>"));
  }
}

void CVTable::setCV(uint8_t cvNumber, uint16_t value) {
  if (cvNumber <= CV_MAX) {
    cv[cvNumber] = value;
    save(); // Save the updated CV table to Preferences
  }
}

#else
void CVTable::load(){};
void CVTable::save(){};
void CVTable::dump(Print * stream) {
  StringFormatter::send(stream,F("<* CVTable dump not supported on this platform *\n"));
}
void CVTable::setCV(uint8_t cvNumber, uint16_t value) {
  // Do nothing on non-ESP32 platforms
  (void)cvNumber; // Suppress unused parameter warning

}
#endif
