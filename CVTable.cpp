/**********************************************************************

The NVS Table class for DCC-EX Command Station Nodes

**********************************************************************/

#include "CVTable.h"
#include "StringFormatter.h"

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
void CVTable::dump(Print * stream) {
  // Dump all non-zero CV values to the provided stream
  stream->print(F("<* All non-zero CV values \n"));
    for (int16_t i = 1; i <= CV_MAX; i++) {
    if (cv[i] != 0) {
      StringFormatter::send(stream,F("<C CV %3d  %d>\n"), i, cv[i]);
    }
  }
  stream->print(F("*>\n"));
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
  stream->print(F("<* CVs not supported on this platform *>\n"));
}
void CVTable::setCV(uint8_t cvNumber, uint16_t value) {
  // Do nothing on non-ESP32 platforms
  (void)cvNumber; // Suppress unused parameter warning
  (void)value;    // Suppress unused parameter warning
}
#endif
