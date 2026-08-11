/*
 *  © 2020-2026 Chris Harlow
 *  © 2026 Ross Scanlon
 *  All rights reserved.
 *  
 *  This file is part of DCC-EX API
 *
 *  This is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  It is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with CommandStation.  If not, see <https://www.gnu.org/licenses/>.
 */

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
