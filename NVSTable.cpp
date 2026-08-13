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

#include "NVSTable.h"
#include "StringFormatter.h"

#ifdef ARDUINO_ARCH_ESP32

#include "Preferences.h"

uint16_t NVSTable::nvs[NVSTable::NVS_MAX+1];

void NVSTable::load() {
  // Load NVS values from Preferences
  Preferences prefs;
  prefs.begin("nvstable", true); // Read-only
  prefs.getBytes("NVSTable",nvs,sizeof(nvs)); 
  prefs.end();
}

void NVSTable::save() {
  // Save NVS values to Preferences
  Preferences prefs;
  prefs.begin("nvstable", false); // Read-write
  prefs.putBytes("NVSTable", nvs, sizeof(nvs));
  prefs.end();
}
void NVSTable::dump(Print * stream) {
  // Dump all non-zero NVS values to the provided stream
  stream->print(F("<* All non-zero NVS values \n"));
    for (int16_t i = 1; i <= NVS_MAX; i++) {
    if (nvs[i] != 0) {
      StringFormatter::send(stream,F("<C NVS %3d  %d>\n"), i, nvs[i]);
    }
  }
  stream->print(F("*>\n"));
}


void NVSTable::setNVS(uint8_t nvsNumber, uint16_t value) {
  if (nvsNumber <= NVS_MAX) {
    nvs[nvsNumber] = value;
    save(); // Save the updated NVS table to Preferences
  }
}


void NVSTable::setNVS(uint8_t nvsNumber, String value) {
  if (nvsNumber <= NVS_MAX) {
    char key[15];
    snprintf(key, sizeof(key), "nvsText_%03d", nvsNumber);
    Preferences prefs;
    prefs.begin("nvstable", false);
// Retrieve the string. If it doesn't exist yet, return an empty string ""
    String storedStr = prefs.getString(key, "");
    if (storedStr != value) {
      prefs.putString(key, value);
      nvs[nvsNumber] = INT16_MAX;
      save();
    }
    prefs.end();
  }
}



int16_t NVSTable::getNVS(uint8_t nvsNumber) {
  return nvs[nvsNumber];
}

String NVSTable::getTextNVS(uint8_t nvsNumber) {
  if (nvs[nvsNumber] == INT16_MAX) {
    Preferences prefs;
    prefs.begin("nvstable", true);
    char key[15];
    snprintf(key, sizeof(key), "nvstext_%03d", nvsNumber);
    // Retrieve the string. If it doesn't exist yet, return an empty string ""
    String storedStr = prefs.getString(key, "");
    prefs.end();
    return storedStr;
  }
  else {
    return String(nvs[nvsNumber]);
  }
}

#else
void NVSTable::load(){};
void NVSTable::save(){};
void NVSTable::dump(Print * stream) {
  stream->print(F("<* CVs not supported on this platform *>\n"));
}
/*
void NVSTable::setNVS(uint8_t nvsNumber, uint16_t value) {
  // Do nothing on non-ESP32 platforms
  (void)nvsNumber; // Suppress unused parameter warning
  (void)value;    // Suppress unused parameter warning
}
*/
void NVSTable::setNVS(uint8_t nvsNumber, uint16_t value, String str) {
  (void)nvsNumber;
  (void)value;
  (void)str;
}
int16_t NVSTable::getNVS(uint8_t nvsNumber) {
  (void)nvsNumber;
  return INT16_MIN;
}
String NVSTable::getTextNVS(uint8_t nvsNumber) {
  (void)nvsNumber;
  return "<* CVs not supported on this platform *>";
}
#endif
