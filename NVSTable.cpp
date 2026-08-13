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

int16_t NVSTable::nvs[NVSTable::NVS_MAX+1];

void NVSTable::load() {
  // Load NVS values from Preferences
  Preferences prefs;
  prefs.begin("nvstable", true); // Read-only
  prefs.getBytes("NVSTable",nvs,sizeof(nvs)); 
  prefs.end();
}

void NVSTable::dump(Print * stream) {
  stream->print(F("<* All non-zero NVS values *>\n"));
  for (int16_t i = 0; i <= NVS_MAX; i++) {
    if (nvs[i]) dump(stream,i);
  }
}

void NVSTable::dump(Print * stream, uint8_t nvsNumber) {
    if (nvs[nvsNumber] == NVS_IS_STRING) {  
      StringFormatter::send(stream,F("<C NVS %3d \"%s\">\n"),nvsNumber, getTextNVS(nvsNumber).c_str());
    }
    else {
      StringFormatter::send(stream,F("<C NVS %3d %5d>\n"), nvsNumber, nvs[nvsNumber]);
    }
}

void NVSTable::streamJSArray(Print * stream) {
  for (int i=0;i<=NVS_MAX;i++) {
    auto value = NVSTable::getNVS(i);
    if (value == NVSTable::NVS_IS_STRING) {
      stream->print('"');
      stream->print(NVSTable::getTextNVS(i).c_str());
      stream->print('"');
    }
    else stream->print(value);
    if (i < NVS_MAX) stream->print(',');
  }
}

void NVSTable::setNVS(uint8_t nvsNumber, int16_t value) {
  if (nvsNumber > NVS_MAX  || nvs[nvsNumber] == value) return; // No change needed
  nvs[nvsNumber] = value;
  // Save NVS values to Preferences
  Preferences prefs;
  prefs.begin("nvstable", false); // Read-write
  prefs.putBytes("NVSTable", nvs, sizeof(nvs));
  prefs.end();
}


void NVSTable::setNVS(uint8_t nvsNumber, String value) {
  if (nvsNumber > NVS_MAX) return;
  char key[15];
  snprintf(key, sizeof(key), "nvsText_%03d", nvsNumber);
  Preferences prefs;
  prefs.begin("nvstable", false);
  // Retrieve the string. If it doesn't exist yet, return an empty string ""
  String storedStr = prefs.getString(key, "");
  if (storedStr != value) {
    prefs.putString(key, value);
    if (nvs[nvsNumber] != NVS_IS_STRING) {
      nvs[nvsNumber] = NVS_IS_STRING; // Mark this NVS entry as a string
      prefs.putBytes("NVSTable", nvs, sizeof(nvs)); // Save the updated NVS table to Preferences
    }
  }
  prefs.end();
}



int16_t NVSTable::getNVS(uint8_t nvsNumber) {
  return nvs[nvsNumber];
}

String NVSTable::getTextNVS(uint8_t nvsNumber) {
  if (nvs[nvsNumber] != NVS_IS_STRING) return String(nvs[nvsNumber]); // Not a string, return the numeric value as a string
  Preferences prefs;
  prefs.begin("nvstable", true);
  char key[15];
  snprintf(key, sizeof(key), "nvsText_%03d", nvsNumber);
  // Retrieve the string. If it doesn't exist yet, return an empty string ""
  String storedStr = prefs.getString(key, "");
  prefs.end();
  return storedStr;
}

#else
void NVSTable::load(){};
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
void NVSTable::setNVS(uint8_t nvsNumber, int16_t value, String str) {
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
  return "";
}
#endif
