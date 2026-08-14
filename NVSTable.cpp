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
#include "DIAG.h"
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
    auto value = nvs[i];
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
  if (nvs[nvsNumber] == value) return; // No change needed
  nvs[nvsNumber] = value;
  // Save NVS values to Preferences
  Preferences prefs;
  prefs.begin("nvstable", false); // Read-write
  prefs.putBytes("NVSTable", nvs, sizeof(nvs));
  prefs.end();
}


void NVSTable::setNVS(uint8_t nvsNumber, String value) {
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

void NVSTable::applyChanges(const String& changes) {
  enum State:byte { WAITING_FOR_ID, READING_ID, WAITING_FOR_VALUE, READING_VALUE, READING_STRING };
  State state = WAITING_FOR_ID;
  uint16_t nvsid=0;
  int16_t nvsval=0; 
  bool negate=false;
  int stringStart=0;

  for(int pos=0;pos<changes.length();pos++) {
    char c = changes[pos];
    
    switch(state) {
      case WAITING_FOR_ID:
        if (c==',') break; // Ignore separator
        if (c >= '0' && c <= '9') {
          state = READING_ID;
          nvsid=c-'0';
        }
        break;

      case READING_ID:
        if (c == '=') {
          state = WAITING_FOR_VALUE;
          nvsval=0;
          negate = false;
          break;
        }
        // assume syntax is valid 
        nvsid = 10 * nvsid + (c - '0');
        break;
      case WAITING_FOR_VALUE:
        if (c=='-') {
          negate = true;
          state=READING_VALUE;
          break;
        }
        if (c == '"') {
          state = READING_STRING;
          stringStart = pos + 1; // Start of string value
          break;
        }
        // assume it's a digit, fall through to READING_VALUE
        [[fallthrough]];
      case READING_VALUE:
        if (c >= '0' && c <= '9') {
          nvsval = 10 * nvsval + (c - '0');
          break;
        }
        // end of numeric value 
        if (negate) nvsval = -nvsval;
        setNVS((uint8_t)nvsid, (int16_t)nvsval);
        state = WAITING_FOR_ID;
        break;
      case READING_STRING:
        if (c !='"') break;
        // end of string value
        String strValue = changes.substring(stringStart, pos);
        setNVS((uint8_t)nvsid, strValue);
        state = WAITING_FOR_ID; // End of value, reset for next ID
        break;
    }
  }
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

// Decode a token into its corresponding NVS value. 
// This identifies EXRAIL type 32-bit tokens which may represnt
// NVS[x]+y or just return y.
// This allows things like DELAY(NVS(6)) DELAY(1000+NVS(6)) or DELAY(1000)  in exrail.
int16_t NVSTable::decodeNVSToken(int32_t token) {
  
  if ((token >> 24)!=0x7E) {
      // this is just a number, not an NVS token, return it as is.
      return (int16_t)token;
  }
  byte nvsNumber = (token >> 16) & 0xFF;
  auto addition=(int16_t)(token & 0xFFFF);
   return nvs[nvsNumber]+addition;
}

#else
void NVSTable::load(){};
void NVSTable::dump(Print * stream) {
  stream->print(F("<* CVs not supported on this platform *>\n"));
}
int16_t NVSTable::decodeNVSToken(int32_t token) {
  return token&0xFFFF;
}

void NVSTable::setNVS(uint8_t nvsNumber, int16_t value, String str) {
  (void)nvsNumber;
  (void)value;
  (void)str;
}
void NVSTable::applyChanges(const String& changes) {
  (void)changes;
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
