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
#include "StringBuffer.h"


// nvs values are held as a linked list of key-value pairs.
// The key is an int16 end the value may be int16 or char*

class NVSentry {
  public:
    static NVSentry * first;
    static bool savePending;
    NVSentry * next;
    uint16_t nvsNumber;
    int16_t nvsValue;
    char * nvsStringValue;
    bool bootNeeded;
    
    // Find an existing NVS entry by its number
    static NVSentry * find(uint16_t number) {
      for (auto e=first; e ; e = e->next) {
        if (e->nvsNumber == number) return e;
      }
      return nullptr;
    } 
    
    // Set the numeric value of this NVS entry.
    void set(int16_t value) {
      if (nvsStringValue) {
        free(nvsStringValue);
        nvsStringValue = nullptr;
        savePending=true;
      }
      if (nvsValue != value) {
        nvsValue = value;
        savePending=true;
      }
    }

    // Set the string value of this NVS entry.
void set(const char * value) {
      if (nvsStringValue) {
        free(nvsStringValue);
        nvsStringValue = nullptr;
        savePending=true;
      }
     if (value) {
        nvsStringValue = new char[strlen(value) + 1];
        strcpy(nvsStringValue, value);
        nvsValue=0;
        savePending=true;
      }
     }

    // dump the value as a <C NVS> command
    void dump(Print * stream) {
    if (nvsStringValue)  
         StringFormatter::send(stream,F("<C NVS %3d \"%s\">\n"),nvsNumber, nvsStringValue);
    else StringFormatter::send(stream,F("<C NVS %3d %5d>\n"), nvsNumber, nvsValue);
    }
    
    // Save the value to a stream in a format suitable for reloading later.
  void save(Print * stream) {
    if (nvsStringValue) StringFormatter::send(stream, F("%d=\"%s\","),
                        nvsNumber, nvsStringValue);
    else if (nvsValue) StringFormatter::send(stream, F("%d=%d,"),
         nvsNumber, nvsValue);
  }

  // stream format suitable for web interface
  void streamJSEntry(Print * stream) {
    if (nvsStringValue) {
      StringFormatter::send(stream, F("NVSTable[%d]=\"%s\";\n"),
         nvsNumber, nvsStringValue);
    } else if (nvsValue) {
      StringFormatter::send(stream, F("NVSTable[%d]=%d;\n"),
         nvsNumber, nvsValue);
    }
    if (bootNeeded) {
      StringFormatter::send(stream, F("NVSBootNeeded[%d]=true;\n"), nvsNumber);
    }
  }
 
    NVSentry(int16_t number)  : nvsNumber(number), nvsValue(0), nvsStringValue(nullptr), bootNeeded(false) {
      next = first;
      first = this;
    }
};

NVSentry * NVSentry::first = nullptr;
bool NVSentry::savePending = false;

#ifdef ARDUINO_ARCH_ESP32
#include "Preferences.h"

// Load NVS values from Preferences
void NVSTable::load() {
  Preferences prefs;
  prefs.begin("DCC-EX-NVS", true); // Read-only
  auto savedSize = prefs.getBytesLength("NVSTable");  
  if (savedSize) {
    char buffer[savedSize+1];
    prefs.getBytes("NVSTable", buffer, savedSize);
    buffer[savedSize] = '\0'; // Null-terminate the buffer just in case
    applyChanges(buffer, true); // Load the saved NVS values into the nvs array
  }
  prefs.end();
}

#else
void NVSTable::load() {  
  DIAG(F("NVSTable::load() not implemented on this platform"));
}
#endif


void NVSTable::save() {
  StringBuffer buffer(4096); // Create a buffer to hold the serialized NVS data
  for (auto e = NVSentry::first; e; e = e->next) e->save(&buffer);
  NVSentry::savePending = false; // Reset the save pending flag after saving 

  #ifdef ARDUINO_ARCH_ESP32
  DIAG(F("Saving NVS to Preferences: %s"), buffer.getString());
  // ESP32 Preferences library requires a key-value pair for each entry, so we will store the entire NVS table as a single byte array under the key "NVSTable".
  Preferences prefs;
  prefs.begin("DCC-EX-NVS", false); // Read-write
  prefs.putBytes("NVSTable", buffer.getString(), buffer.getLength());
  prefs.end();
  #else
  DIAG(F("NVSTable::save() not implemented on this platform"));
  #endif
}

bool NVSTable::saveNeeded() {
  return NVSentry::savePending;
}

// SUpport <D NVS> command to show NVS values from DIAG interface
void NVSTable::dump(Print * stream) {
  stream->print(F("<* All non-zero NVS values *>\n"));
  for (auto e=NVSentry::first; e ; e = e->next) e->dump(stream);
}

void NVSTable::dump(Print * stream, uint16_t nvsNumber) {
  auto e = NVSentry::find(nvsNumber);
  if (e) e->dump(stream);
}

void NVSTable::streamJSArray(Print * stream) {
  stream->print(F("NVSTable=[];\nNVSBootNeeded=[];\n"));
   for (auto e=NVSentry::first; e ; e = e->next) e->streamJSEntry(stream);
   stream->print(F("NVSTableBefore=NVSTable.slice();\n"));  
}

void NVSTable::setNVS(uint16_t nvsNumber, int16_t value, bool autosave) {
  auto e = NVSentry::find(nvsNumber);
  if (!e) {
    if (value==0) return; // No need to create an entry for a zero value
    e = new NVSentry(nvsNumber);
  }
  e->set(value); // Set the new value
  if (autosave && NVSentry::savePending) save();
}

void NVSTable::setNVS(uint16_t nvsNumber, const char * value, bool autosave) {
  auto e = NVSentry::find(nvsNumber);
  if (!e) {
    if (!value || value[0]=='\0') return; // No need to create an entry for a null or empty string
    e = new NVSentry(nvsNumber);
  }
  e->set(value); // Set the new string value
  if (autosave && NVSentry::savePending) save();
}

bool NVSTable::hasNVS(uint16_t nvsNumber) {
  return NVSentry::find(nvsNumber)!=nullptr;
}

void NVSTable::applyChanges(char * changes,bool fromNewBoot) {
  enum State:byte { WAITING_FOR_ID, READING_ID, WAITING_FOR_VALUE, READING_VALUE, READING_STRING };
  State state = WAITING_FOR_ID;
  uint16_t nvsid=0;
  int16_t nvsval=0; 
  bool negate=false;
  int stringStart=0;

  for(int pos=0;changes[pos]!='\0';pos++) {
    char c = changes[pos];
    
    switch(state) {
      case WAITING_FOR_ID:
        if (c==',') break; // Ignore separator
        if (c >= '0' && c <= '9') {

          state = READING_ID;
          nvsid=c-'0';
          negate=false;
          break;
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
        setNVS(nvsid, (int16_t)nvsval,false);
        state = WAITING_FOR_ID;
        break;
      case READING_STRING:
        if (c !='"') break;
        // end of string value
        changes[pos] = '\0'; // Null-terminate the string
        setNVS(nvsid, &changes[stringStart], false);
        state = WAITING_FOR_ID; // End of value, reset for next ID
        break;
    }
  }
  if (NVSTable::saveNeeded() && !fromNewBoot) NVSTable::save(); // Save changes if needed and not during boot
}



int16_t NVSTable::getNVS(uint16_t nvsNumber, bool atBoot) {
  auto e=NVSentry::find(nvsNumber);
  if (!e) return 0; // No entry found, return default value of 0
  if (atBoot) e->bootNeeded=true; // Mark this NVS entry as needing to be set during boot  
  return e->nvsValue;
}

const char * NVSTable::getTextNVS(uint16_t nvsNumber) {
    auto e=NVSentry::find(nvsNumber);
  if (!e) return ""; // No entry found, return default value of 0
  e->bootNeeded=true; // Mark this NVS entry as needing to be set during boot  
  return e->nvsStringValue;
}

// Decode a token into its corresponding NVS value. 
// This identifies EXRAIL type 32-bit tokens which may represnt
// NVS[x]+y or just return y.
// This allows things like DELAY(NVS(6)) DELAY(1000+NVS(6)) or DELAY(1000)  in exrail.
int16_t NVSTable::decodeNVSToken(int32_t token, bool atBoot) {
  
  // Token is an nvs reference if first 2 bits are 01, otherwise it's just a number.
  if ((token & 0xC0000000) != 0x40000000) {
      // this is just a number, not an NVS token, return it as is.
      return (int16_t)token;
  }
  uint16_t nvsNumber = (token >> 16) & 0x3FFF;
  auto addition=(int16_t)(token & 0xFFFF);
  auto e=NVSentry::find(nvsNumber);
  if (!e) return addition; // No entry found, return the addition as the value
  if (atBoot) e->bootNeeded=true; // Mark this NVS entry as needing to be set during boot
  return e->nvsValue+addition;
}
