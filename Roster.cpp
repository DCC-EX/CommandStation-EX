/*
 *  © 2026 Chris Harlow
 *  All rights reserved.
 *  
 *  This file is part of CommandStation-EX
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

#include "defines.h"
#include "Roster.h"
#include "StringFormatter.h"
#include "DCCEXParser.h"
#include "StringBuffer.h"
#include "DIAG.h"

void Roster::create(int locoId, const char* name, const char* functionMap) {
    auto rr=find(locoId);
    if (!rr) rr = new Roster();
    
    rr->locoId = locoId;
    if (rr->name) delete[] rr->name;
    rr->name = new char[strlen(name) + 1];
    strcpy(rr->name, name);
    if (rr->functionMap) delete[] rr->functionMap;
    rr->functionMap = nullptr;
    if (functionMap && *functionMap) {
        // blank function map, does not allocate memory
        rr->functionMap = new char[strlen(functionMap) + 1];
        strcpy(rr->functionMap, functionMap);
    }
}

void Roster::remove(int locoId) {
    auto rr=find(locoId);
    if (!rr) return;
    rr->locoId=-1;
    if (rr->name) delete[] rr->name;
    rr->name = nullptr;
    if (rr->functionMap ) delete[] rr->functionMap;
    rr->functionMap = nullptr;
}

void Roster::sendIdsToThrottle(Print * stream) {
    stream->print("<jR ");
    for (auto rr = first; rr; rr = rr->next) {
        if (rr->locoId >= 0) {
            stream->print(rr->locoId);
            stream->print(" ");
        }
    }
    stream->println(">\n");
}

void Roster::sendToThrottle(int locoId, Print * stream) {
    auto rr=find(locoId);
    if (rr) rr->sendToThrottle(stream); 
    else StringFormatter::send(stream,F("<jR %d X>\n"), locoId);
}


static const char EEPROM_HEADER[11] = "DCC-EX-NVS";

#ifdef ARDUINO_ARCH_ESP32
#include "Preferences.h"

// Load NVS values from Preferences
void Roster::load() {
  Preferences prefs;
  prefs.begin(EEPROM_HEADER, true); // Read-only
  auto savedSize = prefs.getBytesLength("Roster");  
  if (savedSize>0) {
    char buffer[savedSize+1];
    prefs.getBytes("Roster", buffer, savedSize);
    buffer[savedSize] = '\0'; // Null-terminate the buffer just in case
    DCCEXParser::parse(&USB_SERIAL, buffer); // Load the saved NVS values into the nvs array
    }
  prefs.end();
  DIAG(F("Roster::load completed"));
}

void Roster::save() {
  StringBuffer sb(4096);// ESP32 Preferences library requires a key-value pair for each entry, so we will store the entire NVS table as a single byte array under the key "NVSTable".
  for (auto rr = first; rr; rr = rr->next)
      if (rr->locoId >= 0) rr->sendToThrottle(&sb,true);
  Preferences prefs;
  prefs.begin(EEPROM_HEADER, false); // Read-write
  prefs.putBytes("Roster", sb.getString(), sb.getLength());
  prefs.end();
  DIAG(F("Roster::save completed"));
}

#else

// STM32 version not yet implemented
void Roster::load() {
  DIAG(F("Roster::load() not yet implemented for STM32"));
}

void Roster::save()  {
  DIAG(F("Roster::save() not yet implemented for STM32"));
}
#endif

/*** PRIVATE STATICS *****/
Roster* Roster::first = nullptr;

Roster * Roster::find(int locoId) {
    for (auto rr = first; rr; rr = rr->next) 
        if (rr->locoId == locoId) return rr;
    return nullptr;
}

/******* PRIVATE METHODS *******/

Roster::Roster() {
    next=first;
    first=this;
    locoId=-1;
    name=nullptr;
    functionMap=nullptr;
}

void Roster::sendToThrottle(Print * stream,bool asSave) {
       StringFormatter::send(stream,F("<%cR %d \"%S\" \"%S\" >\n"),
       asSave?'J':'j', locoId, name, functionMap?functionMap:"");
}
 
void Roster::sendToWithrottle(Print * stream) {
    int count=0;
    for (auto rr = first; rr; rr = rr->next)
      if ((rr->locoId >= 0)) count++;
    
    StringFormatter::send(stream,F("RL%d"), count);
    for (auto rr = first; rr; rr = rr->next) { 
      if (rr->locoId >= 0)
        StringFormatter::send(stream,F("]\\[%S}|{%d}|{%c"),
			      rr->name,rr->locoId,
                  rr->locoId<HIGHEST_SHORT_ADDR?'S':'L');
  }
  StringFormatter::send(stream,F("\n"));       
}

const char * Roster::getFunctionNamesForWithrottle(int locoid) {
    auto rr = find(locoid);
    if (rr) return rr->functionMap;
    return nullptr;
}
