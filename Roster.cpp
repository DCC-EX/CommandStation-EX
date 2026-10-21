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
#include "DIAG.h"
#include "StringBuffer.h"

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
    stream->print("<jR");
    for (auto rr = first; rr; rr = rr->next) {
        if (rr->locoId >= 0) {
            stream->print(" ");
            stream->print(rr->locoId);
        }
    }
    stream->println(">\n");
}

void Roster::sendToThrottle(int locoId, Print * stream) {
    auto rr=find(locoId);
    if (rr) rr->sendToThrottle(stream); 
    else StringFormatter::send(stream,F("<jR %d X>\n"), locoId);
}


#ifdef ARDUINO_ARCH_ESP32
#include <SPIFFS.h>

static File mountRosterFilesystem(bool write) {
  static bool mounted = false;
  if (!mounted) {
    mounted = SPIFFS.begin(true);
    if (!mounted) DIAG(F("Roster: failed to mount SPIFFS"));
  }
  auto rosterFile = SPIFFS.open("/Roster.txt", write? FILE_WRITE : FILE_READ);
  if (rosterFile) return rosterFile;

  DIAG( F("Roster: /Roster.txt %S"),
    write ? F("failed to open for write") : F("not found"));
  return File();
}

void Roster::load() {
  auto rosterFile = mountRosterFilesystem(false);
  if (!rosterFile) return;

  StringBuffer buffer(500);
  // Note all lines end \n and dont care about line length beyond buffer size
  while (rosterFile.available()) {
    auto nextByte = rosterFile.read();
    if (nextByte < 0) break; // end of file 

    if (nextByte == '\n') {
        DCCEXParser::parse(&USB_SERIAL, buffer.getString());
        buffer.flush();
      }
     else buffer.write(nextByte);
    }
  rosterFile.close();
  DIAG(F("Roster::load completed"));
}

void Roster::save() {
  auto rosterFile = mountRosterFilesystem(true);
  if (!rosterFile) return;

  for (auto rr = first; rr; rr = rr->next)
      if (rr->locoId >= 0) rr->sendToThrottle(&rosterFile,true);
  rosterFile.flush();
  auto writeFailed = rosterFile.getWriteError();
  rosterFile.close();
  if (writeFailed) DIAG(F("Roster: failed to write saved roster file"));
  else DIAG(F("Roster::save completed"));
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
       StringFormatter::send(stream,F("<%cR %d \"%S\" \"%S\">\n"),
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

// Prints a quoted JS string literal, escaping quotes, backslashes and control chars.
static void printJsString(Print * stream, const char * s) {
    stream->print('"');
    for (; s && *s; s++) {
        char c = *s;
        if (c == '"' || c == '\\') { stream->print('\\'); stream->print(c); }
        else if (c == '\n') stream->print("\\n");
        else if (c == '\r') stream->print("\\r");
        else if (c == '\t') stream->print("\\t");
        else if ((unsigned char)c < 0x20) stream->print(' ');
        else stream->print(c);
    }
    stream->print('"');
}

// Emits: var roster=[];roster[3]={name:"Loco",functions:"Light/*Horn"};...
void Roster::sendAsJavascript(Print * stream) {
    stream->print(F("var roster=[];\n"));
    for (auto rr = first; rr; rr = rr->next) {
        if (rr->locoId < 0) continue;
        stream->print(F("roster["));
        stream->print(rr->locoId);
        stream->print(F("]={name:"));
        printJsString(stream, rr->name);
        stream->print(F(",functions:"));
        printJsString(stream, rr->functionMap ? rr->functionMap : "");
        stream->print(F("};\n"));
    }
}

const char * Roster::getFunctionNamesForWithrottle(int locoid) {
    auto rr = find(locoid);
    if (rr) return rr->functionMap;
    return nullptr;
}
