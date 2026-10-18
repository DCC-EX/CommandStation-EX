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
    else StringFormatter::send(stream,F("<jR %s X>\n"), locoId);
}

void Roster::save() {
    // Implementation for saving the roster
}
void Roster::load() {
    // Implementation for loading the roster
}

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

void Roster::sendToThrottle(Print * stream) {
       StringFormatter::send(stream,F("<jR %d \"%S\" \"%S\" >\n"),
        locoId, name, functionMap?functionMap:"");
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
