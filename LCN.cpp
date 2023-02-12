/*
 *  © 2021, Chris Harlow. All rights reserved.
 *  
 *  This file is part of DCC-EX CommandStation-EX 
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

#include "LCN.h"
#include "DIAG.h"
#include "Turnouts.h"
#include "Sensors.h"

int  LCN::id = 0;
Stream * LCN::stream=NULL;
bool LCN::firstLoop=true;

void LCN::init(Stream & lcnstream) {
  stream=&lcnstream; 
  DIAG(F("LCN connection setup")); 
}


// Inbound LCN traffic is postfix notation...   nnnX  where nnn is an id, X is the opcode
void LCN::loop() {
  if (!stream) return;
  if (firstLoop) {
    firstLoop=false;
    stream->println('X');
    return; 
  }
  
  while (stream->available()) {
    int ch = stream->read();
    if (ch >= '0' && ch <= '9') {  // accumulate id value
      id = 10 * id + ch - '0';
    }
    else if (ch == 't' || ch == 'T') { // Turnout opcodes
      if (Diag::LCN) DIAG(F("LCN IN %d%c"),id,(char)ch);
      if (!Turnout::exists(id)) LCNTurnout::create(id);
      Turnout::setClosedStateOnly(id,ch=='t');
      id = 0;
    }
    else if (ch == 'y' || ch == 'Y') { // Turnout opcodes
      if (Diag::LCN) DIAG(F("LCN IN %d%c"),id,(char)ch);
      Turnout::setClosed(id,ch=='y');
      id = 0;
    }
    else if (ch == 'S' || ch == 's') {
      if (Diag::LCN) DIAG(F("LCN IN %d%c"),id,(char)ch);
      Sensor * ss = Sensor::get(id);
      if (!ss) ss = Sensor::create(id, VPIN_NONE, 0); // impossible pin
      ss->setState(ch == 'S');
      id = 0;
    }
    else  id = 0; // ignore any other garbage from LCN
  }
}

void LCN::send(char opcode, int id, bool state) {
   if (stream) {
      StringFormatter::send(stream,F("%c/%d/%d"), opcode, id , state);
      if (Diag::LCN) DIAG(F("LCN OUT %c/%d/%d"), opcode, id , state);
   }
}
