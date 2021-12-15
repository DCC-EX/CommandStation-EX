 /*
 *  © 2021, Chris Harlow. All rights reserved.
 *  
 *  This file is part of DCC++EX
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

#include "SerialManager.h"
 #include "DCCEXParser.h"
  SerialManager * SerialManager::first=NULL;

  SerialManager::SerialManager(HardwareSerial * myserial) {
    serial=myserial;
    next=first;
    first=this;
    bufferLength=0;
    myserial->begin(115200);
    inCommandPayload=false; 
  } 

void SerialManager::init() {
#ifdef SERIAL3_COMMANDS
    new SerialManager(&Serial3);
#endif
#ifdef SERIAL2_COMMANDS
    new SerialManager(&Serial2);
#endif
#ifdef SERIAL1_COMMANDS
    new SerialManager(&Serial1);
#endif
   new SerialManager(&Serial);
}

void SerialManager::broadcast(RingStream * ring) {
    for (SerialManager * s=first;s;s=s->next) s->broadcast2(ring);
}
void SerialManager::broadcast2(RingStream * ring) {
    ring->printBuffer(serial);
}

void SerialManager::loop() {
    for (SerialManager * s=first;s;s=s->next) s->loop2();
}

void SerialManager::loop2() {
    while (serial->available()) {
        char ch = serial->read();
        if (ch == '<') {
            inCommandPayload = true;
            bufferLength = 0;
            buffer[0] = '\0';
        }
        else if (ch == '>') {
            buffer[bufferLength] = '\0';
            DCCEXParser::parse(serial, buffer, NULL); 
            inCommandPayload = false;
            break;
        }
        else if (inCommandPayload) {
            if (bufferLength <  (COMMAND_BUFFER_SIZE-1)) buffer[bufferLength++] = ch;
        }
    }
    
}