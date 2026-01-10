 /*
 *  © 2021 Chris Harlow
 *  All rights reserved.
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

#ifndef SerialManager_h
#define SerialManager_h

#include "Arduino.h"
#include "defines.h"


// We size this to keep max two commands of maximum lenght
// in the COMMAND_BUFFER. Most commands are shorter, the
// <C WIFI SSID PASSWORD> can be 106 bytes. We do not
// support that on AVR
// AVR:   32bytes+32bytes
//        <foo  ><bar   >
// Other: 128bytes+128bytes
//        <fooo   ><baar  >
#ifndef COMMAND_BUFFER_SIZE
#ifdef ARDUINO_ARCH_AVR
 #define COMMAND_BUFFER_SIZE 64
#else
 #define COMMAND_BUFFER_SIZE 256
#endif
#endif

class SerialManager {
public:
  static void init();
  static void loop();
  static void broadcast(char * stringBuffer);
  
private:  
  static SerialManager * first;
  SerialManager(Stream * myserial);
  void loop2();
  void broadcast2(char * stringBuffer);
  Stream * serial;
  SerialManager * next;
  byte bufferLength;
  byte buffer[COMMAND_BUFFER_SIZE]; 
  byte inCommandPayload;
};
#endif
