/*
 *  © 202 5Chris Harlow
 *  All rights reserved.
 *  
 *  This file is part of DCC-EX
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

#ifndef Railcom_h
#define Railcom_h
#include "Arduino.h"

typedef void (*ACK_CALLBACK)(int16_t result);

class Railcom {
  public:
    static void anticipate(uint16_t loco, uint16_t cv, ACK_CALLBACK callback);
    static void process(int16_t firstVpin,byte * buffer, byte length );
    static void loop();
  private:
    static const unsigned long POM_READ_TIMEOUT=500; // as per spec
    static uint16_t expectCV,expectLoco;
    static unsigned long expectWait;
    static ACK_CALLBACK expectCallback;
    static const byte MAX_WAIT_FOR_GLITCH=20; // number of dead or empty packets before assuming loco=0 
};

#endif