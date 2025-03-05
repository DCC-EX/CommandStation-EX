/*
 *  © 2025 Chris Harlow
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

#include "Railcom.h"
#include "DCC.h"
#include "DCCWaveform.h"

uint16_t Railcom::expectLoco=0;
uint16_t Railcom::expectCV=0;
unsigned long Railcom::expectWait=0;
ACK_CALLBACK Railcom::expectCallback=0;

// anticipate is used when waiting for a CV read from a railcom loco
void Railcom::anticipate(uint16_t loco, uint16_t cv, ACK_CALLBACK callback) { 
    expectLoco=loco;
    expectCV=cv;
    expectWait=millis(); // start of timeout 
    expectCallback=callback;
}

// process is called to handle data buffer sent by collector
void Railcom::process(int16_t firstVpin,byte * buffer, byte length) { 
    //  block,locohi,locolow
    //  block|0x80,data pom read cv
    byte i=0; 
    while (i<length) {
        byte block=buffer[i] & 0x3f;
        byte type=buffer[i]>>6;

        switch (type) {
            // a type=0 record has block,locohi,locolow
            case 0: {
                uint16_t locoid= ((uint16_t)buffer[i+1])<<8 | ((uint16_t)buffer[i+2]);
                DIAG(F("RC3 b=%d l=%d"),block,locoid);
        
                if (locoid==0) DCC::clearBlock(firstVpin+block);
                else DCC::setLocoInBlock(locoid,firstVpin+block,true);
                i+=3;
            }
            break;
       case 2: { // csv value from POM read
            byte value=buffer[i+1];
            if (expectCV && DCCWaveform::getRailcomLastLocoAddress()==expectLoco) {
                DCC::setLocoInBlock(expectLoco,firstVpin+block,false);
                if (expectCallback) expectCallback(value);
                expectCV=0;
            }
            i+=2;
        }
         break;
         default:
          DIAG(F("Unknown RC Collector code %d"),type);
          return;
        }
      }
    }


// loop() is called to detect timeouts waiting for a POM read result
void Railcom::loop() {
    if (expectCV && (millis()-expectWait)> POM_READ_TIMEOUT) { // still waiting 
                expectCallback(-1);
                expectCV=0;
    }
}
