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


 /*
 This class acts as a 3 way coordinator between
  - the DCC waveform cutout generator,
  - the incoming railcom collector notifications
  - and the EXRAIL2 block enter/exit system   
  */
#include "Railcom.h"
#include "DCC.h"
#include "DCCWaveform.h"
#include "EXRAIL2.h"

uint16_t Railcom::expectLoco=0;
uint16_t Railcom::nextLoco=0;

uint16_t Railcom::expectCV=0;
unsigned long Railcom::expectWait=0;
ACK_CALLBACK Railcom::expectCallback=0;

enum ResponseType: byte { 
        ENTER_BLOCK=0x00,  // loco entering block, block id and loco id follow
        EXIT_BLOCK=0x80,   // loco exiting block, block id and loco id follow
        CV_VALUE=0x40,     // cv value read from a POM, cv value follow
        CV_VALUE_LIST=0xC0, // list of cv values read from a POM, cv id and 4 values follow
    };
  
void Railcom::setLoco(byte packet0, byte packet1) {
    // first 2 bits 00=short loco, 11=long loco , 01/10 = accessory
      byte addressType=packet0 & 0xC0;
      if (packet0==0xff) nextLoco=0;  // idle or estop
      else if (addressType==0xC0) nextLoco=((packet0 & 0x3f)<<8) | packet1;
      else if (addressType==0x00) nextLoco=packet0 & 0x3F;
      else nextLoco=0; 
}

uint16_t Railcom::getLoco() {
    return nextLoco; 
}

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
        (void)block; // avoid compiler warning if not using EXRAIL
        byte type=buffer[i] & 0xc0;
        
        switch (type) {
            // a type=0 record has block,locohi,locolow
            case ENTER_BLOCK:
            case EXIT_BLOCK:
             {
                #ifdef EXRAIL_ACTIVE
                uint16_t locoid= ((uint16_t)buffer[i+1])<<8 | ((uint16_t)buffer[i+2]);
                RMFT2::blockEvent(firstVpin+block,locoid,type==ENTER_BLOCK);
                #endif
                i+=3;
            }
            break;
       case CV_VALUE: { // csv value from POM read
            byte value=buffer[i+1];
            if (expectCallback) expectCallback(value);
            expectCallback=0;
            i+=2;
        }
         break;
         default:
          DIAG(F("Unknown RC Collector code 0x%x"),type);
          return;
        }
      }
    }


// loop() is called to detect timeouts waiting for a POM read result
void Railcom::loop() {
    if (expectCallback && (millis()-expectWait)> POM_READ_TIMEOUT) { // still waiting 
                expectCallback(-1);
                expectCallback=0;
    }
}

byte Railcom::cutoutCounter=0;
void Railcom::incCutout() {cutoutCounter++;};
byte Railcom::getCutout() {return cutoutCounter;};
 
