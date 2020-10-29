/*
 *  © 2013-2016 Gregg E. Berman
 *  © 2020, Chris Harlow. All rights reserved.
 *  © 2020, Harald Barth.
 *
 *  This file is part of Asbelos DCC API
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
#include "EEStore.h"
#include "Turnouts.h"
#include "Sensors.h"
#include "Outputs.h"
#include "DIAG.h"

#if defined(ARDUINO_ARCH_SAMD)
ExternalEEPROM EEPROM;
#endif

void EEStore::init(){
#if defined(ARDUINO_ARCH_SAMD)
    EEPROM.begin(0x50);     // Address for Microchip 24-series EEPROM with all three A pins grounded (0b1010000 = 0x50)
#endif

    eeStore=(EEStore *)calloc(1,sizeof(EEStore));
    
    EEPROM.get(0,eeStore->data);                                       // get eeStore data

    if(strncmp(eeStore->data.id,EESTORE_ID,sizeof(EESTORE_ID))!=0){    // check to see that eeStore contains valid DCC++ ID
        sprintf(eeStore->data.id,EESTORE_ID);                           // if not, create blank eeStore structure (no turnouts, no sensors) and save it back to EEPROM
        eeStore->data.nTurnouts=0;
        eeStore->data.nSensors=0;
        eeStore->data.nOutputs=0;
        EEPROM.put(0,eeStore->data);
    }

    reset();            // set memory pointer to first free EEPROM space
    Turnout::load();    // load turnout definitions
    Sensor::load();     // load sensor definitions
    Output::load();     // load output definitions

}

///////////////////////////////////////////////////////////////////////////////

void EEStore::clear(){

    sprintf(eeStore->data.id,EESTORE_ID);                           // create blank eeStore structure (no turnouts, no sensors) and save it back to EEPROM
    eeStore->data.nTurnouts=0;
    eeStore->data.nSensors=0;
    eeStore->data.nOutputs=0;
    EEPROM.put(0,eeStore->data);

}

///////////////////////////////////////////////////////////////////////////////

void EEStore::store(){
    reset();
    Turnout::store();
    Sensor::store();
    Output::store();
    EEPROM.put(0,eeStore->data);
}

///////////////////////////////////////////////////////////////////////////////

void EEStore::advance(int n){
    eeAddress+=n;
}

///////////////////////////////////////////////////////////////////////////////

void EEStore::reset(){
    eeAddress=sizeof(EEStore);
}
///////////////////////////////////////////////////////////////////////////////

int EEStore::pointer(){
    return(eeAddress);
}
///////////////////////////////////////////////////////////////////////////////

void EEStore::dump(int num) {
    byte b;
    DIAG(F("\nAddr  0x  char\n"));
    for (int n=0 ; n<num; n++) {
	EEPROM.get(n, b);
	DIAG(F("%d     %x    %c\n"),n,b,isprint(b) ? b : ' ');
    }
}
///////////////////////////////////////////////////////////////////////////////

EEStore *EEStore::eeStore=NULL;
int EEStore::eeAddress=0;
