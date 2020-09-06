/*
 *  EEStore.cpp
 * 
 *  This file is part of CommandStation.
 *
 *  CommandStation is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  CommandStation is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with CommandStation.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "EEStore.h"

#include "Outputs.h"
#include "Sensors.h"
#include "Turnouts.h"

#if defined(ARDUINO_ARCH_SAMD) || defined(ARDUINO_ARCH_SAMC)
#include <SparkFun_External_EEPROM.h>
ExternalEEPROM EEPROM;
#else
#include <EEPROM.h>
#endif

void EEStore::init(Print* stream){
#if defined(ARDUINO_ARCH_SAMD) || defined(ARDUINO_ARCH_SAMC)
  // Address for Microchip 24-series EEPROM with all three address pins grounded 
  // (0b1010000 = 0x50)
  EEPROM.begin(0x50);     
#endif

  eeStore=(EEStore *)calloc(1,sizeof(EEStore));
  
  EEPROM.get(0,eeStore->data);

  // check to see that eeStore contains valid DCC++ ID
  if(strncmp(eeStore->data.id,EESTORE_ID,sizeof(EESTORE_ID))!=0){    
    // if not, create blank eeStore structure (no turnouts, no sensors) and save 
    // it back to EEPROM

    // TODO(davidcutting42@gmail.com): replace with EEStore::clear();
    sprintf(eeStore->data.id,EESTORE_ID);                           
    eeStore->data.nTurnouts=0;
    eeStore->data.nSensors=0;
    eeStore->data.nOutputs=0;
    EEPROM.put(0,eeStore->data);
  }

  reset();            // set memory pointer to first free EEPROM space
  Turnout::load();    // load turnout definitions
  Sensor::load(stream);     // load sensor definitions
  Output::load(stream);     // load output definitions
}

void EEStore::clear(){
  // create blank eeStore structure (no turnouts, no sensors) and save it back 
  // to EEPROM
  sprintf(eeStore->data.id,EESTORE_ID);                           
  eeStore->data.nTurnouts=0;
  eeStore->data.nSensors=0;
  eeStore->data.nOutputs=0;
  EEPROM.put(0,eeStore->data);
}

void EEStore::store(){
  reset();
  Turnout::store();
  Sensor::store();
  Output::store();
  EEPROM.put(0,eeStore->data);
}

void EEStore::advance(int n){
  eeAddress+=n;
}

void EEStore::reset(){
  eeAddress=sizeof(EEStore);
}

int EEStore::pointer(){
  return(eeAddress);
}

EEStore *EEStore::eeStore=NULL;
int EEStore::eeAddress=0;