#include "EEStore.h"
#include "Turnouts.h"
#include "Sensors.h"
#include "Outputs.h"


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

EEStore *EEStore::eeStore=NULL;
int EEStore::eeAddress=0;
