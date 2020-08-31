/*
 *  Outputs.cpp
 * 
 *  This file is part of CommandStation-EX.
 *
 *  CommandStation-EX is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  CommandStation-EX is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with CommandStation-EX.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "Outputs.h"

#include "../CommInterface/CommManager.h"
#include "EEStore.h"

#if defined(ARDUINO_ARCH_SAMD) || defined(ARDUINO_ARCH_SAMC)
#include <SparkFun_External_EEPROM.h>
#else
#include <EEPROM.h>
#endif

void Output::activate(Print* stream, int s){
  // if s>0, set status to active, else inactive
  data.oStatus=(s>0);             
  // set state of output pin to HIGH or LOW depending on whether bit zero of 
  // iFlag is set to 0 (ACTIVE=HIGH) or 1 (ACTIVE=LOW)                                  
  digitalWrite(data.pin,data.oStatus ^ bitRead(data.iFlag,0));      
  if(num>0)
    EEPROM.put(num,data.oStatus);
  CommManager::send(stream, F("<Y %d %d>"), data.id, data.oStatus);
}

Output* Output::get(int n){
  Output *tt;
  for(tt=firstOutput;tt!=NULL && tt->data.id!=n;tt=tt->nextOutput);
  return(tt);
}

void Output::remove(Print* stream, int n){
  Output *tt,*pp;
  tt=firstOutput;
  pp=tt;

  for( ; tt!=NULL && tt->data.id!=n; pp=tt,tt=tt->nextOutput);

  if(tt==NULL){
    CommManager::send(stream, F("<X>"));
    return;
  }

  if(tt==firstOutput)
    firstOutput=tt->nextOutput;
  else
    pp->nextOutput=tt->nextOutput;

  free(tt);

  CommManager::send(stream, F("<O>"));
}

void Output::show(Print* stream, int n){
  Output *tt;

  if(firstOutput==NULL){
    CommManager::send(stream, F("<X>"));
    return;
  }

  for(tt=firstOutput;tt!=NULL;tt=tt->nextOutput){
    if(n==1) {
    CommManager::send(stream, F("<Y %d %d %d %d>"), tt->data.id, tt->data.pin, tt->data.iFlag, tt->data.oStatus);
    } else {
    CommManager::send(stream, F("<Y %d %d>"), tt->data.id, tt->data.oStatus);
    }
  }
}

void Output::load(Print* stream){
  struct OutputData data;
  Output *tt;

  for(int i=0;i<EEStore::eeStore->data.nOutputs;i++){
    EEPROM.get(EEStore::pointer(),data);
    tt=create(stream, data.id, data.pin, data.iFlag);
    tt->data.oStatus=bitRead(tt->data.iFlag,1)?bitRead(tt->data.iFlag,2):data.oStatus;      // restore status to EEPROM value is bit 1 of iFlag=0, otherwise set to value of bit 2 of iFlag
    digitalWrite(tt->data.pin,tt->data.oStatus ^ bitRead(tt->data.iFlag,0));
    pinMode(tt->data.pin,OUTPUT);
    tt->num=EEStore::pointer();
    EEStore::advance(sizeof(tt->data));
  }
}

void Output::store(){
  Output *tt;

  tt=firstOutput;
  EEStore::eeStore->data.nOutputs=0;

  while(tt!=NULL){
    tt->num=EEStore::pointer();
    EEPROM.put(EEStore::pointer(),tt->data);
    EEStore::advance(sizeof(tt->data));
    tt=tt->nextOutput;
    EEStore::eeStore->data.nOutputs++;
  }

}

Output *Output::create(Print* stream, int id, int pin, int iFlag, int v){
  Output *tt;

  if(firstOutput==NULL){
    firstOutput=(Output *)calloc(1,sizeof(Output));
    tt=firstOutput;
  } else if((tt=get(id))==NULL){
    tt=firstOutput;
    while(tt->nextOutput!=NULL)
    tt=tt->nextOutput;
    tt->nextOutput=(Output *)calloc(1,sizeof(Output));
    tt=tt->nextOutput;
  }

  if(tt==NULL){       // problem allocating memory
    if(v==1)
    CommManager::send(stream, F("<X>"));
    return(tt);
  }

  tt->data.id=id;
  tt->data.pin=pin;
  tt->data.iFlag=iFlag;
  tt->data.oStatus=0;

  if(v==1){
    tt->data.oStatus=bitRead(tt->data.iFlag,1)?bitRead(tt->data.iFlag,2):0;      // sets status to 0 (INACTIVE) is bit 1 of iFlag=0, otherwise set to value of bit 2 of iFlag
    digitalWrite(tt->data.pin,tt->data.oStatus ^ bitRead(tt->data.iFlag,0));
    pinMode(tt->data.pin,OUTPUT);
    CommManager::send(stream, F("<O>"));
  }

  return(tt);

}

Output *Output::firstOutput=NULL;
