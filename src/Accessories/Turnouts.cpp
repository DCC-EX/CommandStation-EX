/*
 *  Turnouts.cpp
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

#include "Turnouts.h"

#include "../CommInterface/CommManager.h"
#include "EEStore.h"

#if defined(ARDUINO_ARCH_SAMD) || defined(ARDUINO_ARCH_SAMC)
#include <SparkFun_External_EEPROM.h>
#else
#include <EEPROM.h>
#endif

void Turnout::activate(Print* stream, int s, DCC* track){
  // if s>0 set turnout=ON, else if zero or negative set turnout=OFF
  data.tStatus=(s>0);   
  genericResponse response;
  track->setAccessory(data.address, data.subAddress, data.tStatus, response);
  if(num>0)
    EEPROM.put(num,data.tStatus);
  CommManager::send(stream, F("<H %d %d>"), data.id, data.tStatus);
  turnoutlistHash++;
}

Turnout* Turnout::get(int n){
  Turnout *tt;
  for(tt=firstTurnout;tt!=NULL && tt->data.id!=n;tt=tt->nextTurnout);
  return(tt);
}

void Turnout::remove(Print* stream, int n){
  Turnout *tt,*pp;
  tt=firstTurnout;
  pp=tt;

  for( ;tt!=NULL && tt->data.id!=n;pp=tt,tt=tt->nextTurnout);

  if(tt==NULL){
    CommManager::send(stream, F("<X>"));
    return;
  }

  if(tt==firstTurnout)
    firstTurnout=tt->nextTurnout;
  else
    pp->nextTurnout=tt->nextTurnout;

  free(tt);

  CommManager::send(stream, F("<O>"));
  turnoutlistHash++;
}

void Turnout::show(Print* stream, int n){
  Turnout *tt;

  if(firstTurnout==NULL){
    CommManager::send(stream, F("<X>"));
    return;
  }

  for(tt=firstTurnout;tt!=NULL;tt=tt->nextTurnout){
    if(n==1) {
    CommManager::send(stream, F("<H %d %d %d %d>"), tt->data.id, tt->data.address, 
      tt->data.subAddress, tt->data.tStatus);
    } 
    else {
    CommManager::send(stream, F("<H %d %d>"), tt->data.id, tt->data.tStatus);
    }
  }
}

void Turnout::load(Print* stream){
  struct TurnoutData data;
  Turnout *tt;

  for(int i=0;i<EEStore::eeStore->data.nTurnouts;i++){
    EEPROM.get(EEStore::pointer(),data);
    tt=create(stream, data.id, data.address, data.subAddress);
    tt->data.tStatus=data.tStatus;
    tt->num=EEStore::pointer();
    EEStore::advance(sizeof(tt->data));
  }
}

void Turnout::store(){
  Turnout *tt;

  tt=firstTurnout;
  EEStore::eeStore->data.nTurnouts=0;

  while(tt!=NULL){
    tt->num=EEStore::pointer();
    EEPROM.put(EEStore::pointer(),tt->data);
    EEStore::advance(sizeof(tt->data));
    tt=tt->nextTurnout;
    EEStore::eeStore->data.nTurnouts++;
  }

}

Turnout *Turnout::create(Print* stream, int id, int add, int subAdd, int v){
  Turnout *tt;

  if(firstTurnout==NULL){
    firstTurnout=(Turnout *)calloc(1,sizeof(Turnout));
    tt=firstTurnout;
  } else if((tt=get(id))==NULL){
    tt=firstTurnout;
    while(tt->nextTurnout!=NULL)
    tt=tt->nextTurnout;
    tt->nextTurnout=(Turnout *)calloc(1,sizeof(Turnout));
    tt=tt->nextTurnout;
  }

  if(tt==NULL){       // problem allocating memory
    if(v==1)
    CommManager::send(stream, F("<X>"));
    return(tt);
  }

  tt->data.id=id;
  tt->data.address=add;
  tt->data.subAddress=subAdd;
  tt->data.tStatus=0;
  if(v==1)
    CommManager::send(stream, F("<O>"));
  turnoutlistHash++;
  return(tt);
}

Turnout *Turnout::firstTurnout=NULL;
int Turnout::turnoutlistHash=0; //bump on every change so clients know when to refresh their lists