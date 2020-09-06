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

bool Turnout::activate(int n,bool state,DCC* track){
  Turnout * tt=get(n);
  if (tt==NULL) return false;
  tt->activate(state, track);
  if(n>0) EEPROM.put(n,tt->data.tStatus);
  return true;
}
// activate is virtual here so that it can be overridden by a non-DCC turnout mechanism
 void Turnout::activate(bool state, DCC* track) {
  data.tStatus=(state>0); 
  genericResponse response;
  track->setAccessory(data.address, data.subAddress, data.tStatus, response);
  turnoutlistHash++;
}

Turnout* Turnout::get(int n){
  Turnout *tt;
  for(tt=firstTurnout;tt!=NULL && tt->data.id!=n;tt=tt->nextTurnout);
  return(tt);
}

//returns false if error removing turnot
bool Turnout::remove(int n){
  Turnout *tt,*pp;
  tt=firstTurnout;
  pp=tt;

  for( ;tt!=NULL && tt->data.id!=n;pp=tt,tt=tt->nextTurnout);

  if(tt==NULL) return false;

  if(tt==firstTurnout)
    firstTurnout=tt->nextTurnout;
  else
    pp->nextTurnout=tt->nextTurnout;

  free(tt);

  turnoutlistHash++;
  return true;
}

//read turnout list from EEPROM
void Turnout::load(){
  struct TurnoutData data;
  Turnout *tt;

  for(int i=0;i<EEStore::eeStore->data.nTurnouts;i++){
    EEPROM.get(EEStore::pointer(),data);
    tt=create(data.id, data.address, data.subAddress);
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

/* returns new Turnout or null if problem creating */
Turnout *Turnout::create(int id, int add, int subAdd, int v){
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
    return(tt);
  }
  tt->data.id=id;
  tt->data.address=add;
  tt->data.subAddress=subAdd;
  tt->data.tStatus=0;
  turnoutlistHash++;
  return(tt);
}

Turnout *Turnout::firstTurnout=NULL;
int Turnout::turnoutlistHash=0; //bump on every change so clients know when to refresh their lists