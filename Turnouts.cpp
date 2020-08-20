/*
 *  © 2020, Chris Harlow. All rights reserved.
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
#include "Turnouts.h"
#include "EEStore.h"
#include "StringFormatter.h"

#include "PWMServoDriver.h"
//#include "DIAG.h"             // uncomment if you need DIAG below

 bool Turnout::activate(int n,bool state){
  //DIAG(F("\nTurnout::activate(%d,%d)\n"),n,state);
  Turnout * tt=get(n);
  if (tt==NULL) return false;
  tt->activate(state);
  if(n>0) EEPROM.put(n,tt->data.tStatus);
  turnoutlistHash++;
  return true;
}

 bool Turnout::isActive(int n){
  Turnout * tt=get(n);
  if (tt==NULL) return false;
  return tt->data.tStatus & STATUS_ACTIVE;
}

// activate is virtual here so that it can be overridden by a non-DCC turnout mechanism
 void Turnout::activate(bool state) {
  //DIAG(F("\nTurnout::activate\n"));
  if (state) data.tStatus|=STATUS_ACTIVE;
  else data.tStatus &= ~STATUS_ACTIVE;                            
  if (data.tStatus & STATUS_PWM) PWMServoDriver::setServo(data.tStatus & STATUS_PWMPIN, (data.inactiveAngle+(state?data.moveAngle:0)));
     else DCC::setAccessory(data.address,data.subAddress, state);
}
///////////////////////////////////////////////////////////////////////////////

Turnout* Turnout::get(int n){
  Turnout *tt;
  for(tt=firstTurnout;tt!=NULL && tt->data.id!=n;tt=tt->nextTurnout);
  return(tt);
}
///////////////////////////////////////////////////////////////////////////////

bool Turnout::remove(int n){
  Turnout *tt,*pp=NULL;

  for(tt=firstTurnout;tt!=NULL && tt->data.id!=n;pp=tt,tt=tt->nextTurnout);

  if(tt==NULL) return false;
  
  if(tt==firstTurnout)
    firstTurnout=tt->nextTurnout;
  else
    pp->nextTurnout=tt->nextTurnout;

  free(tt);
  turnoutlistHash++;
  return true;
 
}

///////////////////////////////////////////////////////////////////////////////

void Turnout::show(Print * stream, int n){
  for(Turnout *tt=firstTurnout;tt!=NULL;tt=tt->nextTurnout){
      if (tt->data.id==n) {
        StringFormatter::send(stream,F("<H %d %d>"), tt->data.id, tt->data.tStatus & STATUS_ACTIVE);
        return;
      }
  }
}

bool Turnout::showAll(Print * stream){
  bool gotOne=false;
  for(Turnout * tt=firstTurnout;tt!=NULL;tt=tt->nextTurnout){
      StringFormatter::send(stream,F("<H %d %d %d %d>"), tt->data.id, tt->data.address, tt->data.subAddress, (tt->data.tStatus & STATUS_ACTIVE)!=0);
      gotOne=true;
  }
  return gotOne;
}


///////////////////////////////////////////////////////////////////////////////

void Turnout::load(){
  struct TurnoutData data;
  Turnout *tt;

  for(int i=0;i<EEStore::eeStore->data.nTurnouts;i++){
    EEPROM.get(EEStore::pointer(),data);
    if (data.tStatus & STATUS_PWM) tt=create(data.id,data.tStatus & STATUS_PWMPIN, data.inactiveAngle,data.moveAngle);
    else tt=create(data.id,data.address,data.subAddress);
    tt->data.tStatus=data.tStatus;
    EEStore::advance(sizeof(tt->data));
  }
}

///////////////////////////////////////////////////////////////////////////////

void Turnout::store(){
  Turnout *tt;

  tt=firstTurnout;
  EEStore::eeStore->data.nTurnouts=0;

  while(tt!=NULL){
    EEPROM.put(EEStore::pointer(),tt->data);
    EEStore::advance(sizeof(tt->data));
    tt=tt->nextTurnout;
    EEStore::eeStore->data.nTurnouts++;
  }

}
///////////////////////////////////////////////////////////////////////////////

Turnout *Turnout::create(int id, int add, int subAdd){
  Turnout *tt=create(id);
  tt->data.address=add;
  tt->data.subAddress=subAdd;
  tt->data.tStatus=0;
  return(tt);
}

Turnout *Turnout::create(int id, byte pin, int activeAngle, int inactiveAngle){
  Turnout *tt=create(id);
  tt->data.tStatus= STATUS_PWM | (pin &  STATUS_PWMPIN);
  tt->data.inactiveAngle=inactiveAngle;
  tt->data.moveAngle=activeAngle-inactiveAngle;
  return(tt);
}

Turnout *Turnout::create(int id){
  Turnout *tt=get(id);
  if (tt==NULL) { 
     tt=(Turnout *)calloc(1,sizeof(Turnout));
     tt->nextTurnout=firstTurnout;
     firstTurnout=tt;
     tt->data.id=id;
    }
  turnoutlistHash++;
  return tt;
  }
  ///////////////////////////////////////////////////////////////////////////////

Turnout *Turnout::firstTurnout=NULL;
int Turnout::turnoutlistHash=0; //bump on every change so clients know when to refresh their lists

