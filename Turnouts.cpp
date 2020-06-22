#include "Turnouts.h"
#include "EEStore.h"
#include "StringFormatter.h"

 bool Turnout::activate(int n,bool state){
  Turnout * tt=get(n);
  if (tt==NULL) return false;
  tt->activate(state);
  if(n>0) EEPROM.put(n,tt->data.tStatus);
  return true;
}

// activate is virtual here so that it can be overridden by a non-DCC turnout mechanism
void Turnout::activate(bool state) {
  data.tStatus=state;                            
  DCC::setAccessory(data.address,data.subAddress, data.tStatus);
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
  return true;
 
}

///////////////////////////////////////////////////////////////////////////////

void Turnout::show(Print & stream, int n){
  for(Turnout *tt=firstTurnout;tt!=NULL;tt=tt->nextTurnout){
      if (tt->data.id==n) {
        StringFormatter::send(stream,F("<H %d %d>"), tt->data.id, tt->data.tStatus);
        return;
      }
  }
}

bool Turnout::showAll(Print & stream){
  bool gotOne=false;
  for(Turnout * tt=firstTurnout;tt!=NULL;tt=tt->nextTurnout){
      StringFormatter::send(stream,F("<H %d %d %d %d>"), tt->data.id, tt->data.address, tt->data.subAddress, tt->data.tStatus);
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
    tt=create(data.id,data.address,data.subAddress);
    tt->data.tStatus=data.tStatus;
    tt->num=EEStore::pointer();
    EEStore::advance(sizeof(tt->data));
  }
}

///////////////////////////////////////////////////////////////////////////////

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
///////////////////////////////////////////////////////////////////////////////

Turnout *Turnout::create(int id, int add, int subAdd){
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

  if (tt==NULL) return(tt);
  
  tt->data.id=id;
  tt->data.address=add;
  tt->data.subAddress=subAdd;
  tt->data.tStatus=0;
  return(tt);

}

///////////////////////////////////////////////////////////////////////////////

Turnout *Turnout::firstTurnout=NULL;
