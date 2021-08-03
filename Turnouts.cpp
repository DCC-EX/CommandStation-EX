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
#define EESTOREDEBUG 
#include "defines.h"
#include "Turnouts.h"
#include "EEStore.h"
#include "StringFormatter.h"
#include "RMFT2.h"
#ifdef EESTOREDEBUG
#include "DIAG.h"
#endif

// Keywords used for turnout configuration.
const int16_t HASH_KEYWORD_SERVO=27709;
const int16_t HASH_KEYWORD_DCC=6436;
const int16_t HASH_KEYWORD_VPIN=-415;

enum unit8_t {
  TURNOUT_DCC = 1,
  TURNOUT_SERVO = 2,
  TURNOUT_VPIN = 3,
  TURNOUT_LCN = 4,
};

///////////////////////////////////////////////////////////////////////////////
// Static function to print all Turnout states to stream in form "<H id state>"

void Turnout::printAll(Print *stream){
  for (Turnout *tt = Turnout::firstTurnout; tt != NULL; tt = tt->nextTurnout)
    StringFormatter::send(stream, F("<H %d %d>\n"), tt->data.id, tt->data.active);
} // Turnout::printAll

///////////////////////////////////////////////////////////////////////////////
// Object method to print configuration of one Turnout to stream, in one of the following forms:
//  <H id SERVO vpin activePos inactivePos profile state>
//  <H id LCN state>
//  <H id VPIN vpin state>
//  <H id DCC address subAddress state>

void Turnout::print(Print *stream){
  uint8_t state = ((data.active) != 0);
  uint8_t type = data.type;
  switch (type) {
    case TURNOUT_LCN:
      // LCN Turnout
      StringFormatter::send(stream, F("<H %d LCN %d>\n"), data.id, state);
      break;
    case TURNOUT_DCC:
      // DCC Turnout
      StringFormatter::send(stream, F("<H %d DCC %d %d %d>\n"), data.id, 
        (((data.dccAccessoryData.address-1) >> 2)+1), ((data.dccAccessoryData.address-1) & 3), state);
      break;
    case TURNOUT_VPIN:
      // VPIN Digital output
      StringFormatter::send(stream, F("<H %d VPIN %d %d>\n"), data.id, data.vpinData.vpin, state);
      break;
#ifndef IO_NO_HAL
    case TURNOUT_SERVO:
      // Servo Turnout
      StringFormatter::send(stream, F("<H %d SERVO %d %d %d %d %d>\n"), data.id, data.servoData.vpin, 
        data.servoData.activePosition, data.servoData.inactivePosition, data.servoData.profile, state);
      break;
#endif
    default:
      break;
  }
}

///////////////////////////////////////////////////////////////////////////////
// Static function to activate/deactivate Turnout with ID 'n'.
//   Returns false if turnout not found.

bool Turnout::activate(int n, bool state){
#ifdef EESTOREDEBUG
  DIAG(F("Turnout::activate(%d,%d)"),n,state);
#endif
  Turnout * tt=get(n);
  if (!tt) return false;
  tt->activate(state);
  turnoutlistHash++;
  return true;
}

///////////////////////////////////////////////////////////////////////////////
// Static function to check if the Turnout with ID 'n' is activated or not.
// Returns false if turnout not found.

bool Turnout::isActive(int n){
  Turnout * tt=get(n);
  if (!tt) return false;
  return tt->isActive();
}


///////////////////////////////////////////////////////////////////////////////
// Object function to check the status of Turnout is activated or not.

bool Turnout::isActive() {
  return data.active;
}

///////////////////////////////////////////////////////////////////////////////
// Object method to activate or deactivate the Turnout.  

// activate is virtual here so that it can be overridden by a non-DCC turnout mechanism
void Turnout::activate(bool state) {
#ifdef EESTOREDEBUG
  DIAG(F("Turnout::activate(%d)"),state);
#endif
  if (data.type == TURNOUT_LCN) {
      // A LCN turnout is transmitted to the LCN master.
      LCN::send('T', data.id, state);
      return;   // The tStatus will be updated by a message from the LCN master, later.
  }
    data.active = state;
    switch (data.type) {
      case TURNOUT_DCC:
        DCC::setAccessory((((data.dccAccessoryData.address-1) >> 2) + 1), 
          ((data.dccAccessoryData.address-1) & 3), state);
        break;
#ifndef IO_NO_HAL
      case TURNOUT_SERVO:
        IODevice::write(data.servoData.vpin, state);
        break;
#endif
      case TURNOUT_VPIN:
        IODevice::write(data.vpinData.vpin, state);
        break;
    }
    // Save state if stored in EEPROM
    if (EEStore::eeStore->data.nTurnouts > 0 && num > 0) 
      EEPROM.put(num, data.tStatus);

#if defined(RMFT_ACTIVE)
  RMFT2::turnoutEvent(data.id, state);
#endif  

}

///////////////////////////////////////////////////////////////////////////////
// Static function to find Turnout object specified by ID 'n'.  Return NULL if not found.

Turnout* Turnout::get(int n){
  Turnout *tt;
  for(tt=firstTurnout;tt!=NULL && tt->data.id!=n;tt=tt->nextTurnout);
  return(tt);
}

///////////////////////////////////////////////////////////////////////////////
// Static function to delete Turnout object specified by ID 'n'.  Return false if not found.

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
// Static function to load all Turnout definitions from EEPROM
// TODO: Consider transmitting the initial state of the DCC/LCN turnout here.
//  (already done for servo turnouts and VPIN turnouts).

void Turnout::load(){
  struct TurnoutData data;
  Turnout *tt=NULL;

  for(uint16_t i=0;i<EEStore::eeStore->data.nTurnouts;i++){
    // Retrieve data
    EEPROM.get(EEStore::pointer(), data);
    
    int lastKnownState = data.active;
    switch (data.type) {
      case TURNOUT_DCC:
        tt=createDCC(data.id, ((data.dccAccessoryData.address-1)>>2)+1, (data.dccAccessoryData.address-1)&3); // DCC-based turnout
        break;
      case TURNOUT_LCN:
        // LCN turnouts are created when the remote device sends a message.
        break;
#ifndef IO_NO_HAL
      case TURNOUT_SERVO:
        tt=createServo(data.id, data.servoData.vpin, 
          data.servoData.activePosition, data.servoData.inactivePosition, data.servoData.profile, lastKnownState);
        break;
#endif
      case TURNOUT_VPIN:
        tt=createVpin(data.id, data.vpinData.vpin, lastKnownState);  // VPIN-based turnout
        break;
       
      default:
       tt=NULL;
    }
    if (tt) tt->num = EEStore::pointer() + offsetof(TurnoutData, tStatus);  // Save pointer to tStatus byte within EEPROM
    // Advance by the actual size of the individual turnout struct.
    EEStore::advance(data.size);
#ifdef EESTOREDEBUG
    if (tt) print(tt);
#endif
  }
}

///////////////////////////////////////////////////////////////////////////////
// Static function to store all Turnout definitions to EEPROM

void Turnout::store(){
  Turnout *tt;

  tt=firstTurnout;
  EEStore::eeStore->data.nTurnouts=0;

  while(tt!=NULL){
    // LCN turnouts aren't saved to EEPROM
    if (tt->data.type != TURNOUT_LCN) {  
#ifdef EESTOREDEBUG
      print(tt);
#endif
      tt->num = EEStore::pointer() + offsetof(TurnoutData, tStatus); // Save pointer to tstatus byte within EEPROM
      EEPROM.put(EEStore::pointer(),tt->data);
      EEStore::advance(tt->data.size);
      EEStore::eeStore->data.nTurnouts++;
    }
    tt=tt->nextTurnout;
  }
}

///////////////////////////////////////////////////////////////////////////////
// Static function for creating a DCC-controlled Turnout.

Turnout *Turnout::createDCC(int id, uint16_t add, uint8_t subAdd){
  if (add > 511 || subAdd > 3) return NULL;
  Turnout *tt=create(id);
  if (!tt) return(tt);
  tt->data.type = TURNOUT_DCC;
  tt->data.size = sizeof(tt->data.header) + sizeof(tt->data.dccAccessoryData);
  tt->data.active = 0;
  tt->data.dccAccessoryData.address = ((add-1) << 2) + subAdd + 1;
  return(tt);
}

///////////////////////////////////////////////////////////////////////////////
// Static function for creating a LCN-controlled Turnout.

Turnout *Turnout::createLCN(int id, uint8_t state) {
  Turnout *tt=create(id);
  if (!tt) return(tt);
  tt->data.type = TURNOUT_LCN;
  tt->data.size = sizeof(tt->data.header) + sizeof(tt->data.lcnData);
  tt->data.active = (state != 0);
  return(tt);
}

///////////////////////////////////////////////////////////////////////////////
// Static function for associating a Turnout id with a virtual pin in IODevice space.
// The actual creation and configuration of the pin must be done elsewhere,
// e.g. in mySetup.cpp during startup of the CS.

Turnout *Turnout::createVpin(int id, VPIN vpin, uint8_t state){
  if (vpin > VPIN_MAX) return NULL;
  Turnout *tt=create(id);
  if(!tt) return(tt);
  tt->data.type = TURNOUT_VPIN;;
  tt->data.size = sizeof(tt->data.header) + sizeof(tt->data.vpinData);
  tt->data.active = (state != 0);
  tt->data.vpinData.vpin = vpin;
  IODevice::write(vpin, state);   // Set initial state of output.
  return(tt);
}

#ifndef IO_NO_HAL
///////////////////////////////////////////////////////////////////////////////
// Method for creating a Servo Turnout, e.g. connected to PCA9685 PWM device.

Turnout *Turnout::createServo(int id, VPIN vpin, uint16_t activePosition, uint16_t inactivePosition, uint8_t profile, uint8_t state){
  if (activePosition > 511 || inactivePosition > 511 || profile > 4) return NULL;

  Turnout *tt=create(id);
  if (!tt) return(tt);
  if (tt->data.type != TURNOUT_SERVO) tt->data.active = (state != 0);  // Retain current state if it's an existing servo turnout.
  tt->data.type = TURNOUT_SERVO;
  tt->data.size = sizeof(tt->data.header) + sizeof(tt->data.servoData);
  tt->data.servoData.vpin = vpin;
  tt->data.servoData.activePosition = activePosition;
  tt->data.servoData.inactivePosition = inactivePosition;
  tt->data.servoData.profile = profile;
  // Configure PWM interface device
  int deviceParams[] = {(int)activePosition, (int)inactivePosition, profile, tt->data.active};
  if (!IODevice::configure(vpin, IODevice::CONFIGURE_SERVO, 4, deviceParams)) {
    remove(id);
    return NULL;
  }
  return(tt);
}
#endif

///////////////////////////////////////////////////////////////////////////////
// Support for <T id SERVO pin activepos inactive pos profile>
// and <T id DCC address subaddress>
// and <T id VPIN pin>

Turnout *Turnout::create(int id, int params, int16_t p[]) {
#ifndef IO_NO_HAL
  if (p[0] == HASH_KEYWORD_SERVO) { // <T id SERVO n n n n>
    if (params == 5)
      return createServo(id, (VPIN)p[1], (uint16_t)p[2], (uint16_t)p[3], (uint8_t)p[4]);
    else  
      return NULL;
  } else 
#endif
  if (p[0] == HASH_KEYWORD_VPIN) { // <T id VPIN n>
    if (params==2)
      return createVpin(id, p[1]);
    else
      return NULL;
  } else
  if (p[0]==HASH_KEYWORD_DCC) {
    if (params==3 && p[1]>0 && p[1]<=512 && p[2]>=0 && p[2]<4)  // <T id DCC n n>
      return createDCC(id, p[1], p[2]);
    else if (params==2 && p[1]>0 && p[1]<=512*4)  // <T id DCC nn>
      return createDCC(id, (p[1]-1)/4+1, (p[1]-1)%4);
    else
      return NULL;
  } else if (params==2) { // <T id n n> for DCC or LCN
    return createDCC(id, p[0], p[1]);
  } 
#ifndef IO_NO_HAL
  else if (params==3) { // legacy <T id n n n> for Servo
    return createServo(id, (VPIN)p[0], (uint16_t)p[1], (uint16_t)p[2]);
  }
#endif

  return NULL;
}

///////////////////////////////////////////////////////////////////////////////
// Create basic Turnout object.  The details of what sort of object it is 
//  controlling are not set here.

Turnout *Turnout::create(int id){
  Turnout *tt=get(id);
  if (tt==NULL) { 
     tt=(Turnout *)calloc(1,sizeof(Turnout));
     if (!tt) return (tt);
     tt->nextTurnout=firstTurnout;
     firstTurnout=tt;
     tt->data.id=id;
  }
  turnoutlistHash++;
  return tt;
}

///////////////////////////////////////////////////////////////////////////////
//
// Object method to print debug info about the state of a Turnout object
//
#ifdef EESTOREDEBUG
void Turnout::print(Turnout *tt) {
  tt->print(StringFormatter::diagSerial);
}
#endif

///////////////////////////////////////////////////////////////////////////////
Turnout *Turnout::firstTurnout=NULL;
int Turnout::turnoutlistHash=0; //bump on every change so clients know when to refresh their lists
