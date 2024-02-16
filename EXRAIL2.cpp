/*
 *  © 2021 Neil McKechnie
 *  © 2021-2023 Harald Barth
 *  © 2020-2023 Chris Harlow
 *  © 2022-2023 Colin Murdoch
 *  All rights reserved.
 *  
 *  This file is part of CommandStation-EX
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

/* EXRAILPlus planned FEATURE additions
   F1. [DONE] DCC accessory packet opcodes (short and long form)
   F2. [DONE] ONAccessory catchers 
   F3. [DONE] Turnout descriptions for Withrottle
   F4. [DONE] Oled announcements (depends on HAL)
   F5. [DONE] Withrottle roster info
   F6. Multi-occupancy semaphore
   F7. [DONE see AUTOSTART] Self starting sequences
   F8. Park/unpark
   F9.  [DONE] Analog drive
   F10. [DONE] Alias anywhere
   F11. [DONE]EXRAIL/ENDEXRAIL unnecessary
   F12. [DONE] Allow guarded code (as effect of ALIAS anywhere)
   F13. [DONE] IFGTE/IFLT function  
  */
/* EXRAILPlus planned TRANSPARENT additions
   T1. [DONE] RAM based fast lookup for sequences ON* event catchers and signals.
   T2. Extend to >64k
  */

#include <Arduino.h>
#include "defines.h"
#include "EXRAIL2.h"
#include "DCC.h"
#include "DCCWaveform.h"
#include "DIAG.h"
#include "WiThrottle.h"
#include "DCCEXParser.h"
#include "Turnouts.h"
#include "CommandDistributor.h"
#include "TrackManager.h"
#include "Turntables.h"
#include "IODevice.h"


// One instance of RMFT clas is used for each "thread" in the automation.
// Each thread manages a loco on a journey through the layout, and/or may manage a scenery automation.
// The threads exist in a ring, each time through loop() the next thread in the ring is serviced.

// Statics 
const int16_t LOCO_ID_WAITING=-99; // waiting for loco id from prog track
int16_t RMFT2::progtrackLocoId;  // used for callback when detecting a loco on prog track
bool RMFT2::diag=false;      // <D EXRAIL ON>  
RMFT2 * RMFT2::loopTask=NULL; // loopTask contains the address of ONE of the tasks in a ring.
RMFT2 * RMFT2::pausingTask=NULL; // Task causing a PAUSE.
 // when pausingTask is set, that is the ONLY task that gets any service,
 // and all others will have their locos stopped, then resumed after the pausing task resumes.
byte RMFT2::flags[MAX_FLAGS];
Print * RMFT2::LCCSerial=0;
LookList *  RMFT2::routeLookup=NULL;
LookList *  RMFT2::onThrowLookup=NULL;
LookList *  RMFT2::onCloseLookup=NULL;
LookList *  RMFT2::onActivateLookup=NULL;
LookList *  RMFT2::onDeactivateLookup=NULL;
LookList *  RMFT2::onRedLookup=NULL;
LookList *  RMFT2::onAmberLookup=NULL;
LookList *  RMFT2::onGreenLookup=NULL;
LookList *  RMFT2::onChangeLookup=NULL;
LookList *  RMFT2::onClockLookup=NULL;
#ifndef IO_NO_HAL
LookList *  RMFT2::onRotateLookup=NULL;
#endif
LookList *  RMFT2::onOverloadLookup=NULL;
byte * RMFT2::routeStateArray=nullptr; 
const FSH  * * RMFT2::routeCaptionArray=nullptr; 
int16_t * RMFT2::stashArray=nullptr;
int16_t RMFT2::maxStashId=0;

// getOperand instance version, uses progCounter from instance.
uint16_t RMFT2::getOperand(byte n) {
  return getOperand(progCounter,n);
}

// getOperand static version, must be provided prog counter from loop etc.
uint16_t RMFT2::getOperand(int progCounter,byte n) {
  int offset=progCounter+1+(n*3);
  byte lsb=GETHIGHFLASH(RouteCode,offset);
  byte msb=GETHIGHFLASH(RouteCode,offset+1);
  return msb<<8|lsb;
}

LookList::LookList(int16_t size) {
  m_size=size;
  m_loaded=0;
  m_chain=nullptr;
  if (size) {
    m_lookupArray=new int16_t[size];
    m_resultArray=new int16_t[size];
  }
}

void LookList::add(int16_t lookup, int16_t result) {
  if (m_loaded==m_size) return; // and forget
  m_lookupArray[m_loaded]=lookup;
  m_resultArray[m_loaded]=result;
  m_loaded++;
}

int16_t LookList::find(int16_t value) {
  for (int16_t i=0;i<m_size;i++) {
    if (m_lookupArray[i]==value) return m_resultArray[i];
  }
  return m_chain ?  m_chain->find(value)  :-1;
}
void LookList::chain(LookList * chain) {
  m_chain=chain;
}
void LookList::handleEvent(const FSH* reason,int16_t id) {
  // New feature... create multiple ONhandlers
  for (int i=0;i<m_size;i++) 
    if (m_lookupArray[i]==id)
       RMFT2::startNonRecursiveTask(reason,id,m_resultArray[i]);
}


void LookList::stream(Print * _stream) {
  for (int16_t i=0;i<m_size;i++) {
    _stream->print(" ");
    _stream->print(m_lookupArray[i]);
  }
}

int16_t LookList::findPosition(int16_t value) {
  for (int16_t i=0;i<m_size;i++) {
    if (m_lookupArray[i]==value) return i;
  }
  return -1;
}
int16_t LookList::size() {
   return m_size;
}

LookList* RMFT2::LookListLoader(OPCODE op1, OPCODE op2, OPCODE op3) {
  int progCounter;
  int16_t count=0;
  // find size for list
  for (progCounter=0;; SKIPOP) {
    byte opcode=GET_OPCODE;
    if (opcode==OPCODE_ENDEXRAIL) break;
    if (opcode==op1 || opcode==op2 || opcode==op3) count++;
  }
  // create list 
  LookList* list=new LookList(count);
  if (count==0) return list;

  for (progCounter=0;; SKIPOP) {
    byte opcode=GET_OPCODE;
    if (opcode==OPCODE_ENDEXRAIL) break;
    if (opcode==op1 || opcode==op2 || opcode==op3)  list->add(getOperand(progCounter,0),progCounter);   
  }
  return list;
}

/* static */ void RMFT2::begin() {

  DIAG(F("EXRAIL RoutCode at =%P"),RouteCode);
    
  bool saved_diag=diag;
  diag=true;
  DCCEXParser::setRMFTFilter(RMFT2::ComandFilter);
  for (int f=0;f<MAX_FLAGS;f++) flags[f]=0;
  
  // create lookups
  routeLookup=LookListLoader(OPCODE_ROUTE, OPCODE_AUTOMATION);
  routeLookup->chain(LookListLoader(OPCODE_SEQUENCE));
  if (compileFeatures && FEATURE_ROUTESTATE) {
    routeStateArray=(byte *)calloc(routeLookup->size(),sizeof(byte));
    routeCaptionArray=(const FSH * *)calloc(routeLookup->size(),sizeof(const FSH *));
  }
  onThrowLookup=LookListLoader(OPCODE_ONTHROW);
  onCloseLookup=LookListLoader(OPCODE_ONCLOSE);
  onActivateLookup=LookListLoader(OPCODE_ONACTIVATE);
  onDeactivateLookup=LookListLoader(OPCODE_ONDEACTIVATE);
  onChangeLookup=LookListLoader(OPCODE_ONCHANGE);
  onClockLookup=LookListLoader(OPCODE_ONTIME);
#ifndef IO_NO_HAL
  onRotateLookup=LookListLoader(OPCODE_ONROTATE);
#endif
  onOverloadLookup=LookListLoader(OPCODE_ONOVERLOAD);
  // onLCCLookup is not the same so not loaded here. 

  // Second pass startup, define any turnouts or servos, set signals red
  // add sequences onRoutines to the lookups
if (compileFeatures & FEATURE_SIGNAL) {
  onRedLookup=LookListLoader(OPCODE_ONRED);
  onAmberLookup=LookListLoader(OPCODE_ONAMBER);
  onGreenLookup=LookListLoader(OPCODE_ONGREEN);
  for (int sigslot=0;;sigslot++) {
    VPIN sigid=GETHIGHFLASHW(RMFT2::SignalDefinitions,sigslot*8);
    if (sigid==0) break;  // end of signal list
    doSignal(sigid & SIGNAL_ID_MASK, SIGNAL_RED);
  }
  }

  int progCounter;
  for (progCounter=0;; SKIPOP){
    byte opcode=GET_OPCODE;
    if (opcode==OPCODE_ENDEXRAIL) break;
    VPIN operand=getOperand(progCounter,0);
    
    switch (opcode) {
    case OPCODE_AT:
    case OPCODE_ATTIMEOUT2:
    case OPCODE_AFTER:
    case OPCODE_AFTEROVERLOAD:
    case OPCODE_IF:
    case OPCODE_IFNOT: {
      int16_t pin = (int16_t)operand;
      if (pin<0) pin = -pin;
      DIAG(F("EXRAIL input VPIN %u"),pin);
      IODevice::configureInput((VPIN)pin,true);
      break;
    }
    case OPCODE_STASH:
    case OPCODE_CLEAR_STASH:
    case OPCODE_PICKUP_STASH: {
      maxStashId=max(maxStashId,((int16_t)operand));
      break;
    }

    case OPCODE_ATGTE:
    case OPCODE_ATLT:
    case OPCODE_IFGTE:
    case OPCODE_IFLT:
    case OPCODE_DRIVE: {
      DIAG(F("EXRAIL analog input VPIN %u"),(VPIN)operand);
      IODevice::configureAnalogIn((VPIN)operand);
      break;
    }

    case OPCODE_TURNOUT: {
      VPIN id=operand;
      int addr=getOperand(progCounter,1);
      byte subAddr=getOperand(progCounter,2);
      setTurnoutHiddenState(DCCTurnout::create(id,addr,subAddr));
      break;
    }

    case OPCODE_SERVOTURNOUT: {
      VPIN id=operand;
      VPIN pin=getOperand(progCounter,1);
      int activeAngle=getOperand(progCounter,2);
      int inactiveAngle=getOperand(progCounter,3);
      int profile=getOperand(progCounter,4);
      setTurnoutHiddenState(ServoTurnout::create(id,pin,activeAngle,inactiveAngle,profile));
      break;
    }

    case OPCODE_PINTURNOUT: {
      VPIN id=operand;
      VPIN pin=getOperand(progCounter,1);
      setTurnoutHiddenState(VpinTurnout::create(id,pin));
      break;
    }

#ifndef IO_NO_HAL
    case OPCODE_DCCTURNTABLE: {
      VPIN id=operand;
      int home=getOperand(progCounter,1);
      setTurntableHiddenState(DCCTurntable::create(id));
      Turntable *tto=Turntable::get(id);
      tto->addPosition(0,0,home);
      break;
    }

    case OPCODE_EXTTTURNTABLE: {
      VPIN id=operand;
      VPIN pin=getOperand(progCounter,1);
      int home=getOperand(progCounter,3);
      setTurntableHiddenState(EXTTTurntable::create(id,pin));
      Turntable *tto=Turntable::get(id);
      tto->addPosition(0,0,home);
      break;
    }

    case OPCODE_TTADDPOSITION: {
      VPIN id=operand;
      int position=getOperand(progCounter,1);
      int value=getOperand(progCounter,2);
      int angle=getOperand(progCounter,3);
      Turntable *tto=Turntable::get(id);
      tto->addPosition(position,value,angle);
      break;
    }
#endif

    case OPCODE_AUTOSTART:
      // automatically create a task from here at startup.
      // Removed if (progCounter>0) check 4.2.31 because 
      // default start it top of file is now removed. .   
      new RMFT2(progCounter);
      break;
      
    default: // Ignore
      break;
    }
  }
  SKIPOP; // include ENDROUTES opcode
  
  if (compileFeatures & FEATURE_STASH) {
    // create the stash array from the highest id found
    if (maxStashId>0) stashArray=(int16_t*)calloc(maxStashId+1, sizeof(int16_t));
     //TODO check EEPROM and fetch stashArray
  }
  
  DIAG(F("EXRAIL %db, fl=%d, stash=%d"),progCounter,MAX_FLAGS, maxStashId);

  // Removed for 4.2.31  new RMFT2(0); // add the startup route
  diag=saved_diag;
}

void RMFT2::setTurnoutHiddenState(Turnout * t) {
  // turnout descriptions are in low flash F strings
  const FSH *desc = getTurnoutDescription(t->getId());
  if (desc) t->setHidden(GETFLASH(desc)==0x01);
}

#ifndef IO_NO_HAL
void RMFT2::setTurntableHiddenState(Turntable * tto) {
  const FSH *desc = getTurntableDescription(tto->getId());
  if (desc) tto->setHidden(GETFLASH(desc)==0x01);
}
#endif

char RMFT2::getRouteType(int16_t id) {
  int16_t progCounter=routeLookup->find(id);
  if (progCounter>=0) {
    byte type=GET_OPCODE; 
    if (type==OPCODE_ROUTE) return 'R';
    if (type==OPCODE_AUTOMATION) return 'A';
  }
  return 'X';
}


RMFT2::RMFT2(int progCtr) {
  progCounter=progCtr;

  // get an unused  task id from the flags table
  taskId=255; // in case of overflow
  for (int f=0;f<MAX_FLAGS;f++) {
    if (!getFlag(f,TASK_FLAG)) {
      taskId=f;
      setFlag(f, TASK_FLAG);
      break;
    }
  }
  delayTime=0;
  loco=0;
  speedo=0;
  forward=true;
  invert=false;
  timeoutFlag=false;
  stackDepth=0;
  onEventStartPosition=-1; // Not handling an ONxxx 

  // chain into ring of RMFTs
  if (loopTask==NULL) {
    loopTask=this;
    next=this;
  } else {
    next=loopTask->next;
    loopTask->next=this;
  }
}


RMFT2::~RMFT2() {
  driveLoco(1); // ESTOP my loco if any
  setFlag(taskId,0,TASK_FLAG); // we are no longer using this id
  if (next==this)
    loopTask=NULL;
  else
    for (RMFT2* ring=next;;ring=ring->next)
      if (ring->next == this) {
	ring->next=next;
	loopTask=next;
	break;
      }
}

void RMFT2::createNewTask(int route, uint16_t cab) {
      int pc=routeLookup->find(route);
      if (pc<0) return;
      RMFT2* task=new RMFT2(pc);
      task->loco=cab;
}

void RMFT2::driveLoco(byte speed) {
  if (loco<=0) return;  // Prevent broadcast!
  if (diag) DIAG(F("EXRAIL drive %d %d %d"),loco,speed,forward^invert);
 /* TODO.....
 power on appropriate track if DC or main if dcc
  if (TrackManager::getMainPowerMode()==POWERMODE::OFF) {
    TrackManager::setMainPower(POWERMODE::ON);
  }
  **********/

  DCC::setThrottle(loco,speed, forward^invert);
  speedo=speed;
}

bool RMFT2::readSensor(uint16_t sensorId) {
  // Exrail operands are unsigned but we need the signed version as inserted by the macros.
  int16_t sId=(int16_t) sensorId;

  VPIN vpin=abs(sId);
  if (getFlag(vpin,LATCH_FLAG)) return true; // latched on
  
  // negative sensorIds invert the logic (e.g. for a break-beam sensor which goes OFF when detecting)
  bool s= IODevice::read(vpin) ^ (sId<0);
  if (s && diag) DIAG(F("EXRAIL Sensor %d hit"),sId);
  return s;
}

// This skips to the end of an if block, or to the ELSE within it.
bool RMFT2::skipIfBlock() {
  // returns false if killed
  short nest = 1;
  while (nest > 0) {
    SKIPOP;
    byte opcode =  GET_OPCODE;
    // all other IF type commands increase the nesting level
    if (opcode>IF_TYPE_OPCODES) nest++;
    else switch(opcode) {
      case OPCODE_ENDEXRAIL:
        kill(F("missing ENDIF"), nest);
        return false;
    
      case OPCODE_ENDIF:
        nest--;
        break;
    
      case OPCODE_ELSE:
        // if nest==1 then this is the ELSE for the IF we are skipping
        if (nest==1) nest=0; // cause loop exit and return after ELSE
        break;
    default:
      break;
    }
  }
  return true;
}



/* static */ void RMFT2::readLocoCallback(int16_t cv) {
  if (cv & LONG_ADDR_MARKER) {               // maker bit indicates long addr
    progtrackLocoId = cv ^ LONG_ADDR_MARKER; // remove marker bit to get real long addr
    if (progtrackLocoId <= HIGHEST_SHORT_ADDR ) {     // out of range for long addr
      DIAG(F("Long addr %d <= %d unsupported\n"), progtrackLocoId, HIGHEST_SHORT_ADDR);
      progtrackLocoId = -1;
    }
  } else {
    progtrackLocoId=cv;
  }
}

void RMFT2::loop() {

  // Round Robin call to a RMFT task each time
  if (loopTask==NULL) return;
  loopTask=loopTask->next;
  if (pausingTask==NULL || pausingTask==loopTask) loopTask->loop2();
}


void RMFT2::loop2() {
  if (delayTime!=0 && millis()-delayStart < delayTime) return;

  byte opcode = GET_OPCODE;
  int16_t operand =  getOperand(0);

  // skipIf will get set to indicate a failing IF condition 
  bool skipIf=false; 

  // if (diag) DIAG(F("RMFT2 %d %d"),opcode,operand);
  // Attention: Returning from this switch leaves the program counter unchanged.
  //            This is used for unfinished waits for timers or sensors.
  //            Breaking from this switch will step to the next step in the route.
  switch ((OPCODE)opcode) {

  case OPCODE_THROW:
    Turnout::setClosed(operand, false);
    break;
    
  case OPCODE_CLOSE:
    Turnout::setClosed(operand, true);
    break;

#ifndef IO_NO_HAL
  case OPCODE_ROTATE:
    uint8_t activity;
    activity=getOperand(2);
    Turntable::setPosition(operand,getOperand(1),activity);
    break;
#endif

  case OPCODE_REV:
    forward = false;
    driveLoco(operand);
    break;
    
  case OPCODE_FWD:
      forward = true;
      driveLoco(operand);
      break;
      
  case OPCODE_SPEED:
    forward=DCC::getThrottleDirection(loco)^invert;
    driveLoco(operand);
    break;
    
  case OPCODE_FORGET:
    if (loco!=0) {
      DCC::forgetLoco(loco);
      loco=0; 
    } 
    break;

  case OPCODE_INVERT_DIRECTION:
    invert= !invert;
    driveLoco(speedo);
    break;
    
  case OPCODE_RESERVE:
    if (getFlag(operand,SECTION_FLAG)) {
      driveLoco(0);
      delayMe(500);
      return;
    }
    setFlag(operand,SECTION_FLAG);
    break;
    
  case OPCODE_FREE:
    setFlag(operand,0,SECTION_FLAG);
    break;
    
  case OPCODE_AT:
    timeoutFlag=false;
    if (readSensor(operand)) break;
    delayMe(50);
    return;
    
  case OPCODE_ATGTE: // wait for analog sensor>= value
    timeoutFlag=false;
    if (IODevice::readAnalogue(operand) >= (int)(getOperand(1))) break;
    delayMe(50);
    return;
    
  case OPCODE_ATLT: // wait for analog sensor < value
    timeoutFlag=false;
    if (IODevice::readAnalogue(operand) < (int)(getOperand(1))) break;
    delayMe(50);
    return;
      
  case OPCODE_ATTIMEOUT1:   // ATTIMEOUT(vpin,timeout) part 1
    timeoutStart=millis();
    timeoutFlag=false;
    break;
    
  case OPCODE_ATTIMEOUT2:
    if (readSensor(operand)) break; // success without timeout
    if (millis()-timeoutStart > 100*getOperand(1)) {
      timeoutFlag=true;
      break; // and drop through
    }
    delayMe(50);
    return;
    
  case OPCODE_IFTIMEOUT: // do next operand if timeout flag set
    skipIf=!timeoutFlag;
    break;
    
  case OPCODE_AFTER: // waits for sensor to hit and then remain off for 0.5 seconds. (must come after an AT operation)
    if (readSensor(operand)) {
      // reset timer to half a second and keep waiting
      waitAfter=millis();
      delayMe(50);
      return;
    }
    if (millis()-waitAfter < 500 ) return;
    break;

  case OPCODE_AFTEROVERLOAD: // waits for the power to be turned back on - either by power routine or button
    if (!TrackManager::isPowerOn(operand)) {
      // reset timer to half a second and keep waiting
      waitAfter=millis();
      delayMe(50);
      return;
    }
    if (millis()-waitAfter < 500 ) return;
    break;

  case OPCODE_LATCH:
    setFlag(operand,LATCH_FLAG);
    break;
    
  case OPCODE_UNLATCH:
    setFlag(operand,0,LATCH_FLAG);
    break;

  case OPCODE_SET:
    IODevice::write(operand,true);
    break;
    
  case OPCODE_RESET:
    IODevice::write(operand,false);
    break;
    
  case OPCODE_PAUSE:
    DCC::setThrottle(0,1,true);  // pause all locos on the track
    pausingTask=this;
    break;

  case OPCODE_POM:
    if (loco) DCC::writeCVByteMain(loco, operand, getOperand(1));
    break;

  case OPCODE_POWEROFF:
    TrackManager::setPower(POWERMODE::OFF);
    TrackManager::setJoin(false);
    break;
  
  case OPCODE_SET_POWER:
      // operand is TRACK_POWER , trackid
        //byte thistrack=getOperand(1);
        switch (operand) {
          case TRACK_POWER_0:
            TrackManager::setTrackPower(POWERMODE::OFF, getOperand(1));
          break;
          case TRACK_POWER_1:
            TrackManager::setTrackPower(POWERMODE::ON, getOperand(1));
          break;
        }

    break;

  case OPCODE_SET_TRACK:
      // operand is trackmode<<8 | track id
      // If DC/DCX use  my loco for DC address 
      {
        TRACK_MODE mode = (TRACK_MODE)(operand>>8);
        int16_t cab=(mode & TRACK_MODE_DC) ? loco : 0;
        TrackManager::setTrackMode(operand & 0x0F, mode, cab);
      }
      break; 

  case OPCODE_RESUME:
    pausingTask=NULL;
    driveLoco(speedo);
    for (RMFT2 * t=next; t!=this;t=t->next) if (t->loco >0) t->driveLoco(t->speedo);
    break;
    
  case OPCODE_IF: // do next operand if sensor set
    skipIf=!readSensor(operand);
    break;
    
  case OPCODE_ELSE: // skip to matching ENDIF
    skipIf=true;
    break;
    
  case OPCODE_IFGTE: // do next operand if sensor>= value
    skipIf=IODevice::readAnalogue(operand)<(int)(getOperand(1));
    break;
    
  case OPCODE_IFLT: // do next operand if sensor< value
    skipIf=IODevice::readAnalogue(operand)>=(int)(getOperand(1));
    break;
    
  case OPCODE_IFLOCO: // do if the loco is the active one
    skipIf=loco!=(uint16_t)operand; // bad luck if someone enters negative loco numbers into EXRAIL
    break;

  case OPCODE_IFNOT: // do next operand if sensor not set
    skipIf=readSensor(operand);
    break;

  case OPCODE_IFRE: // do next operand if rotary encoder != position
    skipIf=IODevice::readAnalogue(operand)!=(int)(getOperand(1));
    break;
    
  case OPCODE_IFRANDOM: // do block on random percentage
    skipIf=(uint8_t)micros() >= operand * 255/100;
    break;
    
  case OPCODE_IFRESERVE: // do block if we successfully RERSERVE
    if (!getFlag(operand,SECTION_FLAG)) setFlag(operand,SECTION_FLAG);
    else skipIf=true;
    break;
    
  case OPCODE_IFRED: // do block if signal as expected
    skipIf=!isSignal(operand,SIGNAL_RED);
    break;
    
  case OPCODE_IFAMBER: // do block if signal as expected
    skipIf=!isSignal(operand,SIGNAL_AMBER);
    break;
    
  case OPCODE_IFGREEN: // do block if signal as expected
    skipIf=!isSignal(operand,SIGNAL_GREEN);
    break;
    
  case OPCODE_IFTHROWN:
    skipIf=Turnout::isClosed(operand);
    break;
    
  case OPCODE_IFCLOSED:
    skipIf=Turnout::isThrown(operand);
    break;

#ifndef IO_NO_HAL
  case OPCODE_IFTTPOSITION: // do block if turntable at this position
    skipIf=Turntable::getPosition(operand)!=(int)getOperand(1);
    break;
#endif

  case OPCODE_ENDIF:
    break;
    
  case OPCODE_DELAYMS:
    delayMe(operand);
    break;
    
  case OPCODE_DELAY:
    delayMe(operand*100L);
    break;
    
  case OPCODE_DELAYMINS:
    delayMe(operand*60L*1000L);
    break;
    
  case OPCODE_RANDWAIT:
    delayMe(operand==0 ? 0 : (micros()%operand) *100L);
    break;
    
  case OPCODE_RED:
    doSignal(operand,SIGNAL_RED);
    break;
    
  case OPCODE_AMBER:
    doSignal(operand,SIGNAL_AMBER);
    break;
    
  case OPCODE_GREEN:
    doSignal(operand,SIGNAL_GREEN);
    break;
    
  case OPCODE_FON:
    if (loco) DCC::setFn(loco,operand,true);
    break;

  case OPCODE_FOFF:
    if (loco) DCC::setFn(loco,operand,false);
    break;
    
  case OPCODE_DRIVE:
    {
      byte analogSpeed=IODevice::readAnalogue(operand) *127 / 1024;
      if (speedo!=analogSpeed) driveLoco(analogSpeed);
      break;
    }
    
  case OPCODE_XFON:
    DCC::setFn(operand,getOperand(1),true);
    break;
    
  case OPCODE_XFOFF:
    DCC::setFn(operand,getOperand(1),false);
    break;
    
  case OPCODE_DCCACTIVATE: {
    // operand is address<<3 | subaddr<<1 | active
    int16_t addr=operand>>3;
    int16_t subaddr=(operand>>1) & 0x03;
    bool active=operand & 0x01;
    DCC::setAccessory(addr,subaddr,active);
    break;
  }
   case OPCODE_ASPECT: {
    // operand is address<<5 |  value
    int16_t address=operand>>5;
    byte aspect=operand & 0x1f;
    if (!signalAspectEvent(address,aspect))
      DCC::setExtendedAccessory(address,aspect);
    break;
  }
    
  case OPCODE_FOLLOW:
    progCounter=routeLookup->find(operand);
    if (progCounter<0) kill(F("FOLLOW unknown"), operand);
    return;
    
  case OPCODE_CALL:
    if (stackDepth==MAX_STACK_DEPTH) {
      kill(F("CALL stack"), stackDepth);
      return;
    }
    callStack[stackDepth++]=progCounter+3;
    progCounter=routeLookup->find(operand);
    if (progCounter<0) kill(F("CALL unknown"),operand);
    return;
    
  case OPCODE_RETURN:
    if (stackDepth==0) {
      kill(F("RETURN stack"));
      return;
    }
    progCounter=callStack[--stackDepth];
    return;
    
  case OPCODE_ENDTASK:
  case OPCODE_ENDEXRAIL:
    kill();
    return;

  case OPCODE_KILLALL:
    while(loopTask) loopTask->kill(F("KILLALL"));
    return;

#ifndef DISABLE_PROG
  case OPCODE_JOIN:
    TrackManager::setPower(POWERMODE::ON);
    TrackManager::setJoin(true);
    break;

  case OPCODE_UNJOIN:
    TrackManager::setJoin(false);
    break;

  case OPCODE_READ_LOCO1: // READ_LOCO is implemented as 2 separate opcodes
    progtrackLocoId=LOCO_ID_WAITING;  // Nothing found yet
    DCC::getLocoId(readLocoCallback);
    break;
    
  case OPCODE_READ_LOCO2:
    if (progtrackLocoId==LOCO_ID_WAITING) {
      delayMe(100);
      return; // still waiting for callback
    }
    if (progtrackLocoId<0) {
      kill(F("No Loco Found"),progtrackLocoId);
      return; // still waiting for callback
    }
    
    loco=progtrackLocoId;
    speedo=0;
    forward=true;
    invert=false;
    break;
#endif

  case OPCODE_POWERON:
    TrackManager::setMainPower(POWERMODE::ON);
    TrackManager::setJoin(false);
    break;
    
  case OPCODE_START:
    {
      int newPc=routeLookup->find(operand);
      if (newPc<0) break;
      new RMFT2(newPc);
    }
    break;
    
  case OPCODE_SENDLOCO:  // cab, route
    {
      int newPc=routeLookup->find(getOperand(1));
      if (newPc<0) break;
      RMFT2* newtask=new RMFT2(newPc); // create new task
      newtask->loco=operand;
    }
    break;
    
  case OPCODE_SETLOCO:
    {
      loco=operand;
      speedo=0;
      forward=true;
      invert=false;
    }
    break;

  case OPCODE_LCC:  // short form LCC
      if ((compileFeatures & FEATURE_LCC) && LCCSerial) 
          StringFormatter::send(LCCSerial,F("<L x%h>"),(uint16_t)operand);
       break; 

  case OPCODE_LCCX: // long form LCC
       if ((compileFeatures & FEATURE_LCC) && LCCSerial)
            StringFormatter::send(LCCSerial,F("<L x%h%h%h%h>\n"),
                 getOperand(progCounter,1),
                 getOperand(progCounter,2),
                 getOperand(progCounter,3),
                 getOperand(progCounter,0)
                 );    
        break;  
    
  case OPCODE_SERVO: // OPCODE_SERVO,V(vpin),OPCODE_PAD,V(position),OPCODE_PAD,V(profile),OPCODE_PAD,V(duration)
    IODevice::writeAnalogue(operand,getOperand(1),getOperand(2),getOperand(3));
    break;
    
  case OPCODE_WAITFOR: // OPCODE_SERVO,V(pin)
    if (IODevice::isBusy(operand)) {
      delayMe(100);
      return;
    }
    break;

#ifndef IO_NO_HAL
  case OPCODE_WAITFORTT:  // OPCODE_WAITFOR,V(turntable_id)
    if (Turntable::ttMoving(operand)) {
      delayMe(100);
      return;
    }
    break;
#endif

  case OPCODE_PRINT:
    printMessage(operand);
    break;
  case OPCODE_ROUTE_HIDDEN:
    manageRouteState(operand,2);
    break;   
  case OPCODE_ROUTE_INACTIVE:
    manageRouteState(operand,0);
    break;   
  case OPCODE_ROUTE_ACTIVE:
    manageRouteState(operand,1);
    break;   
  case OPCODE_ROUTE_DISABLED:
    manageRouteState(operand,4);
    break;   

  case OPCODE_STASH:
    if (compileFeatures & FEATURE_STASH) 
      stashArray[operand] = invert? -loco : loco;
    break; 

  case OPCODE_CLEAR_STASH:
    if (compileFeatures & FEATURE_STASH) 
      stashArray[operand] = 0;
    break; 
       
  case OPCODE_CLEAR_ALL_STASH:
    if (compileFeatures & FEATURE_STASH) 
      for (int i=0;i<=maxStashId;i++) stashArray[operand]=0;
    break;

  case OPCODE_PICKUP_STASH:
    if (compileFeatures & FEATURE_STASH) {
      int16_t x=stashArray[operand];
      if (x>=0) {
        loco=x;
        invert=false;
        break;
      }
      loco=-x;
      invert=true;
    }
    break;
  
  case OPCODE_ROUTE:
  case OPCODE_AUTOMATION:
  case OPCODE_SEQUENCE:
    if (diag) DIAG(F("EXRAIL begin(%d)"),operand);
    break;
    
  case OPCODE_AUTOSTART: // Handled only during begin process
  case OPCODE_PAD: // Just a padding for previous opcode needing >1 operand byte.
  case OPCODE_TURNOUT: // Turnout definition ignored at runtime
  case OPCODE_SERVOTURNOUT: // Turnout definition ignored at runtime
  case OPCODE_PINTURNOUT: // Turnout definition ignored at runtime
  case OPCODE_ONCLOSE: // Turnout event catchers ignored here
  case OPCODE_ONLCC:   // LCC event catchers ignored here 
  case OPCODE_ONTHROW:
  case OPCODE_ONACTIVATE: // Activate event catchers ignored here
  case OPCODE_ONDEACTIVATE:
  case OPCODE_ONRED:
  case OPCODE_ONAMBER:
  case OPCODE_ONGREEN:
  case OPCODE_ONCHANGE:
  case OPCODE_ONTIME:
#ifndef IO_NO_HAL
  case OPCODE_DCCTURNTABLE: // Turntable definition ignored at runtime
  case OPCODE_EXTTTURNTABLE:  // Turntable definition ignored at runtime
  case OPCODE_TTADDPOSITION:  // Turntable position definition ignored at runtime
  case OPCODE_ONROTATE:
#endif
  case OPCODE_ONOVERLOAD:
  
    break;
    
  default:
    kill(F("INVOP"),operand);
  }
  // Falling out of the switch means move on to the next opcode
  // but if we are skipping a false IF or else
  if (skipIf)  if (!skipIfBlock()) return;
  SKIPOP;
}

void RMFT2::delayMe(long delay) {
  delayTime=delay;
  delayStart=millis();
}

bool RMFT2::setFlag(VPIN id,byte onMask, byte offMask) {
   if (FLAGOVERFLOW(id)) return false; // Outside range limit
   byte f=flags[id];
   f &= ~offMask;
   f |= onMask;
   flags[id]=f;
   return true;
}

bool RMFT2::getFlag(VPIN id,byte mask) {
  if (FLAGOVERFLOW(id)) return 0; // Outside range limit
  return flags[id]&mask;
}

void RMFT2::kill(const FSH * reason, int operand) {
  if (reason) DIAG(F("EXRAIL ERROR pc=%d, cab=%d, %S %d"), progCounter,loco, reason, operand);
  else if (diag) DIAG(F("ENDTASK at pc=%d"), progCounter);
  delete this;
}

int16_t RMFT2::getSignalSlot(int16_t id) {
  for (int sigslot=0;;sigslot++) {
      int16_t sigid=GETHIGHFLASHW(RMFT2::SignalDefinitions,sigslot*8);
      if (sigid==0) { // end of signal list 
        DIAG(F("EXRAIL Signal %d not defined"), id);
        return -1;
      }
      // sigid is the signal id used in RED/AMBER/GREEN macro
      // for a LED signal it will be same as redpin
      // but for a servo signal it will also have SERVO_SIGNAL_FLAG set. 

      if ((sigid & SIGNAL_ID_MASK)!= id) continue; // keep looking
      return sigslot; // relative slot in signals table
  }  
}

/* static */ void RMFT2::doSignal(int16_t id,char rag) {
  if (!(compileFeatures & FEATURE_SIGNAL)) return; // dont compile code below
  if (diag) DIAG(F(" doSignal %d %x"),id,rag);
  
  // Schedule any event handler for this signal change.
  // This will work even without a signal definition. 
  if (rag==SIGNAL_RED) onRedLookup->handleEvent(F("RED"),id);
  else if (rag==SIGNAL_GREEN) onGreenLookup->handleEvent(F("GREEN"),id);
  else onAmberLookup->handleEvent(F("AMBER"),id);
  
  int16_t sigslot=getSignalSlot(id);
  if (sigslot<0) return; 
  
  // keep track of signal state 
  setFlag(sigslot,rag,SIGNAL_MASK);
 
  // Correct signal definition found, get the rag values
  int16_t sigpos=sigslot*8; 
  VPIN sigid=GETHIGHFLASHW(RMFT2::SignalDefinitions,sigpos);
  VPIN redpin=GETHIGHFLASHW(RMFT2::SignalDefinitions,sigpos+2);
  VPIN amberpin=GETHIGHFLASHW(RMFT2::SignalDefinitions,sigpos+4);
  VPIN greenpin=GETHIGHFLASHW(RMFT2::SignalDefinitions,sigpos+6);
  if (diag) DIAG(F("signal %d %d %d %d %d"),sigid,id,redpin,amberpin,greenpin);

  VPIN sigtype=sigid & ~SIGNAL_ID_MASK;

  if (sigtype == SERVO_SIGNAL_FLAG) {
    // A servo signal, the pin numbers are actually servo positions
    // Note, setting a signal to a zero position has no effect.
    int16_t servopos= rag==SIGNAL_RED? redpin: (rag==SIGNAL_GREEN? greenpin : amberpin);
    if (diag) DIAG(F("sigA %d %d"),id,servopos);
    if  (servopos!=0) IODevice::writeAnalogue(id,servopos,PCA9685::Bounce);
    return;  
  }

  
  if (sigtype== DCC_SIGNAL_FLAG) {
    // redpin,amberpin are the DCC addr,subaddr 
    DCC::setAccessory(redpin,amberpin, rag!=SIGNAL_RED);
    return; 
  }

 if (sigtype== DCCX_SIGNAL_FLAG) {
    // redpin,amberpin,greenpin are the 3 aspects
    byte value=redpin;
    if (rag==SIGNAL_AMBER) value=amberpin;
    if (rag==SIGNAL_GREEN) value=greenpin; 
    DCC::setExtendedAccessory(sigid & SIGNAL_ID_MASK,value);
    return; 
  }


  // LED or similar 3 pin signal, (all pins zero would be a virtual signal)
  // If amberpin is zero, synthesise amber from red+green
  const byte SIMAMBER=0x00;
  if (rag==SIGNAL_AMBER && (amberpin==0)) rag=SIMAMBER; // special case this func only
   
  // Manage invert (HIGH on) pins
  bool aHigh=sigid & ACTIVE_HIGH_SIGNAL_FLAG;
      
  // set the three pins 
  if (redpin) {
    bool redval=(rag==SIGNAL_RED || rag==SIMAMBER);
    if (!aHigh) redval=!redval;
    IODevice::write(redpin,redval);
  }
  if (amberpin) {
    bool amberval=(rag==SIGNAL_AMBER);
    if (!aHigh) amberval=!amberval;
    IODevice::write(amberpin,amberval);
  }
  if (greenpin) {
    bool greenval=(rag==SIGNAL_GREEN || rag==SIMAMBER);
    if (!aHigh) greenval=!greenval;
    IODevice::write(greenpin,greenval);
  }
}

/* static */ bool RMFT2::isSignal(int16_t id,char rag) {
  if (!(compileFeatures & FEATURE_SIGNAL)) return false; 
  int16_t sigslot=getSignalSlot(id);
  if (sigslot<0) return false; 
  return (flags[sigslot] & SIGNAL_MASK) == rag;
}


// signalAspectEvent returns true if the aspect is destined
// for a defined DCCX_SIGNAL which will handle all the RAG flags
// and ON* handlers.
// Otherwise false so the parser should send the command directly 
bool RMFT2::signalAspectEvent(int16_t address, byte aspect ) {
  if (!(compileFeatures & FEATURE_SIGNAL)) return false; 
  int16_t sigslot=getSignalSlot(address);
  if (sigslot<0) return false;  // this is not a defined signal 
  int16_t sigpos=sigslot*8; 
  VPIN sigid=GETHIGHFLASHW(RMFT2::SignalDefinitions,sigpos);
  VPIN sigtype=sigid & ~SIGNAL_ID_MASK;
  if (sigtype!=DCCX_SIGNAL_FLAG) return false; // not a DCCX signal
  // Turn an aspect change into a RED/AMBER/GREEN setting
  if (aspect==GETHIGHFLASHW(RMFT2::SignalDefinitions,sigpos+2)) {
      doSignal(sigid,SIGNAL_RED);
      return true;
  }
  
  if (aspect==GETHIGHFLASHW(RMFT2::SignalDefinitions,sigpos+4)) {
      doSignal(sigid,SIGNAL_AMBER);
      return true;
  }
  
  if (aspect==GETHIGHFLASHW(RMFT2::SignalDefinitions,sigpos+6)) {
      doSignal(sigid,SIGNAL_GREEN);
      return true;
  }

  return false;  // aspect is not a defined one    
}

void RMFT2::turnoutEvent(int16_t turnoutId, bool closed) {
  // Hunt for an ONTHROW/ONCLOSE for this turnout
  if (closed)  onCloseLookup->handleEvent(F("CLOSE"),turnoutId);
  else onThrowLookup->handleEvent(F("THROW"),turnoutId);
}


void RMFT2::activateEvent(int16_t addr, bool activate) {
  // Hunt for an ONACTIVATE/ONDEACTIVATE for this accessory
  if (activate)  onActivateLookup->handleEvent(F("ACTIVATE"),addr);
  else onDeactivateLookup->handleEvent(F("DEACTIVATE"),addr);
}

void RMFT2::changeEvent(int16_t vpin, bool change) {
  // Hunt for an ONCHANGE for this sensor
  if (change)  onChangeLookup->handleEvent(F("CHANGE"),vpin);
}

#ifndef IO_NO_HAL
void RMFT2::rotateEvent(int16_t turntableId, bool change) {
  // Hunt or an ONROTATE for this turntable
  if (change) onRotateLookup->handleEvent(F("ROTATE"),turntableId);
}
#endif

void RMFT2::clockEvent(int16_t clocktime, bool change) {
  // Hunt for an ONTIME for this time
  if (Diag::CMD)
   DIAG(F("Looking for clock event at : %d"), clocktime);
  if (change) {
    onClockLookup->handleEvent(F("CLOCK"),clocktime);
    onClockLookup->handleEvent(F("CLOCK"),25*60+clocktime%60);
  }
} 

void RMFT2::powerEvent(int16_t track, bool overload) {
  // Hunt for an ONOVERLOAD for this item
  if (Diag::CMD)
   DIAG(F("Looking for Power event on track : %c"), track);
  if (overload) {
    onOverloadLookup->handleEvent(F("POWER"),track);
  }
}

void RMFT2::startNonRecursiveTask(const FSH* reason, int16_t id,int pc) {  
  // Check we dont already have a task running this handler
  RMFT2 * task=loopTask;
  while(task) {
    if (task->onEventStartPosition==pc) {
      DIAG(F("Recursive ON%S(%d)"),reason, id);
      return;
    }
    task=task->next;
    if (task==loopTask) break;
  }
  
  task=new RMFT2(pc);  // new task starts at this instruction
  task->onEventStartPosition=pc; // flag for recursion detector
}

void RMFT2::printMessage2(const FSH * msg) {
  DIAG(F("EXRAIL(%d) %S"),loco,msg);
}
static StringBuffer * buffer=NULL;
/* thrungeString is used to stream a HIGHFLASH string to a suitable Serial
and handle the oddities like LCD, BROADCAST and PARSE */    
void RMFT2::thrungeString(uint32_t strfar, thrunger mode, byte id) {
   //DIAG(F("thrunge addr=%l mode=%d id=%d"), strfar,mode,id);
   Print * stream=NULL;
   // Find out where the string is going 
   switch (mode) {
    case thrunge_print:
         StringFormatter::send(&USB_SERIAL,F("<* EXRAIL(%d) "),loco);
         stream=&USB_SERIAL;
         break;

    case thrunge_serial: stream=&USB_SERIAL; break;  
    case thrunge_serial1: 
         #ifdef SERIAL1_COMMANDS
         stream=&Serial1; 
         #endif
         break;
    case thrunge_serial2: 
         #ifdef SERIAL2_COMMANDS
         stream=&Serial2; 
         #endif
         break;
    case thrunge_serial3: 
         #ifdef SERIAL3_COMMANDS
         stream=&Serial3; 
         #endif
         break;
    case thrunge_serial4: 
         #ifdef SERIAL4_COMMANDS
         stream=&Serial4; 
         #endif
         break;
    case thrunge_serial5: 
         #ifdef SERIAL5_COMMANDS
         stream=&Serial5; 
         #endif
         break;
   case thrunge_serial6: 
         #ifdef SERIAL6_COMMANDS
         stream=&Serial6; 
         #endif
         break;
    case thrunge_lcn: 
      #if defined(LCN_SERIAL) 
      stream=&LCN_SERIAL;
      #endif
       break;  
    case thrunge_parse:
    case thrunge_broadcast:
    case thrunge_lcd:
    default:    // thrunge_lcd+1, ...
         if (!buffer) buffer=new StringBuffer();
         buffer->flush();
         stream=buffer;
         break; 
    }
    if (!stream) return; 
    
     #if defined(ARDUINO_AVR_MEGA) || defined(ARDUINO_AVR_MEGA2560)
    // if mega stream it out 
    for (;;strfar++) {
      char c=pgm_read_byte_far(strfar);
      if (c=='\0') break;
      stream->write(c);
    }
    #else
    // UNO/NANO CPUs dont have high memory
    // 32 bit cpus dont care anyway
    stream->print((FSH *)strfar);
    #endif

  // and decide what to do next
   switch (mode) {
    case thrunge_print:
         StringFormatter::send(&USB_SERIAL,F(" *>\n"));
         break;
    // TODO  more serials for SAMx case thrunge_serial4: stream=&Serial4; break;
    case thrunge_parse: 
      DCCEXParser::parseOne(&USB_SERIAL,(byte*)buffer->getString(),NULL);
      break;
    case thrunge_broadcast:
      CommandDistributor::broadcastRaw(CommandDistributor::COMMAND_TYPE,buffer->getString());
      break;
    case thrunge_withrottle:
      CommandDistributor::broadcastRaw(CommandDistributor::WITHROTTLE_TYPE,buffer->getString());
      break;
    case thrunge_lcd:
         LCD(id,F("%s"),buffer->getString());
         break;
    default:  // thrunge_lcd+1, ...
      if (mode > thrunge_lcd) 
        SCREEN(mode-thrunge_lcd, id, F("%s"),buffer->getString());  // print to other display
      break;       
    }
}

void RMFT2::manageRouteState(uint16_t id, byte state) {
  if (compileFeatures && FEATURE_ROUTESTATE) {
    // Route state must be maintained for when new throttles connect.
    // locate route id in the Routes lookup
    int16_t position=routeLookup->findPosition(id);
    if (position<0) return; 
    // set state beside it 
    if (routeStateArray[position]==state) return; 
    routeStateArray[position]=state;
    CommandDistributor::broadcastRouteState(id,state);
  }
}
void RMFT2::manageRouteCaption(uint16_t id,const FSH* caption) {
  if (compileFeatures && FEATURE_ROUTESTATE) {
    // Route state must be maintained for when new throttles connect.
    // locate route id in the Routes lookup
    int16_t position=routeLookup->findPosition(id);
    if (position<0) return; 
    // set state beside it 
    if (routeCaptionArray[position]==caption) return; 
    routeCaptionArray[position]=caption;
    CommandDistributor::broadcastRouteCaption(id,caption);
  }
}
  
