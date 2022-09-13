/*
 *  © 2021 Neil McKechnie
 *  © 2021-2022 Harald Barth
 *  © 2020-2022 Chris Harlow
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
   F4. Oled announcements (depends on HAL)
   F5. Withrottle roster info
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
#include "EXRAIL2.h"
#include "DCC.h"
#include "DCCWaveform.h"
#include "DIAG.h"
#include "WiThrottle.h"
#include "DCCEXParser.h"
#include "Turnouts.h"
#include "CommandDistributor.h"


// Command parsing keywords
const int16_t HASH_KEYWORD_EXRAIL=15435;    
const int16_t HASH_KEYWORD_ON = 2657;
const int16_t HASH_KEYWORD_START=23232;
const int16_t HASH_KEYWORD_RESERVE=11392;
const int16_t HASH_KEYWORD_FREE=-23052;
const int16_t HASH_KEYWORD_LATCH=1618;  
const int16_t HASH_KEYWORD_UNLATCH=1353;
const int16_t HASH_KEYWORD_PAUSE=-4142;
const int16_t HASH_KEYWORD_RESUME=27609;
const int16_t HASH_KEYWORD_KILL=5218;
const int16_t HASH_KEYWORD_ALL=3457;
const int16_t HASH_KEYWORD_ROUTES=-3702;
const int16_t HASH_KEYWORD_RED=26099;
const int16_t HASH_KEYWORD_AMBER=18713;
const int16_t HASH_KEYWORD_GREEN=-31493;

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

LookList *  RMFT2::sequenceLookup=NULL;
LookList *  RMFT2::onThrowLookup=NULL;
LookList *  RMFT2::onCloseLookup=NULL;
LookList *  RMFT2::onActivateLookup=NULL;
LookList *  RMFT2::onDeactivateLookup=NULL;

#define GET_OPCODE GETFLASH(RMFT2::RouteCode+progCounter)
#define GET_OPERAND(n) GETFLASHW(RMFT2::RouteCode+progCounter+1+(n*3))
#define SKIPOP progCounter+=3


LookList::LookList(int16_t size) {
  m_size=size;
  m_loaded=0;
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
  return -1;
}

/* static */ void RMFT2::begin() {
  DCCEXParser::setRMFTFilter(RMFT2::ComandFilter);
  for (int f=0;f<MAX_FLAGS;f++) flags[f]=0;
  int progCounter;

  // counters to create lookup arrays
  int sequenceCount=0; // to allow for seq 0 at start
  int onThrowCount=0;
  int onCloseCount=0;
  int onActivateCount=0;
  int onDeactivateCount=0;

  // first pass count sizes for fast lookup arrays
  for (progCounter=0;; SKIPOP) {
    byte opcode=GET_OPCODE;
    if (opcode==OPCODE_ENDEXRAIL) break;
    switch (opcode) {
    case OPCODE_ROUTE:
    case OPCODE_AUTOMATION:
    case OPCODE_SEQUENCE:
      sequenceCount++;
      break;

    case OPCODE_ONTHROW:
      onThrowCount++;
      break;
      
    case OPCODE_ONCLOSE:
      onCloseCount++;
      break;
      
    case OPCODE_ONACTIVATE:
      onActivateCount++;
      break;

    case OPCODE_ONDEACTIVATE:
      onDeactivateCount++;
      break;

    default: // Ignore
      break;
    }
  }

  // create lookups
  sequenceLookup=new LookList(sequenceCount);
  onThrowLookup=new LookList(onThrowCount);
  onCloseLookup=new LookList(onCloseCount);
  onActivateLookup=new LookList(onActivateCount);
  onDeactivateLookup=new LookList(onDeactivateCount);

  // Second pass startup, define any turnouts or servos, set signals red
  // add sequences onRoutines to the lookups
  for (int sigpos=0;;sigpos+=4) {
    VPIN sigid=GETFLASHW(RMFT2::SignalDefinitions+sigpos);
    if (sigid==0) break;  // end of signal list
    doSignal(sigid & SIGNAL_ID_MASK, SIGNAL_RED);
  }

  for (progCounter=0;; SKIPOP){
    byte opcode=GET_OPCODE;
    if (opcode==OPCODE_ENDEXRAIL) break;
    VPIN operand=GET_OPERAND(0);
    
    switch (opcode) {
    case OPCODE_AT:
    case OPCODE_AFTER:
    case OPCODE_IF:
    case OPCODE_IFNOT: {
      int16_t pin = (int16_t)operand;
      if (pin<0) pin = -pin;
      IODevice::configureInput((VPIN)pin,true);
      break;
    }

    case OPCODE_TURNOUT: {
      VPIN id=operand;
      int addr=GET_OPERAND(1);
      byte subAddr=GET_OPERAND(2);
      setTurnoutHiddenState(DCCTurnout::create(id,addr,subAddr));
      break;
    }

    case OPCODE_SERVOTURNOUT: {
      VPIN id=operand;
      VPIN pin=GET_OPERAND(1);
      int activeAngle=GET_OPERAND(2);
      int inactiveAngle=GET_OPERAND(3);
      int profile=GET_OPERAND(4);
      setTurnoutHiddenState(ServoTurnout::create(id,pin,activeAngle,inactiveAngle,profile));
      break;
    }

    case OPCODE_PINTURNOUT: {
      VPIN id=operand;
      VPIN pin=GET_OPERAND(1);
      setTurnoutHiddenState(VpinTurnout::create(id,pin));
      break;
    }
      
    case OPCODE_ROUTE:
    case OPCODE_AUTOMATION:
    case OPCODE_SEQUENCE:
      sequenceLookup->add(operand,progCounter);
      break;
      
    case OPCODE_ONTHROW:
      onThrowLookup->add(operand,progCounter);
      break;
      
    case OPCODE_ONCLOSE:
      onCloseLookup->add(operand,progCounter);
      break;
      
    case OPCODE_ONACTIVATE:
      onActivateLookup->add(operand,progCounter);
      break;
      
    case OPCODE_ONDEACTIVATE:
      onDeactivateLookup->add(operand,progCounter);
      break;
      
    case OPCODE_AUTOSTART:
      // automatically create a task from here at startup.
      new RMFT2(progCounter);
      break;
      
    default: // Ignore
      break;
    }
  }
  SKIPOP; // include ENDROUTES opcode

  DIAG(F("EXRAIL %db, fl=%d seq=%d, onT=%d, onC=%d"),
        progCounter,MAX_FLAGS,
        sequenceCount, onThrowCount, onCloseCount);

  new RMFT2(0); // add the startup route
}

void RMFT2::setTurnoutHiddenState(Turnout * t) {
  t->setHidden(GETFLASH(getTurnoutDescription(t->getId()))==0x01);     
}

char RMFT2::getRouteType(int16_t id) {
  for (int16_t i=0;;i++) {
    int16_t rid= GETFLASHW(routeIdList+i);
    if (rid==id) return 'R';
    if (rid==0) break;
  }
  for (int16_t i=0;;i++) {
    int16_t rid= GETFLASHW(automationIdList+i);
    if (rid==id) return 'A';
    if (rid==0) break;
  }
  return 'X';
}   
// This filter intercepts <> commands to do the following:
// - Implement RMFT specific commands/diagnostics
// - Reject/modify JMRI commands that would interfere with RMFT processing
void RMFT2::ComandFilter(Print * stream, byte & opcode, byte & paramCount, int16_t p[]) {
  (void)stream; // avoid compiler warning if we don't access this parameter
  bool reject=false;
  switch(opcode) {
    
  case 'D':
    if (p[0]==HASH_KEYWORD_EXRAIL) { // <D EXRAIL ON/OFF>
      diag = paramCount==2 && (p[1]==HASH_KEYWORD_ON || p[1]==1);
      opcode=0;
    }
        break;
	
  case '/':  // New EXRAIL command
    reject=!parseSlash(stream,paramCount,p);
    opcode=0;
    break;
    
  default:  // other commands pass through
    break;
  }
  if (reject) {
    opcode=0;
    StringFormatter::send(stream,F("<X>"));
  }
}

bool RMFT2::parseSlash(Print * stream, byte & paramCount, int16_t p[]) {

  if (paramCount==0) { // STATUS
    StringFormatter::send(stream, F("<* EXRAIL STATUS"));
    RMFT2 * task=loopTask;
    while(task) {
      StringFormatter::send(stream,F("\nID=%d,PC=%d,LOCO=%d%c,SPEED=%d%c"),
			    (int)(task->taskId),task->progCounter,task->loco,
			    task->invert?'I':' ',
			    task->speedo,
			    task->forward?'F':'R'
			    );
      task=task->next;
      if (task==loopTask) break;
    }
    // Now stream the flags
    for (int id=0;id<MAX_FLAGS; id++) {
      byte flag=flags[id];
      if (flag & ~TASK_FLAG & ~SIGNAL_MASK) { // not interested in TASK_FLAG only. Already shown above
	      StringFormatter::send(stream,F("\nflags[%d] "),id);
	      if (flag & SECTION_FLAG) StringFormatter::send(stream,F(" RESERVED"));
	      if (flag & LATCH_FLAG) StringFormatter::send(stream,F(" LATCHED"));
      }
    }
    // do the signals
    // flags[n] represents the state of the nth signal in the table 
    for (int sigslot=0;;sigslot++) {
      VPIN sigid=GETFLASHW(RMFT2::SignalDefinitions+sigslot*4);
      if (sigid==0) break; // end of signal list 
      byte flag=flags[sigslot] & SIGNAL_MASK; // obtain signal flags for this id
      StringFormatter::send(stream,F("\n%S[%d]"), 
        (flag == SIGNAL_RED)? F("RED") : (flag==SIGNAL_GREEN) ? F("GREEN") : F("AMBER"),
        sigid & SIGNAL_ID_MASK); 
    } 
    
    StringFormatter::send(stream,F(" *>\n"));
    return true;
  }
  switch (p[0]) {
  case HASH_KEYWORD_PAUSE: // </ PAUSE>
    if (paramCount!=1) return false;
    DCC::setThrottle(0,1,true);  // pause all locos on the track
    pausingTask=(RMFT2 *)1; // Impossible task address
    return true;
    
  case HASH_KEYWORD_RESUME: // </ RESUME>
    if (paramCount!=1) return false;
    pausingTask=NULL;
    {
      RMFT2 * task=loopTask;
      while(task) {
	if (task->loco) task->driveLoco(task->speedo);
	task=task->next;
	if (task==loopTask) break;
      }
    }
    return true;
    
    
  case HASH_KEYWORD_START: // </ START [cab] route >
    if (paramCount<2 || paramCount>3) return false;
    {
      int route=(paramCount==2) ? p[1] : p[2];
      uint16_t cab=(paramCount==2)? 0 : p[1];
      int pc=sequenceLookup->find(route);
      if (pc<0) return false;
      RMFT2* task=new RMFT2(pc);
      task->loco=cab;
    }
    return true;
    
  default:
    break;
  }

  // check KILL ALL here, otherwise the next validation confuses ALL with a flag  
  if (p[0]==HASH_KEYWORD_KILL && p[1]==HASH_KEYWORD_ALL) {
    while (loopTask) loopTask->kill(F("KILL ALL")); // destructor changes loopTask
    return true;   
  }

  // all other / commands take 1 parameter 0 to MAX_FLAGS-1
  if (paramCount!=2 || p[1]<0  || p[1]>=MAX_FLAGS) return false;
  
  switch (p[0]) {
  case HASH_KEYWORD_KILL: // Kill taskid|ALL
    {
      RMFT2 * task=loopTask;
      while(task) {
	      if (task->taskId==p[1]) {
	        task->kill(F("KILL"));
	        return  true;
	      }
	      task=task->next;
	      if (task==loopTask) break;
      }
    }
    return false;
    
  case HASH_KEYWORD_RESERVE:  // force reserve a section
    setFlag(p[1],SECTION_FLAG);
    return true;
    
  case HASH_KEYWORD_FREE:  // force free a section
    setFlag(p[1],0,SECTION_FLAG);
    return true;
    
  case HASH_KEYWORD_LATCH:
    setFlag(p[1], LATCH_FLAG);
    return true;
    
  case HASH_KEYWORD_UNLATCH:
    setFlag(p[1], 0, LATCH_FLAG);
    return true;
 
  case HASH_KEYWORD_RED:
    doSignal(p[1],SIGNAL_RED);
    return true;
 
  case HASH_KEYWORD_AMBER:
    doSignal(p[1],SIGNAL_AMBER);
    return true;
 
  case HASH_KEYWORD_GREEN:
    doSignal(p[1],SIGNAL_GREEN);
    return true;
    
  default:
    return false;
  }
}


// This emits Routes and Automations to Withrottle
// Automations are given a state to set the button to "handoff" which implies
// handing over the loco to the automation.
// Routes are given "Set" buttons and do not cause the loco to be handed over.



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
  onTurnoutId=-1; // Not handling an ONTHROW/ONCLOSE

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
      int pc=sequenceLookup->find(route);
      if (pc<0) return;
      RMFT2* task=new RMFT2(pc);
      task->loco=cab;
}

void RMFT2::driveLoco(byte speed) {
  if (loco<=0) return;  // Prevent broadcast!
  if (diag) DIAG(F("EXRAIL drive %d %d %d"),loco,speed,forward^invert);
  if (DCCWaveform::mainTrack.getPowerMode()==POWERMODE::OFF) {
    DCCWaveform::mainTrack.setPowerMode(POWERMODE::ON);
    CommandDistributor::broadcastPower();
  }
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
  int16_t operand =  GET_OPERAND(0);

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

  case OPCODE_REV:
    forward = false;
    driveLoco(operand);
    break;
    
  case OPCODE_FWD:
      forward = true;
      driveLoco(operand);
      break;
      
  case OPCODE_SPEED:
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
    if (IODevice::readAnalogue(operand) >= (int)(GET_OPERAND(1))) break;
    delayMe(50);
    return;
    
  case OPCODE_ATLT: // wait for analog sensor < value
    timeoutFlag=false;
    if (IODevice::readAnalogue(operand) < (int)(GET_OPERAND(1))) break;
    delayMe(50);
    return;
      
  case OPCODE_ATTIMEOUT1:   // ATTIMEOUT(vpin,timeout) part 1
    timeoutStart=millis();
    timeoutFlag=false;
    break;
    
  case OPCODE_ATTIMEOUT2:
    if (readSensor(operand)) break; // success without timeout
    if (millis()-timeoutStart > 100*GET_OPERAND(1)) {
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
    if (loco) DCC::writeCVByteMain(loco, operand, GET_OPERAND(1));
    break;
    
  case OPCODE_POWEROFF:
    DCCWaveform::mainTrack.setPowerMode(POWERMODE::OFF);
    DCCWaveform::progTrack.setPowerMode(POWERMODE::OFF);
    DCC::setProgTrackSyncMain(false);
    CommandDistributor::broadcastPower();
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
    skipIf=IODevice::readAnalogue(operand)<(int)(GET_OPERAND(1));
    break;
    
  case OPCODE_IFLT: // do next operand if sensor< value
    skipIf=IODevice::readAnalogue(operand)>=(int)(GET_OPERAND(1));
    break;
    
  case OPCODE_IFNOT: // do next operand if sensor not set
    skipIf=readSensor(operand);
    break;
    
  case OPCODE_IFRANDOM: // do block on random percentage
    skipIf=(int16_t)random(100)>=operand;
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
    delayMe(random(operand)*100L);
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
    DCC::setFn(operand,GET_OPERAND(1),true);
    break;
    
  case OPCODE_XFOFF:
    DCC::setFn(operand,GET_OPERAND(1),false);
    break;
    
  case OPCODE_DCCACTIVATE: {
    // operand is address<<3 | subaddr<<1 | active
    int16_t addr=operand>>3;
    int16_t subaddr=(operand>>1) & 0x03;
    bool active=operand & 0x01;
    DCC::setAccessory(addr,subaddr,active);
    break;
  }
    
  case OPCODE_FOLLOW:
    progCounter=sequenceLookup->find(operand);
    if (progCounter<0) kill(F("FOLLOW unknown"), operand);
    return;
    
  case OPCODE_CALL:
    if (stackDepth==MAX_STACK_DEPTH) {
      kill(F("CALL stack"), stackDepth);
      return;
    }
    callStack[stackDepth++]=progCounter+3;
    progCounter=sequenceLookup->find(operand);
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

  case OPCODE_JOIN:
    DCCWaveform::mainTrack.setPowerMode(POWERMODE::ON);
    DCCWaveform::progTrack.setPowerMode(POWERMODE::ON);
    DCC::setProgTrackSyncMain(true);
    CommandDistributor::broadcastPower();
    break;
  
  case OPCODE_POWERON:
    DCCWaveform::mainTrack.setPowerMode(POWERMODE::ON);
    DCC::setProgTrackSyncMain(false);
    CommandDistributor::broadcastPower();
    break;
    
  case OPCODE_UNJOIN:
    DCC::setProgTrackSyncMain(false);
    CommandDistributor::broadcastPower();
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
    
  case OPCODE_START:
    {
      int newPc=sequenceLookup->find(operand);
      if (newPc<0) break;
      new RMFT2(newPc);
    }
    break;
    
  case OPCODE_SENDLOCO:  // cab, route
    {
      int newPc=sequenceLookup->find(GET_OPERAND(1));
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
    
    
  case OPCODE_SERVO: // OPCODE_SERVO,V(vpin),OPCODE_PAD,V(position),OPCODE_PAD,V(profile),OPCODE_PAD,V(duration)
    IODevice::writeAnalogue(operand,GET_OPERAND(1),GET_OPERAND(2),GET_OPERAND(3));
    break;
    
  case OPCODE_WAITFOR: // OPCODE_SERVO,V(pin)
    if (IODevice::isBusy(operand)) {
      delayMe(100);
      return;
    }
    break;
    
  case OPCODE_PRINT:
    printMessage(operand);
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
  case OPCODE_ONTHROW:
  case OPCODE_ONACTIVATE: // Activate event catchers ignored here
  case OPCODE_ONDEACTIVATE:
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

void RMFT2::setFlag(VPIN id,byte onMask, byte offMask) {
   if (FLAGOVERFLOW(id)) return; // Outside range limit
   byte f=flags[id];
   f &= ~offMask;
   f |= onMask;
   flags[id]=f;
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

int16_t RMFT2::getSignalSlot(VPIN id) {
  for (int sigpos=0;;sigpos+=4) {
      VPIN sigid=GETFLASHW(RMFT2::SignalDefinitions+sigpos);
      if (sigid==0) { // end of signal list 
        DIAG(F("EXRAIL Signal %d not defined"), id);
        return -1;
      }
      // sigid is the signal id used in RED/AMBER/GREEN macro
      // for a LED signal it will be same as redpin
      // but for a servo signal it will also have SERVO_SIGNAL_FLAG set. 

      if ((sigid & SIGNAL_ID_MASK)!= id) continue; // keep looking
      return sigpos/4; // relative slot in signals table
  }  
}
/* static */ void RMFT2::doSignal(VPIN id,char rag) {
  if (diag) DIAG(F(" doSignal %d %x"),id,rag);
  int16_t sigslot=getSignalSlot(id);
  if (sigslot<0) return; 
  
  // keep track of signal state 
  setFlag(sigslot,rag,SIGNAL_MASK);
 
  // Correct signal definition found, get the rag values
  int16_t sigpos=sigslot*4; 
  VPIN sigid=GETFLASHW(RMFT2::SignalDefinitions+sigpos);
  VPIN redpin=GETFLASHW(RMFT2::SignalDefinitions+sigpos+1);
  VPIN amberpin=GETFLASHW(RMFT2::SignalDefinitions+sigpos+2);
  VPIN greenpin=GETFLASHW(RMFT2::SignalDefinitions+sigpos+3);
  if (diag) DIAG(F("signal %d %d %d %d"),sigid,redpin,amberpin,greenpin);

  if (sigid & SERVO_SIGNAL_FLAG) {
    // A servo signal, the pin numbers are actually servo positions
    // Note, setting a signal to a zero position has no effect.
    int16_t servopos= rag==SIGNAL_RED? redpin: (rag==SIGNAL_GREEN? greenpin : amberpin);
    if (diag) DIAG(F("sigA %d %d"),id,servopos);
    if  (servopos!=0) IODevice::writeAnalogue(id,servopos,PCA9685::Bounce);
    return;  
  }

  // LED or similar 3 pin signal
  // If amberpin is zero, synthesise amber from red+green
  const byte SIMAMBER=0x00;
  if (rag==SIGNAL_AMBER && (amberpin==0)) rag=SIMAMBER; // special case this func only
   
  // Manage invert (HIGH on) pins
  bool aHigh=sigid & ACTIVE_HIGH_SIGNAL_FLAG;
    
  // set the three pins 
  if (redpin) IODevice::write(redpin,(rag==SIGNAL_RED || rag==SIMAMBER)^aHigh);
  if (amberpin) IODevice::write(amberpin,(rag==SIGNAL_AMBER)^aHigh);
  if (greenpin) IODevice::write(greenpin,(rag==SIGNAL_GREEN || rag==SIMAMBER)^aHigh);
  return;  
}

/* static */ bool RMFT2::isSignal(VPIN id,char rag) {
  int16_t sigslot=getSignalSlot(id);
  if (sigslot<0) return false; 
  return (flags[sigslot] & SIGNAL_MASK) == rag;
}

void RMFT2::turnoutEvent(int16_t turnoutId, bool closed) {
  // Hunt for an ONTHROW/ONCLOSE for this turnout
  int pc= (closed?onCloseLookup:onThrowLookup)->find(turnoutId);
  if (pc<0) return;
  
  // Check we dont already have a task running this turnout
  RMFT2 * task=loopTask;
  while(task) {
    if (task->onTurnoutId==turnoutId) {
      DIAG(F("Recursive ONTHROW/ONCLOSE for Turnout %d"),turnoutId);
      return;
    }
    task=task->next;
    if (task==loopTask) break;
  }
  
  task=new RMFT2(pc);  // new task starts at this instruction
  task->onTurnoutId=turnoutId; // flag for recursion detector
}

void RMFT2::activateEvent(int16_t addr, bool activate) {
  // Hunt for an ONACTIVATE/ONDEACTIVATE for this accessory
  int pc= (activate?onActivateLookup:onDeactivateLookup)->find(addr);
  if (pc<0) return;
  
  // Check we dont already have a task running this address
  RMFT2 * task=loopTask;
  while(task) {
    if (task->onActivateAddr==addr) {
      DIAG(F("Recursive ON(DE)ACTIVATE for  %d"),addr);
      return;
    }
    task=task->next;
    if (task==loopTask) break;
  }
  
  task->onActivateAddr=addr; // flag for recursion detector
  task=new RMFT2(pc);  // new task starts at this instruction
}

void RMFT2::printMessage2(const FSH * msg) {
  DIAG(F("EXRAIL(%d) %S"),loco,msg);
}


