/*
 *  © 2020,2021 Chris Harlow. All rights reserved.
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
#include <Arduino.h>
#include "RMFT2.h"
#include "DCC.h"
#include "DIAG.h"
#include "WiThrottle.h"
#include "DCCEXParser.h"
#include "Turnouts.h"


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

// One instance of RMFT clas is used for each "thread" in the automation.
// Each thread manages a loco on a journey through the layout, and/or may manage a scenery automation.
// The thrrads exist in a ring, each time through loop() the next thread in the ring is serviced.

// Statics 
int16_t RMFT2::progtrackLocoId;  // used for callback when detecting a loco on prograck
bool RMFT2::diag=false;      // <D EXRAIL ON>  
RMFT2 * RMFT2::loopTask=NULL; // loopTask contains the address of ONE of the tasks in a ring.
RMFT2 * RMFT2::pausingTask=NULL; // Task causing a PAUSE. 
 // when pausingTask is set, that is the ONLY task that gets any service,
 // and all others will have their locos stopped, then resumed after the pausing task resumes.
byte RMFT2::flags[MAX_FLAGS];

#define GET_OPCODE GETFLASH(RMFT2::RouteCode+progCounter)
#define GET_OPERAND(n) GETFLASHW(RMFT2::RouteCode+progCounter+1+(n*3))
#define SKIPOP progCounter+=3

/* static */ void RMFT2::begin() { 
  DCCEXParser::setRMFTFilter(RMFT2::ComandFilter);
  for (int f=0;f<MAX_FLAGS;f++) flags[f]=0;
  int progCounter;
  // first pass startup, define any turnouts or servos and count size.
  for (progCounter=0;; SKIPOP){
     byte opcode=GET_OPCODE;
     if (opcode==OPCODE_ENDEXRAIL) break;

     if (opcode==OPCODE_SIGNAL) {
      VPIN red=GET_OPERAND(0);
      VPIN amber=GET_OPERAND(1);
      VPIN green=GET_OPERAND(2);
      IODevice::write(red,true);
      if (amber) IODevice::write(amber,false);
      IODevice::write(green,false);
      continue;
     }
     
     if (opcode==OPCODE_TURNOUT) {
      VPIN id=GET_OPERAND(0);
      int addr=GET_OPERAND(1);
      byte subAddr=GET_OPERAND(2);
      Turnout::createDCC(id,addr,subAddr);
      continue;
     }

     if (opcode==OPCODE_SERVOTURNOUT) {
      VPIN id=GET_OPERAND(0);
      int activeAngle=GET_OPERAND(1);
      int inactiveAngle=GET_OPERAND(2);
      Turnout::createServo(id,id,activeAngle,inactiveAngle);
      continue;
     }

     if (opcode==OPCODE_PINTURNOUT) {
      VPIN id=GET_OPERAND(0);
      Turnout::createVpin(id,id);
      continue;
     }
     // other opcodes are not needed on this pass
  } 
  SKIPOP; // include ENDROUTES opcode
  DIAG(F("EXRAIL %db, MAX_FLAGS=%d"), progCounter,MAX_FLAGS);
  new RMFT2(0); // add the startup route
}

// This filter intercepst <> commands to do the following:
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

      case 't': // THROTTLE <t [REGISTER] CAB SPEED DIRECTION>          
          // TODO - Monitor throttle commands and reject any that are in current automation
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
                   StringFormatter::send(stream,F("\nPC=%d,DT=%d,LOCO=%d%c,SPEED=%d%c"),
                         task->progCounter,task->delayTime,task->loco,
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
                   if (flag) {
                     StringFormatter::send(stream,F("\nflags[%d} "),id);
                     if (flag & SECTION_FLAG) StringFormatter::send(stream,F(" RESERVED"));
                     if (flag & LATCH_FLAG) StringFormatter::send(stream,F(" LATCHED"));
                     }                 
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
                  int pc=locateRouteStart(route);                 
                  if (pc<0) return false;
                    RMFT2* task=new RMFT2(pc);
                    task->loco=cab;                    
                 }
              return true;
                 
            default:
              break;
          }
          
          // all other / commands take 1 parameter 0 to MAX_FLAGS-1     

          if (paramCount!=2 || p[1]<0  || p[1]>=MAX_FLAGS) return false;

          switch (p[0]) {     
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
                  
            default:
                 return false;                 
          }
    }


// This emits Routes and Automations to Withrottle
// Automations are given a state to set the button to "handoff" which implies 
// handing over the loco to the automation.
// Routes are given "Set" buttons and do not cause the loco to be handed over. 
void RMFT2::emitWithrottleRouteList(Print* stream) {
   StringFormatter::send(stream,F("PRT]\\[Routes}|{Route]\\[Set}|{2]\\[Handoff}|{4\nPRL%S\n"),RouteDescription);
}


RMFT2::RMFT2(int progCtr) {
  progCounter=progCtr;
  delayTime=0;
  loco=0;
  speedo=0;
  forward=true;
  invert=false;
  stackDepth=0;
   
  // chain into ring of RMFTs
  if (loopTask==NULL) {
    loopTask=this;
    next=this;
  }
  else {
        next=loopTask->next;
        loopTask->next=this;
  }
}


RMFT2::~RMFT2() {
  if (next==this) loopTask=NULL;
  else for (RMFT2* ring=next;;ring=ring->next) if (ring->next == this) {
           ring->next=next;
           loopTask=next;
           break;
       }
}

void RMFT2::createNewTask(int route, uint16_t cab) {
      int pc=locateRouteStart(route);
      if (pc<0) return;
      RMFT2* task=new RMFT2(pc);
      task->loco=cab;
}

   
int RMFT2::locateRouteStart(int16_t _route) {
  if (_route==0) return 0; // Route 0 is always start of ROUTES for default startup 
  for (int progCounter=0;;SKIPOP) {
    byte opcode=GET_OPCODE;
    if (opcode==OPCODE_ENDEXRAIL) {
      DIAG(F("RMFT2 sequence %d not found"), _route);
      return -1;
    }
    if ((opcode==OPCODE_ROUTE || opcode==OPCODE_AUTOMATION || opcode==OPCODE_SEQUENCE) 
       &&  _route==(int)GET_OPERAND(0)) return progCounter;
  }
  return -1;
}


void RMFT2::driveLoco(byte speed) {
     if (loco<0) return;  // Caution, allows broadcast! 
     if (diag) DIAG(F("EXRAIL drive %d %d %d"),loco,speed,forward^invert);
     DCC::setThrottle(loco,speed, forward^invert);
     speedo=speed;
     // TODO... if broadcast speed 0 then pause all other tasks. 
}

bool RMFT2::readSensor(int16_t sensorId) {
  VPIN vpin=abs(sensorId);
  if (getFlag(vpin,LATCH_FLAG)) return true; // latched on
  bool s= IODevice::read(vpin) ^ (sensorId<0);
  if (s && diag) DIAG(F("EXRAIL Sensor %d hit"),sensorId);
  return s;
}

bool RMFT2::skipIfBlock() {
  // returns false if killed
  short nest = 1;
  while (nest > 0) {
    SKIPOP;
    byte opcode =  GET_OPCODE;
    switch(opcode) {
      case OPCODE_ENDEXRAIL: 
           kill(F("missing ENDIF"), nest);
           return false;  
      case OPCODE_IF:
      case OPCODE_IFNOT:
      case OPCODE_IFRANDOM:
      case OPCODE_IFRESERVE:
           nest++;
           break;
      case OPCODE_ENDIF:
           nest--;
           break;
      default:
      break;
    }
  }
  return true; 
}



/* static */ void RMFT2::readLocoCallback(int cv) {
     progtrackLocoId=cv;
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
  // if (diag) DIAG(F("RMFT2 %d %d"),opcode,operand); 
  // Attention: Returning from this switch leaves the program counter unchanged.
  //            This is used for unfinished waits for timers or sensors.
  //            Breaking from this switch will step to the next step in the route. 
  switch ((OPCODE)opcode) {
    
    case OPCODE_THROW:
         Turnout::activate(operand, true);
         break;
          
    case OPCODE_CLOSE:
         Turnout::activate(operand, false);
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
      if (readSensor(operand)) break;
      delayMe(50);
      return;
    
    case OPCODE_AFTER: // waits for sensor to hit and then remain off for 0.5 seconds. (must come after an AT operation)
      if (readSensor(operand)) {
        // reset timer to half a second and keep waiting
        waitAfter=millis();
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
        if (loco!=0) {
          DCC::writeCVByteMain(loco, operand, GET_OPERAND(1));
        }        
        break;

    case OPCODE_RESUME:
         pausingTask=NULL;
         driveLoco(speedo);
         for (RMFT2 * t=next; t!=this;t=t->next) if (t->loco >0) t->driveLoco(t->speedo);
          break;        
    
    case OPCODE_IF: // do next operand if sensor set
      if (!readSensor(operand)) if (!skipIfBlock()) return;
      break;
    
    case OPCODE_IFNOT: // do next operand if sensor not set
      if (readSensor(operand)) if (!skipIfBlock()) return;
      break;
   
    case OPCODE_IFRANDOM: // do block on random percentage
      if (random(100)>=operand) if (!skipIfBlock()) return;
      break;
   
    case OPCODE_IFRESERVE: // do block if we successfully RERSERVE
      if (!getFlag(operand,SECTION_FLAG)) setFlag(operand,SECTION_FLAG);
      else if (!skipIfBlock()) return;
      break;
      
    case OPCODE_ENDIF:
      break;
    
    case OPCODE_DELAY:
      delayMe(operand*100);
      break;
   
    case OPCODE_DELAYMINS:
      delayMe(operand*60*1000);
      break;
    
    case OPCODE_RANDWAIT:
      delayMe((long)random(operand*100));
      break;
    
    case OPCODE_RED:
      doSignal(operand,true,false,false);
      break;
    
    case OPCODE_AMBER:
      doSignal(operand,false,true,false);
      break;
    
    case OPCODE_GREEN:
      doSignal(operand,false,false,true);
      break;
       
    case OPCODE_FON:      
      DCC::setFn(loco,operand,true);
      break;
    
    case OPCODE_FOFF:
      DCC::setFn(loco,operand,false);
      break;

    case OPCODE_FOLLOW:
      progCounter=locateRouteStart(operand);
      if (progCounter<0) kill(F("FOLLOW unknown"), operand); 
      return;
  
    case OPCODE_CALL:
      if (stackDepth==MAX_STACK_DEPTH) {
        kill(F("CALL stack"), stackDepth);
        return;
      }
      callStack[stackDepth++]=progCounter;
      progCounter=locateRouteStart(operand);
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
      
    case OPCODE_JOIN:
       DCC::setProgTrackSyncMain(true);
       break;

    case OPCODE_UNJOIN:
       DCC::setProgTrackSyncMain(false);
       break;
       
    case OPCODE_READ_LOCO1: // READ_LOCO is implemented as 2 separate opcodes
       DCC::getLocoId(readLocoCallback);
       break;
      
      case OPCODE_READ_LOCO2:
       if (progtrackLocoId<0) {
        delayMe(100);
        return; // still waiting for callback
       }
       loco=progtrackLocoId;
       speedo=0;
       forward=true;
       invert=false;
       break;
       
       case OPCODE_START:
           {
            // Create new task and transfer loco.....
            // but cheat by swapping prog counters with new task 
            int newPc=locateRouteStart(operand);
            if (newPc<0) break; 
            new RMFT2(progCounter+3); // give new task my prog counter
            progCounter=newPc;  // and I'll carry on from new task position
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

          
       case OPCODE_SERVO: // OPCODE_SERVO,V(id),OPCODE_PAD,V(position),OPCODE_PAD,V(profile),
        IODevice::writeAnalogue(operand,GET_OPERAND(1),GET_OPERAND(2));          
        break;
          
       case OPCODE_ROUTE:
       case OPCODE_AUTOMATION:
       case OPCODE_SEQUENCE:
          DIAG(F("EXRAIL begin(%d)"),operand);
          break;

       case OPCODE_PAD: // Just a padding for previous opcode needing >1 operad byte.
       case OPCODE_SIGNAL: // Signal definition ignore at run time
       case OPCODE_TURNOUT: // Turnout definition ignored at runtime
       case OPCODE_SERVOTURNOUT: // Turnout definition ignored at runtime
       case OPCODE_PINTURNOUT: // Turnout definition ignored at runtime
       case OPCODE_ONCLOSE: // Turnout event catcers ignored here
       case OPCODE_ONTHROW: // Turnout definition ignored at runtime
       break;
    
    default:
      kill(F("INVOP"),operand);
    }
    // Falling out of the switch means move on to the next opcode
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
}

byte RMFT2::getFlag(VPIN id,byte mask) {
   if (FLAGOVERFLOW(id)) return 0; // Outside range limit
   return flags[id]&mask;   
}

void RMFT2::kill(const FSH * reason, int operand) {
     if (reason) DIAG(F("EXRAIL ERROR pc=%d, cab=%d, %S %d"), progCounter,loco, reason, operand);
     else if (diag) DIAG(F("ENDTASK at pc=%d"), progCounter); 
     delete this;
}

/* static */ void RMFT2::doSignal(VPIN id,bool red, bool amber, bool green) { 
  // CAUTION: hides class member progCounter
  for (int progCounter=0;; SKIPOP){
     byte opcode=GET_OPCODE;
     if (opcode==OPCODE_ENDEXRAIL) return;
     if (opcode!=OPCODE_SIGNAL) continue;
     byte redpin=GET_OPERAND(1);
     if (redpin!=id)continue;
     byte amberpin=GET_OPERAND(2);
     byte greenpin=GET_OPERAND(3);
     IODevice::write(redpin,red);
     if (amberpin) IODevice::write(amberpin,amber);
     if (greenpin) IODevice::write(amberpin,green);
     return;
   }
  } 
 void RMFT2::turnoutEvent(VPIN id, bool thrown) {
    byte huntFor=thrown? OPCODE_ONTHROW : OPCODE_ONCLOSE;
    // caution hides class progCounter;
    for (int progCounter=0;; SKIPOP){
     byte opcode=GET_OPCODE;
     if (opcode==OPCODE_ENDEXRAIL) return;
     if (opcode!=huntFor) continue;
     if (id!=GET_OPERAND(0)) continue;
     new RMFT2(progCounter);  // new task starts at this instruction
     return;
   }
  }
