/*
 *  © 2021 Neil McKechnie
 *  © 2021-2023 Harald Barth
 *  © 2020-2025 Chris Harlow
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

// THIS file is an extension of the RMFT2 class 
//  normally found in EXRAIL2.cpp

#include <Arduino.h>
#include "defines.h"
#include "EXRAIL2.h"
#include "DCC.h"
#include "KeywordHasher.h"

// This filter intercepts <> commands to do the following:
// - Implement RMFT specific commands/diagnostics
// - Reject/modify JMRI commands that would interfere with RMFT processing

void RMFT2::ComandFilter(Print * stream, byte & opcode, byte & paramCount, int16_t p[]) {
  (void)stream; // avoid compiler warning if we don't access this parameter
  
  switch(opcode) {
    
  case 'D':
    if (p[0]=="EXRAIL"_hk) { // <D EXRAIL ON/OFF>
      diag = paramCount==2 && (p[1]=="ON"_hk || p[1]==1);
      opcode=0;
    }
    break;
	
  case '/':  // New EXRAIL command
    if (parseSlash(stream,paramCount,p)) opcode=0;
    break;
  
  case 'A': //  <A address aspect>
    if (paramCount!=2) break; 
    // Ask exrail if this is just changing the aspect on a 
    // predefined DCCX_SIGNAL. Because this will handle all 
    // the IFRED and ONRED type issues at the same time.  
    if (signalAspectEvent(p[0],p[1])) opcode=0; // all done 
    break;

  case 'L':
    // This entire code block is compiled out if LLC macros not used 
    if (!(compileFeatures & FEATURE_LCC)) return;
    static int lccProgCounter=0;
    static int lccEventIndex=0;
      
    if (paramCount==0) {  //<L>  LCC adapter introducing self
      LCCSerial=stream;   // now we know where to send events we raise
      opcode=0;  // flag command as intercepted

      // loop through all possible sent/waited events 
      for (int progCounter=lccProgCounter;; SKIPOP) {
        byte exrailOpcode=GET_OPCODE;
        switch (exrailOpcode) {
          case OPCODE_ENDEXRAIL:
               stream->print(F("<LR>\n")); // ready to roll
               lccProgCounter=0; // allow a second pass
               lccEventIndex=0;
               return;

          case OPCODE_LCC:  
               StringFormatter::send(stream,F("<LS x%h>\n"),getOperand(progCounter,0));
               SKIPOP;
               lccProgCounter=progCounter; 
               return;

          case OPCODE_LCCX:  // long form LCC
               StringFormatter::send(stream,F("<LS x%h%h%h%h>\n"),
                 getOperand(progCounter,1),
                 getOperand(progCounter,2),
                 getOperand(progCounter,3),
                 getOperand(progCounter,0)
                 );
               SKIPOP;SKIPOP;SKIPOP;SKIPOP;          
               lccProgCounter=progCounter; 
               return;

          case OPCODE_ACON:  // CBUS ACON 
          case OPCODE_ACOF:  // CBUS ACOF 
                StringFormatter::send(stream,F("<LS x%c%h%h>\n"),
                  exrailOpcode==OPCODE_ACOF?'1':'0',
                  getOperand(progCounter,0),getOperand(progCounter,1)); 
               SKIPOP;SKIPOP;
               lccProgCounter=progCounter; 
               return;
      
      // we stream the hex events we wish to listen to
      // and at the same time build the event index looku.
      
        case OPCODE_ONLCC:
           StringFormatter::send(stream,F("<LL %d x%h%h%h:%h>\n"),
                 lccEventIndex,
                 getOperand(progCounter,1),
                 getOperand(progCounter,2),
                 getOperand(progCounter,3),
                 getOperand(progCounter,0)
                 );   
           SKIPOP;SKIPOP;SKIPOP;SKIPOP;      
           // start on handler at next      
           onLCCLookup[lccEventIndex]=progCounter; 
           lccEventIndex++;        
           lccProgCounter=progCounter; 
           return;

        case OPCODE_ONACON:
        case OPCODE_ONACOF:
           StringFormatter::send(stream,F("<LL %d x%c%h%h>\n"),
                 lccEventIndex,
                 exrailOpcode==OPCODE_ONACOF?'1':'0',
                 getOperand(progCounter,0),getOperand(progCounter,1)
                 ); 
           SKIPOP;SKIPOP;
           // start on handler at next      
           onLCCLookup[lccEventIndex]=progCounter; 
           lccEventIndex++;        
           lccProgCounter=progCounter; 
           return;
           
         default:
           break;
        }  
      }
    }
    if (paramCount==1) {  // <L eventid> LCC event arrived from adapter
        int16_t eventid=p[0];
        bool reject = eventid<0 || eventid>=countLCCLookup;
        if (!reject) {
          startNonRecursiveTask(F("LCC"),eventid,onLCCLookup[eventid]);
          opcode=0;
        }
    }
    break; 
    
    case 'J':  // throttle info commands
        if (paramCount<1) return; 
        switch(p[0]) {
          case "A"_hk: // <JA> returns automations/routes
            if (paramCount==1) {// <JA>
              StringFormatter::send(stream, F("<jA"));
              routeLookup->stream(stream);
              StringFormatter::send(stream, F(">\n"));
              opcode=0;
              return; 
            }
            if (paramCount==2) {  // <JA id>
              int16_t id=p[1];
              StringFormatter::send(stream,F("<jA %d %c \"%S\">\n"), 
                id, getRouteType(id), getRouteDescription(id));
              
              if (compileFeatures & FEATURE_ROUTESTATE) {
                // Send any non-default button states or captions
                int16_t statePos=routeLookup->findPosition(id);
                if (statePos>=0) {
                 if (routeStateArray[statePos]) 
                 StringFormatter::send(stream,F("<jB %d %d>\n"), id, routeStateArray[statePos]);
                  if (routeCaptionArray[statePos]) 
                  StringFormatter::send(stream,F("<jB %d \"%S\">\n"), id,routeCaptionArray[statePos]);
                }
              }
              opcode=0;
              return;
            }
            break;
        

  case 'K': // <K blockid loco>  Block enter
  case 'k': // <k blockid loco>  Block exit
        if (paramCount!=2) break;
        blockEvent(p[0],p[1],opcode=='K');
        opcode=0;
        break; 
  
  default:  // other commands pass through
    break;
  }
}
}

bool RMFT2::parseSlash(Print * stream, byte & paramCount, int16_t p[]) {

  if (paramCount==0) { // STATUS
    StringFormatter::send(stream, F("<* EXRAIL STATUS"));
    RMFT2 * task=loopTask;
    while(task) {
      if ((compileFeatures & FEATURE_BLINK)
       && (task->blinkState==blink_high || task->blinkState==blink_low)) {
        StringFormatter::send(stream,F("\nID=%d,PC=%d,BLINK=%d"),
			    (int)(task->taskId),task->progCounter,task->blinkPin
			    );
      }
      else {
      StringFormatter::send(stream,F("\nID=%d,PC=%d,LOCO=%d %c"),
			    (int)(task->taskId),task->progCounter,task->loco,
			    task->invert?'I':' '
			    );
                auto progCounter=task->progCounter; // name to satisfy macros below
          auto operand=task->getOperand(progCounter,0);
          switch(GET_OPCODE) {
              case OPCODE_RESERVE:
                StringFormatter::send(stream,F(" WAIT RESERVE %d"),operand);
                break;
              case OPCODE_AT:
              case OPCODE_ATTIMEOUT2:
              case OPCODE_AFTER:
              case OPCODE_ATGTE:
              case OPCODE_ATLT:
                StringFormatter::send(stream,F(" WAIT AT/AFTER %d"),operand);
                break;
              case OPCODE_DELAY:
              case OPCODE_DELAYMINS:
              case OPCODE_DELAYMS:
              case OPCODE_RANDWAIT:
                StringFormatter::send(stream,F(" WAIT DELAY"));
                break; 
            default: break;
          }
      }
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

    if (compileFeatures & FEATURE_SIGNAL) {
      // do the signals
      // flags[n] represents the state of the nth signal in the table 
      for (int sigslot=0;;sigslot++) {
        SIGNAL_DEFINITION slot=getSignalSlot(sigslot);
        if (slot.type==sigtypeNoMoreSignals) break; // end of signal list
	      if (slot.type==sigtypeContinuation) continue; // continueation of previous line
	      byte flag=flags[sigslot] & SIGNAL_MASK; // obtain signal flags for this ids
        StringFormatter::send(stream,F("\n%S[%d]"), 
			      (flag == SIGNAL_RED)? F("RED") : (flag==SIGNAL_GREEN) ? F("GREEN") : F("AMBER"),
			      slot.id);
      } 
    }
    StringFormatter::send(stream,F(" *>\n"));
    return true;
  }
  switch (p[0]) {
  case "PAUSE"_hk: // </ PAUSE>
    if (paramCount!=1) return false;
    { // pause all tasks 
      RMFT2 * task=loopTask;
      while(task) {
	      task->pause();
	      task=task->next;
	      if (task==loopTask) break;
      }
    }
    DCC::estopAll();  // pause all locos on the track
    pausingTask=(RMFT2 *)1; // Impossible task address
    return true;
    
  case "RESUME"_hk: // </ RESUME>
    if (paramCount!=1) return false;
    pausingTask=NULL;
    { // resume all tasks
      RMFT2 * task=loopTask;
      while(task) {
	      task->resume();
	      task=task->next;
	      if (task==loopTask) break;
      }
    }
    return true;
    
    
  case "START"_hk: // </ START [cab] route >
    if (paramCount<2 || paramCount>3) return false;
    {
      int route=(paramCount==2) ? p[1] : p[2];
      uint16_t cab=(paramCount==2)? 0 : p[1];
      int pc=routeLookup->find(route);
      if (pc<0) return false;
      new RMFT2(pc,cab);
    }
    return true;
    
  default:
    break;
  }

  // check KILL ALL here, otherwise the next validation confuses ALL with a flag  
  if (p[0]=="KILL"_hk && p[1]=="ALL"_hk) {
    while (loopTask) loopTask->kill(F("KILL ALL")); // destructor changes loopTask
    return true;   
  }

  // all other / commands take 1 parameter
  if (paramCount!=2 ) return false;
  
  switch (p[0]) {
  case "KILL"_hk: // Kill taskid|ALL
    {
    if ( p[1]<0  || p[1]>=MAX_FLAGS) return false;
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
    
  case "RESERVE"_hk:  // force reserve a section
    return setFlag(p[1],SECTION_FLAG);
    
  case "FREE"_hk:  // force free a section
    return setFlag(p[1],0,SECTION_FLAG);
    
  case "LATCH"_hk:
    return setFlag(p[1], LATCH_FLAG);
    
  case "UNLATCH"_hk:
    return setFlag(p[1], 0, LATCH_FLAG);
 
  case "RED"_hk:
    doSignal(p[1],SIGNAL_RED);
    return true;
 
  case "AMBER"_hk:
    doSignal(p[1],SIGNAL_AMBER);
    return true;
 
  case "GREEN"_hk:
    doSignal(p[1],SIGNAL_GREEN);
    return true;
    
  default:
    return false;
  }
}
