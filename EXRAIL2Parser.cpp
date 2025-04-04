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
#include "DCCEXParser.h"
#include "DCCEXParserMacros.h"

// This filter intercepts <> commands to do the following:
// - Implement RMFT specific commands/diagnostics
// - Reject/modify JMRI commands that would interfere with RMFT processing

void RMFT2::ComandFilter(Print * stream, byte & opcode, byte & paramCount, int16_t p[]) {
    if (parseCommands(stream,opcode,paramCount,p)) opcode='\0'; // command was handled by parseCommands() 
}

void RMFT2::streamStatus(Print * stream) {
  REPLY("<* EXRAIL STATUS")
    auto task=loopTask;
    while(task) {
      if ((compileFeatures & FEATURE_BLINK)
       && (task->blinkState==blink_high || task->blinkState==blink_low)) {
        REPLY("\nID=%d,PC=%d,BLINK=%d",(int)(task->taskId),task->progCounter,task->blinkPin)
      }
      else {
       REPLY("\nID=%d,PC=%d,LOCO=%d %c",(int)(task->taskId),task->progCounter,task->loco,task->invert?'I':' ')
      }
      task=task->next;
      if (task==loopTask) break;
    }
    // Now stream the flags
    for (int id=0;id<MAX_FLAGS; id++) {
      byte flag=flags[id];
      if (flag & ~TASK_FLAG & ~SIGNAL_MASK) { // not interested in TASK_FLAG only. Already shown above
	      REPLY("\nflags[%d] ",id);
	      if (flag & SECTION_FLAG) REPLY(" RESERVED");
	      if (flag & LATCH_FLAG) REPLY(" LATCHED");
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
        REPLY("\n%S[%d]", 
			      (flag == SIGNAL_RED)? F("RED") : (flag==SIGNAL_GREEN) ? F("GREEN") : F("AMBER"),
			      slot.id)
      } 
    }
    REPLY(" *>\n")
}
// 
bool RMFT2::streamLCC(Print * stream) {
  if (!(compileFeatures & FEATURE_LCC)) return false;
  // This function is called to stream the LCC commands to the LCC adapter.
  LCCSerial=stream;   // now we know where to send events we raise
   
  static int lccProgCounter=0;
  static int lccEventIndex=0;
for (int progCounter=lccProgCounter;; SKIPOP) {
  byte exrailOpcode=GET_OPCODE;
  switch (exrailOpcode) {
    case OPCODE_ENDEXRAIL:
         stream->print(F("<LR>\n")); // ready to roll
         lccProgCounter=0; // allow a second pass
         lccEventIndex=0;
         return true;

    case OPCODE_LCC:  
         StringFormatter::send(stream,F("<LS x%h>\n"),getOperand(progCounter,0));
         SKIPOP;
         lccProgCounter=progCounter; 
         return true;

    case OPCODE_LCCX:  // long form LCC
         StringFormatter::send(stream,F("<LS x%h%h%h%h>\n"),
           getOperand(progCounter,1),
           getOperand(progCounter,2),
           getOperand(progCounter,3),
           getOperand(progCounter,0)
           );
         SKIPOP;SKIPOP;SKIPOP;SKIPOP;          
         lccProgCounter=progCounter; 
         return true;

    case OPCODE_ACON:  // CBUS ACON 
    case OPCODE_ACOF:  // CBUS ACOF 
          StringFormatter::send(stream,F("<LS x%c%h%h>\n"),
            exrailOpcode==OPCODE_ACOF?'1':'0',
            getOperand(progCounter,0),getOperand(progCounter,1)); 
         SKIPOP;SKIPOP;
         lccProgCounter=progCounter; 
         return true;

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
     return true;

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
     return true;
     
   default:
     break;
  }  
}
return true;
}

bool RMFT2::parseCommands(Print * stream, byte opcode, byte params, int16_t p[]) {

  ZZBEGIN
    ZZ(D,EXRAIL,ON) diag=1; // <D EXRAIL ON>  - turn on diagnostics
    ZZ(D,EXRAIL,OFF) diag=0; // <D EXRAIL OFF> - turn off diagnostics 
    ZZ(D,EXRAIL,onoff) diag=onoff;
    ZZ(A,address,aspect) // Aspect may intercept or be left for normal parse
                     return signalAspectEvent(address,aspect);
    ZZ(L)  //LCC adapter introducing self
          CHECK(streamLCC(stream),no LCC/CBUS events)
    ZZ(L,eventid)  // loop through all possible sent/waited events 
        CHECK(eventid>=0 && eventid<countLCCLookup)
        startNonRecursiveTask(F("LCC"),eventid,onLCCLookup[eventid]);
    
    ZZ(J,A)     REPLY("<jA") routeLookup->stream(stream); REPLY(">\n")
    ZZ(J,A,id)  REPLY("<jA %d %c \"%S\">\n",id, getRouteType(id), getRouteDescription(id));             
              if (compileFeatures & FEATURE_ROUTESTATE) {
                // Send any non-default button states or captions
                int16_t statePos=routeLookup->findPosition(id);
                if (statePos>=0) {
                  if (routeStateArray[statePos]) 
                    REPLY("<jB %d %d>\n", id, routeStateArray[statePos]);
                  if (routeCaptionArray[statePos]) 
                    REPLY("<jB %d \"%S\">\n", id,routeCaptionArray[statePos]);
                }
              }
    ZZ(K,blockid,loco) blockEvent(blockid,loco,true);
    ZZ(k,blockid,loco) blockEvent(blockid,loco,false);
    ZZ(/) streamStatus(stream);
    ZZ(/,PAUSE) 
      // pause all tasks 
      RMFT2 * task=loopTask;
      while(task) {
	      task->pause();
	      task=task->next;
	      if (task==loopTask) break;
      }
      DCC::estopAll();  // pause all locos on the track
      pausingTask=(RMFT2 *)1; // Impossible task address
  
    ZZ(/,RESUME)
      pausingTask=NULL;
      RMFT2 * task=loopTask;
      while(task) {
	      task->resume();
	      task=task->next;
	      if (task==loopTask) break;
      }
  ZZ(/,START,route)
      auto pc=routeLookup->find(route);
      CHECK(pc>=0,route not found)
      new RMFT2(pc,0); // no cab for route start

  ZZ(/,START,cab,route)
      auto pc=routeLookup->find(route);
      CHECK(pc>=0, route not found)
      new RMFT2(pc,cab); // no cab for route start
 
  ZZ(/,KILL,ALL) while (loopTask) loopTask->kill(F("KILL ALL")); // destructor changes loopTask
  ZZ(/,KILL,taskid) 
    CHECK(taskid>=0 && taskid<MAX_FLAGS)
    auto task=loopTask;
    bool found=false;
    while(task) {
	      if (task->taskId==taskid) {
          found=true;
	        task->kill(F("KILL"));
	        break;
	      }
	      task=task->next;
	      if (task==loopTask) break;
      }
    CHECK(found);
  ZZ(/,RESERVE,section)  CHECK(setFlag(section,SECTION_FLAG),invalid section)
  ZZ(/,FREE,section)  CHECK(setFlag(section,0,SECTION_FLAG),invalid section)
  ZZ(/,LATCH,latch) CHECK(setFlag(latch,LATCH_FLAG),invalid section)
  ZZ(/,UNLATCH,latch) CHECK(setFlag(latch,0,LATCH_FLAG),invalid section)
  ZZ(/,RED,signal)   doSignal(signal,SIGNAL_RED);
  ZZ(/,AMBER,signal) doSignal(signal,SIGNAL_AMBER);
  ZZ(/,GREEN,signal) doSignal(signal,SIGNAL_GREEN);
ZZEND
}
