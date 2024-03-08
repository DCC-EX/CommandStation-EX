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
  bool reject=false;
  switch(opcode) {
    
  case 'D':
    if (p[0]=="EXRAIL"_hk) { // <D EXRAIL ON/OFF>
      diag = paramCount==2 && (p[1]=="ON"_hk || p[1]==1);
      opcode=0;
    }
    break;
	
  case '/':  // New EXRAIL command
    reject=!parseSlash(stream,paramCount,p);
    opcode=0;
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

    if (paramCount==0) {  //<L>  LCC adapter introducing self
      LCCSerial=stream;   // now we know where to send events we raise

      // loop through all possible sent events 
      for (int progCounter=0;; SKIPOP) {
        byte opcode=GET_OPCODE;
        if (opcode==OPCODE_ENDEXRAIL) break;
        if (opcode==OPCODE_LCC)  StringFormatter::send(stream,F("<LS x%h>\n"),getOperand(progCounter,0));   
        if (opcode==OPCODE_LCCX) { // long form LCC
           StringFormatter::send(stream,F("<LS x%h%h%h%h>\n"),
                 getOperand(progCounter,1),
                 getOperand(progCounter,2),
                 getOperand(progCounter,3),
                 getOperand(progCounter,0)
                 );        
        }}
      
      // we stream the hex events we wish to listen to
      // and at the same time build the event index looku.
      
      
      int eventIndex=0;
      for (int progCounter=0;; SKIPOP) {
        byte opcode=GET_OPCODE;
        if (opcode==OPCODE_ENDEXRAIL) break;
        if (opcode==OPCODE_ONLCC) {
           onLCCLookup[eventIndex]=progCounter; // TODO skip...
           StringFormatter::send(stream,F("<LL %d x%h%h%h:%h>\n"),
                 eventIndex,
                 getOperand(progCounter,1),
                 getOperand(progCounter,2),
                 getOperand(progCounter,3),
                 getOperand(progCounter,0)
                 );   
           eventIndex++;      
        }
      }
      StringFormatter::send(stream,F("<LR>\n")); // Ready to rumble
      opcode=0;
      break;
    }
    if (paramCount==1) {  // <L eventid> LCC event arrived from adapter
        int16_t eventid=p[0];
        reject=eventid<0 || eventid>=countLCCLookup;
        if (!reject)  startNonRecursiveTask(F("LCC"),eventid,onLCCLookup[eventid]);
        opcode=0;
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
        case "M"_hk:
            // NOTE: we only need to handle valid calls here because 
            // DCCEXParser has to have code to handle the <J<> cases where
            // exrail isnt involved anyway. 
            // This entire code block is compiled out if STASH macros not used 
          if (!(compileFeatures & FEATURE_STASH)) return;
          if (paramCount==1) { // <JM>
              StringFormatter::send(stream,F("<jM %d>\n"),maxStashId);
              opcode=0;
              break;
            } 
          if (paramCount==2) {  // <JM id>
              if (p[1]<=0 || p[1]>maxStashId) break;
              StringFormatter::send(stream,F("<jM %d %d>\n"),
                    p[1],stashArray[p[1]]);
               opcode=0;     
               break;    
          } 
          if (paramCount==3) {  // <JM id cab>
              if (p[1]<=0 || p[1]>maxStashId) break;
              stashArray[p[1]]=p[2];
              opcode=0;
              break;      
          }
          break; 

        default:
            break;
        }
  default:  // other commands pass through
    break;
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

    if (compileFeatures & FEATURE_SIGNAL) {
      // do the signals
      // flags[n] represents the state of the nth signal in the table 
      for (int sigslot=0;;sigslot++) {
        VPIN sigid=GETHIGHFLASHW(RMFT2::SignalDefinitions,sigslot*8);
        if (sigid==0) break; // end of signal list 
        byte flag=flags[sigslot] & SIGNAL_MASK; // obtain signal flags for this id
        StringFormatter::send(stream,F("\n%S[%d]"), 
          (flag == SIGNAL_RED)? F("RED") : (flag==SIGNAL_GREEN) ? F("GREEN") : F("AMBER"),
          sigid & SIGNAL_ID_MASK); 
      } 
    }

    if (compileFeatures & FEATURE_STASH) {
      for (int i=1;i<=maxStashId;i++) {
        if (stashArray[i])
          StringFormatter::send(stream,F("\nSTASH[%d] Loco=%d"),
              i, stashArray[i]); 
      } 
    }
    
    StringFormatter::send(stream,F(" *>\n"));
    return true;
  }
  switch (p[0]) {
  case "PAUSE"_hk: // </ PAUSE>
    if (paramCount!=1) return false;
    DCC::setThrottle(0,1,true);  // pause all locos on the track
    pausingTask=(RMFT2 *)1; // Impossible task address
    return true;
    
  case "RESUME"_hk: // </ RESUME>
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
    
    
  case "START"_hk: // </ START [cab] route >
    if (paramCount<2 || paramCount>3) return false;
    {
      int route=(paramCount==2) ? p[1] : p[2];
      uint16_t cab=(paramCount==2)? 0 : p[1];
      int pc=routeLookup->find(route);
      if (pc<0) return false;
      RMFT2* task=new RMFT2(pc);
      task->loco=cab;
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

