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

bool RMFT2::streamLCC(Print * stream) {
  (void)stream; // avoid compiler warning if we don't access this parameter
    // This entire code block is compiled out if LLC macros not used 
    if (!(compileFeatures & FEATURE_LCC)) return false;
    static int lccProgCounter=0;
    static int lccEventIndex=0;
      
    LCCSerial=stream;   // now we know where to send events we raise
    
      // loop through all possible sent/waited events 
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
      return false; // nothing found
    }


bool RMFT2::streamStatus(Print * stream) {

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
                StringFormatter::send(stream,F(" WAIT AT/AFTER %d"),(int16_t)operand);
                break;
              case OPCODE_WAIT_WHILE_RED:
                StringFormatter::send(stream,F(" WAIT WHILE RED %d"),operand);
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
  