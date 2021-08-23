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
#ifndef RMFTMacros_H
#define RMFTMacros_H

// remove normal code LCD macro (will be restored later)
#undef LCD


// This file will include and build the EXRAIL script and associated helper tricks.
// It does this by incliding myAutomation.h several times, each with a set of macros to 
// extract the relevant parts.

// The entire automation script is contained within a byte array RMFT2::RouteCode[]
// made up of opcode and parameter pairs.
// ech opcode is a 1 byte operation plus 2 byte operand. 
// The array is normally built using the macros below as this makes it easier 
// to manage the cases where:
// - padding must be applied to ensure the correct alignment of the next instruction
// - large parameters must be split up
// - multiple parameters aligned correctly
// - a single macro requires multiple operations

// Descriptive texts for routes and animations are created in a sepaerate function which
// can be called to emit a list of routes/automatuions in a form suitable for Withrottle. 
 
// PRINT(msg) and LCD(row,msg) is implemented in a separate pass to create 
// a getMessageText(id) function.  

// CAUTION: The macros below are multiple passed over myAutomation.h

// Pass 1 Implements aliases and 
// converts descriptions to  withrottle format emitter function
// Most macros are simply ignored in this pass.


#define ALIAS(name,value) const int name=value; 
#define EXRAIL  void  RMFT2::emitWithrottleDescriptions(Print * stream) {
#define ROUTE(id, description) emitRouteDescription(stream,'R',id,F(description));
#define AUTOMATION(id, description) emitRouteDescription(stream,'A',id,F(description));
#define ENDEXRAIL  }
 
#define AFTER(sensor_id)
#define AMBER(signal_id)
#define AT(sensor_id)
#define CALL(route) 
#define CLOSE(id) 
#define DELAY(mindelay)
#define DELAYMINS(mindelay)
#define DELAYRANDOM(mindelay,maxdelay) 
#define DONE
#define ENDIF  
#define ENDTASK
#define ESTOP 
#define FOFF(func)
#define FOLLOW(route) 
#define FON(func) 
#define FREE(blockid) 
#define FWD(speed) 
#define GREEN(signal_id)
#define IF(sensor_id) 
#define IFNOT(sensor_id)
#define IFRANDOM(percent) 
#define IFRESERVE(block) 
#define INVERT_DIRECTION 
#define JOIN 
#define LATCH(sensor_id) 
#define LCD(row,msg) 
#define ONCLOSE(turnout_id)
#define ONTHROW(turnout_id) 
#define PAUSE
#define PRINT(msg) 
#define POM(cv,value)
#define READ_LOCO 
#define RED(signal_id) 
#define RESERVE(blockid) 
#define RESET(sensor_id) 
#define RESUME 
#define RETURN 
#define REV(speed) 
#define START(route) 
#define SENDLOCO(cab,route) 
#define SERVO(id,position,profile) 
#define SETLOCO(loco) 
#define SET(sensor_id) 
#define SEQUENCE(id) 
#define SPEED(speed) 
#define STOP 
#undef SIGNAL
#define SIGNAL(redpin,amberpin,greenpin) 
#define SERVO_TURNOUT(id,pin,activeAngle,inactiveAngle,profile) 
#define PIN_TURNOUT(id,pin) 
#define THROW(id)  
#define TURNOUT(id,addr,subaddr) 
#define UNJOIN 
#define UNLATCH(sensor_id) 

#include "myAutomation.h"

// setup for pass 2... Create getMessageText function
#undef ALIAS
#undef ROUTE
#undef AUTOMATION  
#define ROUTE(id, description)
#define AUTOMATION(id, description)  

#undef EXRAIL 
#undef PRINT
#undef ENDEXRAIL  
#undef LCD
const int StringMacroTracker1=__COUNTER__;
#define ALIAS(name,value) 
#define EXRAIL void  RMFT2::printMessage(uint16_t id) { switch(id) {
#define ENDEXRAIL  default: DIAG(F("printMessage error %d %d"),id,StringMacroTracker1); return ; }}
#define PRINT(msg)  case (__COUNTER__ - StringMacroTracker1) : printMessage2(F(msg));break;
#define LCD(id,msg)  case (__COUNTER__ - StringMacroTracker1) : StringFormatter::lcd(id,F(msg));break;
#include "myAutomation.h"

// Setup for Pass 3: create main routes table 
#undef AFTER
#undef AMBER
#undef AT
#undef AUTOMATION 
#undef CALL
#undef CLOSE
#undef DELAY
#undef DELAYMINS
#undef DELAYRANDOM
#undef DONE
#undef ENDIF
#undef ENDEXRAIL
#undef ENDTASK
#undef ESTOP
#undef EXRAIL
#undef FOFF
#undef FOLLOW
#undef FON
#undef FREE
#undef FWD
#undef GREEN
#undef IF
#undef IFNOT
#undef IFRANDOM
#undef IFRESERVE
#undef INVERT_DIRECTION
#undef JOIN
#undef LATCH
#undef LCD
#undef ONCLOSE
#undef ONTHROW
#undef PAUSE
#undef POM
#undef PRINT
#undef READ_LOCO
#undef RED
#undef RESERVE
#undef RESET
#undef RESUME
#undef RETURN
#undef REV
#undef ROUTE 
#undef START
#undef SEQUENCE 
#undef SERVO
#undef SENDLOCO
#undef SETLOCO
#undef SET
#undef SPEED
#undef STOP
#undef SIGNAL
#undef SERVO_TURNOUT
#undef PIN_TURNOUT
#undef THROW
#undef TURNOUT
#undef UNJOIN
#undef UNLATCH

// Define macros for route code creation 
#define V(val) ((int16_t)(val))&0x00FF,((int16_t)(val)>>8)&0x00FF
#define NOP 0,0

#define ALIAS(name,value) 
#define EXRAIL const  FLASH  byte RMFT2::RouteCode[] = {
#define AUTOMATION(id, description)  OPCODE_AUTOMATION, V(id), 
#define ROUTE(id, description)  OPCODE_ROUTE, V(id), 
#define SEQUENCE(id)  OPCODE_SEQUENCE, V(id), 
#define ENDTASK OPCODE_ENDTASK,NOP,
#define DONE OPCODE_ENDTASK,NOP,
#define ENDEXRAIL OPCODE_ENDTASK,NOP,OPCODE_ENDEXRAIL,NOP };
 
#define AFTER(sensor_id) OPCODE_AT,V(sensor_id),OPCODE_AFTER,V(sensor_id),
#define AMBER(signal_id) OPCODE_AMBER,V(signal_id),
#define AT(sensor_id) OPCODE_AT,V(sensor_id),
#define CALL(route) OPCODE_CALL,V(route),
#define CLOSE(id)  OPCODE_CLOSE,V(id),
#define DELAY(ms) OPCODE_DELAY,V(ms/100L),
#define DELAYMINS(mindelay) OPCODE_DELAYMINS,V(mindelay),
#define DELAYRANDOM(mindelay,maxdelay) OPCODE_DELAY,V(mindelay/100L),OPCODE_RANDWAIT,V((maxdelay-mindelay)/100L),
#define ENDIF  OPCODE_ENDIF,NOP,
#define ESTOP OPCODE_SPEED,V(1), 
#define FOFF(func) OPCODE_FOFF,V(func),
#define FOLLOW(route) OPCODE_FOLLOW,V(route),
#define FON(func) OPCODE_FON,V(func),
#define FREE(blockid) OPCODE_FREE,V(blockid),
#define FWD(speed) OPCODE_FWD,V(speed),
#define GREEN(signal_id) OPCODE_GREEN,V(signal_id),
#define IF(sensor_id) OPCODE_IF,V(sensor_id),
#define IFNOT(sensor_id) OPCODE_IFNOT,V(sensor_id),
#define IFRANDOM(percent) OPCODE_IFRANDOM,V(percent),
#define IFRESERVE(block) OPCODE_IFRESERVE,V(block),
#define INVERT_DIRECTION OPCODE_INVERT_DIRECTION,NOP,
#define JOIN OPCODE_JOIN,NOP,
#define LATCH(sensor_id) OPCODE_LATCH,V(sensor_id),
#define LCD(id,msg) OPCODE_PRINT,V(__COUNTER__ - StringMacroTracker2),
#define ONCLOSE(turnout_id) OPCODE_ONCLOSE,V(turnout_id),
#define ONTHROW(turnout_id) OPCODE_ONTHROW,V(turnout_id),
#define PAUSE OPCODE_PAUSE,NOP,
#define POM(cv,value) OPCODE_POM,V(cv),OPCODE_PAD,V(value),
#define PRINT(msg) OPCODE_PRINT,V(__COUNTER__ - StringMacroTracker2),
#define READ_LOCO OPCODE_READ_LOCO1,NOP,OPCODE_READ_LOCO2,NOP,
#define RED(signal_id) OPCODE_RED,V(signal_id),
#define RESERVE(blockid) OPCODE_RESERVE,V(blockid),
#define RESET(sensor_id) OPCODE_RESET,V(sensor_id),
#define RESUME OPCODE_RESUME,NOP,
#define RETURN OPCODE_RETURN,NOP,
#define REV(speed) OPCODE_REV,V(speed),
#define SENDLOCO(cab,route) OPCODE_SENDLOCO,V(cab),OPCODE_PAD,V(route),
#define START(route) OPCODE_START,V(route),
#define SERVO(id,position,profile) OPCODE_SERVO,V(id),OPCODE_PAD,V(position),OPCODE_PAD,V(PCA9685::ProfileType::profile),
#define SETLOCO(loco) OPCODE_SETLOCO,V(loco),
#define SET(sensor_id) OPCODE_SET,V(sensor_id),
#define SPEED(speed) OPCODE_SPEED,V(speed),
#define STOP OPCODE_SPEED,V(0), 
#define SIGNAL(redpin,amberpin,greenpin) OPCODE_SIGNAL,V(redpin),OPCODE_PAD,V(amberpin),OPCODE_PAD,V(greenpin), 
#define SERVO_TURNOUT(id,pin,activeAngle,inactiveAngle,profile) OPCODE_SERVOTURNOUT,V(id),OPCODE_PAD,V(pin),OPCODE_PAD,V(activeAngle),OPCODE_PAD,V(inactiveAngle),OPCODE_PAD,V(PCA9685::ProfileType::profile),
#define PIN_TURNOUT(id,pin) OPCODE_PINTURNOUT,V(id),OPCODE_PAD,V(pin), 
#define THROW(id)  OPCODE_THROW,V(id),
#define TURNOUT(id,addr,subaddr) OPCODE_TURNOUT,V(id),OPCODE_PAD,V(addr),OPCODE_PAD,V(subaddr),
#define UNJOIN OPCODE_UNJOIN,NOP,
#define UNLATCH(sensor_id) OPCODE_UNLATCH,V(sensor_id),

// PASS2 Build RouteCode
const int StringMacroTracker2=__COUNTER__;
#include "myAutomation.h"

// Restore normal code LCD macro
#undef LCD
#define LCD   StringFormatter::lcd

#endif
