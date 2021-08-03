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

// The entire automation script is contained within a byte array RMFT2::RouteCode[]
// made up of opcode and parameter pairs.
// ech opcode is a 1 byte operation plus 2 byte operand. 
// The array is normally built using the macros below as this makes it easier 
// to manage the cases where:
// - padding must be applied to ensure the correct alignment of the next instruction
// - large parameters must be split up
// - multiple parameters aligned correctly
// - a single macro requires multiple operations

// Descriptive texts for routes and animations are created in a sepaerate array RMFT2::RouteDescription[]
// but since the C preprocessor is such a wimp, we have to pass over the myAutomation.h 2 times with
// different macros. 
 

#define V(val) ((int16_t)(val))&0x00FF,((int16_t)(val)>>8)&0x00FF
#define NOP 0,0

// CAUTION: The macros below are triple passed over myAutomation.h
// Adding a macro here must have equivalent macros or no-ops  in pass 2 and 3
#define ALIAS(name,value) const int name=value;
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
#define DELAY(ms) OPCODE_DELAY,V(ms/100),
#define DELAYMINS(mindelay) OPCODE_DELAYMINS,V(mindelay),
#define DELAYRANDOM(mindelay,maxdelay) OPCODE_DELAY,V(mindelay/100),OPCODE_RANDWAIT,V((maxdelay-mindelay)/100),
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
#define ONCLOSE(turnout_id) OPCODE_ONCLOSE,V(turnout_id),
#define ONTHROW(turnout_id) OPCODE_ONTHROW,V(turnout_id),
#define PAUSE OPCODE_PAUSE,NOP,
#define POM(cv,value) OPCODE_POM,V(cv),OPCODE_PAD,V(value),
#define READ_LOCO OPCODE_READ_LOCO1,NOP,OPCODE_READ_LOCO2,NOP,
#define RED(signal_id) OPCODE_RED,V(signal_id),
#define RESERVE(blockid) OPCODE_RESERVE,V(blockid),
#define RESET(sensor_id) OPCODE_RESET,V(sensor_id),
#define RESUME OPCODE_RESUME,NOP,
#define RETURN OPCODE_RETURN,NOP,
#define REV(speed) OPCODE_REV,V(speed),
#define START(route) OPCODE_START,V(route),
#define SERVO(id,position,profile) OPCODE_SERVO,V(id),OPCODE_PAD,V(position),OPCODE_PAD,V(profile),
#define SETLOCO(loco) OPCODE_SETLOCO,V(loco),
#define SET(sensor_id) OPCODE_SET,V(sensor_id),
#define SPEED(speed) OPCODE_SPEED,V(speed),
#define STOP OPCODE_SPEED,V(0), 
#undef SIGNAL
#define SIGNAL(redpin,amberpin,greenpin) OPCODE_SIGNAL,V(redpin),OPCODE_PAD,V(amberpin),OPCODE_PAD,V(greenpin), 
#define SERVO_TURNOUT(pin,activeAngle,inactiveAngle) OPCODE_SERVOTURNOUT,V(pin),OPCODE_PAD,V(actibeAngle),OPCODE
#define PIN_TURNOUT(pin) OPCODE_PINTURNOUT,V(pin), 
#define THROW(id)  OPCODE_THROW,V(id),
#define TURNOUT(id,addr,subaddr) OPCODE_TURNOUT,V(id),OPCODE_PAD,V(addr),OPCODE_PAD,V(subaddr),
#define UNJOIN OPCODE_UNJOIN,NOP,
#define UNLATCH(sensor_id) OPCODE_UNLATCH,V(sensor_id),

// PASS1 Build RouteCode
#include "myAutomation.h"

#undef ALIAS
#undef EXRAIL
#undef AUTOMATION 
#undef ROUTE 
#undef SEQUENCE 
#undef ENDTASK
#undef DONE
#undef ENDEXRAIL
 
#undef AFTER
#undef AMBER
#undef AT
#undef CALL
#undef CLOSE
#undef DELAY
#undef DELAYMINS
#undef DELAYRANDOM
#undef ENDIF
#undef ESTOP
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
#undef ONCLOSE
#undef ONTHROW
#undef PAUSE
#undef POM
#undef READ_LOCO
#undef RED
#undef RESERVE
#undef RESET
#undef RESUME
#undef RETURN
#undef REV
#undef START
#undef SERVO
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
//==================

// Pass2 Macros convert descriptions to a flash string constant in withrottle format.
// Most macros are simply ignored in this pass.
#define ALIAS(name,value)  
#define EXRAIL  const FLASH char  RMFT2::RouteDescription[]=
#define AUTOMATION(id, description)  "]\\[A" #id "}|{" description "}|{4"
#define ROUTE(id, description)    "]\\[R" #id "}|{" description "}|{2"
#define SEQUENCE(id) 
#define ENDTASK
#define DONE
#define ENDEXRAIL  "";
 
#define AFTER(sensor_id)
#define AMBER(signal_id)
#define AT(sensor_id)
#define CALL(route) 
#define CLOSE(id) 
#define DELAY(mindelay)
#define DELAYMINS(mindelay)
#define DELAYRANDOM(mindelay,maxdelay) 
#define ENDIF  
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
#define ONCLOSE(turnout_id)
#define ONTHROW(turnout_id) 
#define PAUSE 
#define POM(cv,value)
#define READ_LOCO 
#define RED(signal_id) 
#define RESERVE(blockid) 
#define RESET(sensor_id) 
#define RESUME 
#define RETURN 
#define REV(speed) 
#define START(route) 
#define SERVO(id,position,profile) 
#define SETLOCO(loco) 
#define SET(sensor_id) 
#define SPEED(speed) 
#define STOP 
#define SIGNAL(redpin,amberpin,greenpin) 
#define SERVO_TURNOUT(pin,activeAngle,inactiveAngle) 
#define PIN_TURNOUT(pin) 
#define THROW(id)  
#define TURNOUT(id,addr,subaddr) 
#define UNJOIN 
#define UNLATCH(sensor_id) 

#include "myAutomation.h"
 

#endif
