/*
 *  © 2021 Neil McKechnie
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

#ifndef EXRAILMacros_H
#define EXRAILMacros_H

// remove normal code LCD & SERIAL macros (will be restored later)
#undef LCD
#undef SERIAL


// This file will include and build the EXRAIL script and associated helper tricks.
// It does this by including myAutomation.h several times, each with a set of macros to 
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

// Pass 1 Implements aliases 
#include "EXRAIL2MacroReset.h"
#undef ALIAS
#define ALIAS(name,value) const int name=value; 
#include "myAutomation.h"

// Pass 2 convert descriptions to  withrottle format emitter function
#include "EXRAIL2MacroReset.h"
#undef ROUTE
#define ROUTE(id, description) emitRouteDescription(stream,'R',id,F(description));
#undef AUTOMATION
#define AUTOMATION(id, description) emitRouteDescription(stream,'A',id,F(description));
void  RMFT2::emitWithrottleDescriptions(Print * stream) {
   (void)stream;
    #include "myAutomation.h"
}

// Pass 3... Create Text sending functions
#include "EXRAIL2MacroReset.h"
const int StringMacroTracker1=__COUNTER__;
#undef PRINT
#define PRINT(msg)    case (__COUNTER__ - StringMacroTracker1) : printMessage2(F(msg));break;
#undef LCN
#define LCN(msg)      case (__COUNTER__ - StringMacroTracker1) : StringFormatter::send(&LCN_SERIAL,F(msg));break;
#undef SERIAL
#define SERIAL(msg)   case (__COUNTER__ - StringMacroTracker1) : StringFormatter::send(&Serial,F(msg));break;
#undef SERIAL1
#define SERIAL1(msg)  case (__COUNTER__ - StringMacroTracker1) : StringFormatter::send(&Serial1,F(msg));break;
#undef SERIAL2
#define SERIAL2(msg)  case (__COUNTER__ - StringMacroTracker1) : StringFormatter::send(&Serial2,F(msg));break;
#undef SERIAL3
#define SERIAL3(msg)  case (__COUNTER__ - StringMacroTracker1) : StringFormatter::send(&Serial3,F(msg));break;
#undef LCD
#define LCD(id,msg)   case (__COUNTER__ - StringMacroTracker1) : StringFormatter::lcd(id,F(msg));break;

void  RMFT2::printMessage(uint16_t id) { 
  switch(id) {
    #include "myAutomation.h"
    default: break ; 
  }
}


// Pass 4: Turnout descriptions (optional)
#include "EXRAIL2MacroReset.h"
#undef TURNOUT
#define TURNOUT(id,addr,subaddr,description...) case id: desc=F("" description); break;
#undef PIN_TURNOUT
#define PIN_TURNOUT(id,pin,description...) case id: desc=F("" description); break;
#undef SERVO_TURNOUT
#define SERVO_TURNOUT(id,pin,activeAngle,inactiveAngle,profile,description...) case id: desc=F("" description); break;

void RMFT2::emitTurnoutDescription(Print* stream,int16_t turnoutid) {
     const FSH * desc=F("");
     switch (turnoutid) {
        #include "myAutomation.h"
     default: break;
     }
     if (GETFLASH(desc)=='\0') desc=F("%d");
     StringFormatter::send(stream,desc,turnoutid);    
}

// Pass 5: Roster names (count)
#include "EXRAIL2MacroReset.h"
#undef ROSTER
#define ROSTER(cabid,name,funcmap...) +1

const byte RMFT2::rosterNameCount=0
        #include "myAutomation.h"
     ;

// Pass 6: Roster names emitter
#include "EXRAIL2MacroReset.h"
#undef ROSTER
#define ROSTER(cabid,name,funcmap...) StringFormatter::send(stream,(FSH *)format,F(name),cabid,cabid<128?'S':'L');
void RMFT2::emitWithrottleRoster(Print * stream) {
        static const char format[] FLASH ="]\\[%S}|{%d}|{%c";
        (void)format;
        StringFormatter::send(stream,F("RL%d"), rosterNameCount);
        #include "myAutomation.h"
        stream->write('\n');        
}

// Pass 7: functions getter
#include "EXRAIL2MacroReset.h"
#undef ROSTER
#define ROSTER(cabid,name,funcmap...) case cabid: return F("" funcmap);
const FSH *  RMFT2::getRosterFunctions(int16_t cabid) {
   switch(cabid) {
      #include "myAutomation.h"
      default: return NULL;
   }        
}

// Pass 8 Signal definitions
#include "EXRAIL2MacroReset.h"
#undef SIGNAL
#define SIGNAL(redpin,amberpin,greenpin) redpin,amberpin,greenpin, 
const  FLASH  int16_t RMFT2::SignalDefinitions[] = {
    #include "myAutomation.h"
    0,0,0 };

// Last Pass : create main routes table
// Only undef the macros, not dummy them.  
#define  RMFT2_UNDEF_ONLY
#include "EXRAIL2MacroReset.h"
// Define internal helper macros.
// Everything we generate here has to be compile-time evaluated to 
// a constant.  
#define V(val) (byte)(((int16_t)(val))&0x00FF),(byte)(((int16_t)(val)>>8)&0x00FF)
// Define macros for route code creation 

#define ACTIVATE(addr,subaddr) OPCODE_DCCACTIVATE,V(addr<<3 | subaddr<<1 | 1),
#define ACTIVATEL(addr) OPCODE_DCCACTIVATE,V((addr+3)<<1 | 1),
#define AFTER(sensor_id) OPCODE_AT,V(sensor_id),OPCODE_AFTER,V(sensor_id),
#define ALIAS(name,value) 
#define AMBER(signal_id) OPCODE_AMBER,V(signal_id),
#define AT(sensor_id) OPCODE_AT,V(sensor_id),
#define ATTIMEOUT(sensor_id,timeout) OPCODE_ATTIMEOUT1,0,0,OPCODE_ATTIMEOUT2,V(sensor_id),OPCODE_PAD,V(timeout/100L),
#define AUTOMATION(id, description)  OPCODE_AUTOMATION, V(id), 
#define AUTOSTART OPCODE_AUTOSTART,0,0,
#define CALL(route) OPCODE_CALL,V(route),
#define CLOSE(id)  OPCODE_CLOSE,V(id),
#define DEACTIVATE(addr,subaddr) OPCODE_DCCACTIVATE,V(addr<<3 | subaddr<<1),
#define DEACTIVATEL(addr) OPCODE_DCCACTIVATE,V((addr+3)<<1),
#define DELAY(ms) ms<30000?OPCODE_DELAYMS:OPCODE_DELAY,V(ms/(ms<30000?1L:100L)),
#define DELAYMINS(mindelay) OPCODE_DELAYMINS,V(mindelay),
#define DELAYRANDOM(mindelay,maxdelay) DELAY(mindelay) OPCODE_RANDWAIT,V((maxdelay-mindelay)/100L),
#define DONE OPCODE_ENDTASK,0,0,
#define DRIVE(analogpin) OPCODE_DRIVE,V(analogpin),
#define ELSE OPCODE_ELSE,0,0,
#define ENDEXRAIL 
#define ENDIF  OPCODE_ENDIF,0,0,
#define ENDTASK OPCODE_ENDTASK,0,0,
#define ESTOP OPCODE_SPEED,V(1), 
#define EXRAIL 
#define FADE(pin,value,ms) OPCODE_SERVO,V(pin),OPCODE_PAD,V(value),OPCODE_PAD,V(PCA9685::ProfileType::UseDuration|PCA9685::NoPowerOff),OPCODE_PAD,V(ms/100L),
#define FOFF(func) OPCODE_FOFF,V(func),
#define FOLLOW(route) OPCODE_FOLLOW,V(route),
#define FON(func) OPCODE_FON,V(func),
#define FREE(blockid) OPCODE_FREE,V(blockid),
#define FWD(speed) OPCODE_FWD,V(speed),
#define GREEN(signal_id) OPCODE_GREEN,V(signal_id),
#define IF(sensor_id) OPCODE_IF,V(sensor_id),
#define IFCLOSED(turnout_id) OPCODE_IFCLOSED,V(turnout_id),
#define IFGTE(sensor_id,value) OPCODE_IFGTE,V(sensor_id),OPCODE_PAD,V(value),
#define IFLT(sensor_id,value) OPCODE_IFLT,V(sensor_id),OPCODE_PAD,V(value),
#define IFNOT(sensor_id) OPCODE_IFNOT,V(sensor_id),
#define IFRANDOM(percent) OPCODE_IFRANDOM,V(percent),
#define IFRESERVE(block) OPCODE_IFRESERVE,V(block),
#define IFTHROWN(turnout_id) OPCODE_IFTHROWN,V(turnout_id),
#define IFTIMEOUT OPCODE_IFTIMEOUT,0,0,
#define INVERT_DIRECTION OPCODE_INVERT_DIRECTION,0,0,
#define JOIN OPCODE_JOIN,0,0,
#define LATCH(sensor_id) OPCODE_LATCH,V(sensor_id),
#define LCD(id,msg) PRINT(msg)
#define LCN(msg) PRINT(msg)
#define ONACTIVATE(addr,subaddr) OPCODE_ONACTIVATE,V(addr<<2|subaddr),
#define ONACTIVATEL(linear) OPCODE_ONACTIVATE,V(linear+3),
#define ONCLOSE(turnout_id) OPCODE_ONCLOSE,V(turnout_id),
#define ONDEACTIVATE(addr,subaddr) OPCODE_ONDEACTIVATE,V(addr<<2|subaddr),
#define ONDEACTIVATEL(linear) OPCODE_ONDEACTIVATE,V(linear+3),
#define ONTHROW(turnout_id) OPCODE_ONTHROW,V(turnout_id),
#define PAUSE OPCODE_PAUSE,0,0,
#define PIN_TURNOUT(id,pin,description...) OPCODE_PINTURNOUT,V(id),OPCODE_PAD,V(pin), 
#define POM(cv,value) OPCODE_POM,V(cv),OPCODE_PAD,V(value),
#define POWEROFF OPCODE_POWEROFF,0,0,
#define PRINT(msg) OPCODE_PRINT,V(__COUNTER__ - StringMacroTracker2),
#define READ_LOCO OPCODE_READ_LOCO1,0,0,OPCODE_READ_LOCO2,0,0,
#define RED(signal_id) OPCODE_RED,V(signal_id),
#define RESERVE(blockid) OPCODE_RESERVE,V(blockid),
#define RESET(pin) OPCODE_RESET,V(pin),
#define RESUME OPCODE_RESUME,0,0,
#define RETURN OPCODE_RETURN,0,0,
#define REV(speed) OPCODE_REV,V(speed),
#define ROSTER(cabid,name,funcmap...)
#define ROUTE(id, description)  OPCODE_ROUTE, V(id), 
#define SENDLOCO(cab,route) OPCODE_SENDLOCO,V(cab),OPCODE_PAD,V(route),
#define SEQUENCE(id)  OPCODE_SEQUENCE, V(id), 
#define SERIAL(msg) PRINT(msg)
#define SERIAL1(msg) PRINT(msg)
#define SERIAL2(msg) PRINT(msg)
#define SERIAL3(msg) PRINT(msg)
#define SERVO(id,position,profile) OPCODE_SERVO,V(id),OPCODE_PAD,V(position),OPCODE_PAD,V(PCA9685::profile),OPCODE_PAD,V(0),
#define SERVO2(id,position,ms) OPCODE_SERVO,V(id),OPCODE_PAD,V(position),OPCODE_PAD,V(PCA9685::Instant),OPCODE_PAD,V(ms/100L),
#define SERVO_TURNOUT(id,pin,activeAngle,inactiveAngle,profile,description...) OPCODE_SERVOTURNOUT,V(id),OPCODE_PAD,V(pin),OPCODE_PAD,V(activeAngle),OPCODE_PAD,V(inactiveAngle),OPCODE_PAD,V(PCA9685::ProfileType::profile),
#define SET(pin) OPCODE_SET,V(pin),
#define SETLOCO(loco) OPCODE_SETLOCO,V(loco),
#define SIGNAL(redpin,amberpin,greenpin) 
#define SPEED(speed) OPCODE_SPEED,V(speed),
#define START(route) OPCODE_START,V(route),
#define STOP OPCODE_SPEED,V(0), 
#define THROW(id)  OPCODE_THROW,V(id),
#define TURNOUT(id,addr,subaddr,description...) OPCODE_TURNOUT,V(id),OPCODE_PAD,V(addr),OPCODE_PAD,V(subaddr),
#define UNJOIN OPCODE_UNJOIN,0,0,
#define UNLATCH(sensor_id) OPCODE_UNLATCH,V(sensor_id),
#define WAITFOR(pin) OPCODE_WAITFOR,V(pin),
#define XFOFF(cab,func) OPCODE_XFOFF,V(cab),OPCODE_PAD,V(func),
#define XFON(cab,func) OPCODE_XFON,V(cab),OPCODE_PAD,V(func),

// Build RouteCode
const int StringMacroTracker2=__COUNTER__;
const  FLASH  byte RMFT2::RouteCode[] = {
    #include "myAutomation.h"
    OPCODE_ENDTASK,0,0,OPCODE_ENDEXRAIL,0,0 };

// Restore normal code LCD & SERIAL  macro
#undef LCD
#define LCD   StringFormatter::lcd
#undef SERIAL
#define SERIAL  0x0
#endif
