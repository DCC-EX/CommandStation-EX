/*
 *  © 2021 Neil McKechnie
 *  © 2020-2022 Chris Harlow
 *  © 2022 Colin Murdoch
 *  © 2023 Harald Barth
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
 
// PRINT(msg), LCD(row,msg) and SCREEN(display,row,msg) are implemented in a separate pass to create 
// a getMessageText(id) function.  

// CAUTION: The macros below are multiple passed over myAutomation.h


// helper macro for turnout descriptions, creates NULL for missing description
#define O_DESC(id, desc) case id: return ("" desc)[0]?F("" desc):NULL;
// helper macro for turnout description as HIDDEN 
#define HIDDEN "\x01"

// helper macro to strip leading zeros off time inputs
// (10#mins)%100)
#define STRIP_ZERO(value) 10##value%100

// Pass 1 Implements aliases 
#include "EXRAIL2MacroReset.h"
#undef ALIAS
#define ALIAS(name,value...) const int name= 1##value##0 ==10 ? -__COUNTER__  : value##0/10; 
#include "myAutomation.h"

// Pass 1h Implements HAL macro by creating exrailHalSetup function 
#include "EXRAIL2MacroReset.h"
#undef HAL
#define HAL(haltype,params...)  haltype::create(params);
void exrailHalSetup() {
   #include "myAutomation.h"
}

// Pass 2 create throttle route list 
#include "EXRAIL2MacroReset.h"
#undef ROUTE
#define ROUTE(id, description) id,
const int16_t HIGHFLASH RMFT2::routeIdList[]= {
    #include "myAutomation.h"
    INT16_MAX};
// Pass 2a create throttle automation list 
#include "EXRAIL2MacroReset.h"
#undef AUTOMATION
#define AUTOMATION(id, description) id,
const int16_t HIGHFLASH RMFT2::automationIdList[]= {
    #include "myAutomation.h"
    INT16_MAX};

// Pass 3 Create route descriptions:
#undef ROUTE
#define ROUTE(id, description) case id: return F(description);
#undef AUTOMATION
#define AUTOMATION(id, description) case id: return F(description);
const FSH * RMFT2::getRouteDescription(int16_t id) {
   switch(id) {
    #include "myAutomation.h"
    default: break;
   }
   return F("");
}

// Pass 4... Create Text sending functions
#include "EXRAIL2MacroReset.h"
const int StringMacroTracker1=__COUNTER__;
#define THRUNGE(msg,mode) \
     case (__COUNTER__ - StringMacroTracker1) : {\
         static const char HIGHFLASH thrunge[]=msg;\
         strfar=(uint32_t)GETFARPTR(thrunge);\
         tmode=mode;\
         break;\
      } 
#undef BROADCAST
#define BROADCAST(msg) THRUNGE(msg,thrunge_broadcast)
#undef PARSE
#define PARSE(msg) THRUNGE(msg,thrunge_parse)
#undef PRINT
#define PRINT(msg) THRUNGE(msg,thrunge_print)
#undef LCN
#define LCN(msg)   THRUNGE(msg,thrunge_lcn)
#undef SERIAL
#define SERIAL(msg)   THRUNGE(msg,thrunge_serial)
#undef SERIAL1
#define SERIAL1(msg)  THRUNGE(msg,thrunge_serial1)
#undef SERIAL2
#define SERIAL2(msg)  THRUNGE(msg,thrunge_serial2)
#undef SERIAL3
#define SERIAL3(msg)  THRUNGE(msg,thrunge_serial3)
#undef SERIAL4
#define SERIAL4(msg)  THRUNGE(msg,thrunge_serial4)
#undef SERIAL5
#define SERIAL5(msg)  THRUNGE(msg,thrunge_serial5)
#undef SERIAL6
#define SERIAL6(msg)  THRUNGE(msg,thrunge_serial6)
#undef LCD
#define LCD(id,msg)  \
     case (__COUNTER__ - StringMacroTracker1) : {\
         static const char HIGHFLASH thrunge[]=msg;\
         strfar=(uint32_t)GETFARPTR(thrunge);\
         tmode=thrunge_lcd; \
         lcdid=id;\
         break;\
      }
#undef SCREEN
#define SCREEN(display,id,msg)  \
     case (__COUNTER__ - StringMacroTracker1) : {\
         static const char HIGHFLASH thrunge[]=msg;\
         strfar=(uint32_t)GETFARPTR(thrunge);\
         tmode=(thrunger)(thrunge_lcd+display); \
         lcdid=id;\
         break;\
      } 
#undef WITHROTTLE
#define WITHROTTLE(msg) THRUNGE(msg,thrunge_withrottle)

void  RMFT2::printMessage(uint16_t id) { 
  thrunger tmode;
  uint32_t strfar=0;
  byte lcdid=0; 
  switch(id) {
    #include "myAutomation.h"
    default: break ; 
  }
  if (strfar) thrungeString(strfar,tmode,lcdid);
}


// Pass 5: Turnout descriptions (optional)
#include "EXRAIL2MacroReset.h"
#undef TURNOUT
#define TURNOUT(id,addr,subaddr,description...) O_DESC(id,description)
#undef PIN_TURNOUT
#define PIN_TURNOUT(id,pin,description...) O_DESC(id,description)
#undef SERVO_TURNOUT
#define SERVO_TURNOUT(id,pin,activeAngle,inactiveAngle,profile,description...) O_DESC(id,description)
#undef VIRTUAL_TURNOUT
#define VIRTUAL_TURNOUT(id,description...) O_DESC(id,description)

const FSH * RMFT2::getTurnoutDescription(int16_t turnoutid) {
     switch (turnoutid) {
        #include "myAutomation.h"
     default:break;
     }
     return NULL;
}

// Pass 6: Roster IDs (count)
#include "EXRAIL2MacroReset.h"
#undef ROSTER
#define ROSTER(cabid,name,funcmap...) +(cabid <= 0 ? 0 : 1)
const byte RMFT2::rosterNameCount=0
   #include "myAutomation.h"
   ;
   
// Pass 6: Roster IDs 
#include "EXRAIL2MacroReset.h"
#undef ROSTER
#define ROSTER(cabid,name,funcmap...) cabid,
const int16_t HIGHFLASH  RMFT2::rosterIdList[]={
   #include "myAutomation.h"
   INT16_MAX};

// Pass 7: Roster names getter
#include "EXRAIL2MacroReset.h"
#undef ROSTER
#define ROSTER(cabid,name,funcmap...) case cabid: return F(name);
const FSH * RMFT2::getRosterName(int16_t id) {
   switch(id) {
      #include "myAutomation.h"
   default: break;
   }
   return F("");   
} 

// Pass to get roster functions 
#undef ROSTER
#define ROSTER(cabid,name,funcmap...) case cabid: return F("" funcmap);
const FSH * RMFT2::getRosterFunctions(int16_t id) {
   switch(id) {
      #include "myAutomation.h"
   default: break; 
   }   
   return NULL;
} 

// Pass 8 Signal definitions
#include "EXRAIL2MacroReset.h"
#undef SIGNAL
#define SIGNAL(redpin,amberpin,greenpin) redpin,redpin,amberpin,greenpin, 
#undef SIGNALH
#define SIGNALH(redpin,amberpin,greenpin) redpin | RMFT2::ACTIVE_HIGH_SIGNAL_FLAG,redpin,amberpin,greenpin, 
#undef SERVO_SIGNAL
#define SERVO_SIGNAL(vpin,redval,amberval,greenval) vpin | RMFT2::SERVO_SIGNAL_FLAG,redval,amberval,greenval, 
#undef DCC_SIGNAL
#define DCC_SIGNAL(id,addr,subaddr) id | RMFT2::DCC_SIGNAL_FLAG,addr,subaddr,0,
#undef VIRTUAL_SIGNAL
#define VIRTUAL_SIGNAL(id) id,0,0,0,

const  HIGHFLASH  int16_t RMFT2::SignalDefinitions[] = {
    #include "myAutomation.h"
    0,0,0,0 };

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
#define ALIAS(name,value...) 
#define AMBER(signal_id) OPCODE_AMBER,V(signal_id),
#define ANOUT(vpin,value,param1,param2) OPCODE_SERVO,V(vpin),OPCODE_PAD,V(value),OPCODE_PAD,V(param1),OPCODE_PAD,V(param2),
#define AT(sensor_id) OPCODE_AT,V(sensor_id),
#define ATGTE(sensor_id,value) OPCODE_ATGTE,V(sensor_id),OPCODE_PAD,V(value),  
#define ATLT(sensor_id,value) OPCODE_ATLT,V(sensor_id),OPCODE_PAD,V(value),  
#define ATTIMEOUT(sensor_id,timeout) OPCODE_ATTIMEOUT1,0,0,OPCODE_ATTIMEOUT2,V(sensor_id),OPCODE_PAD,V(timeout/100L),
#define AUTOMATION(id, description)  OPCODE_AUTOMATION, V(id), 
#define AUTOSTART OPCODE_AUTOSTART,0,0,
#define BROADCAST(msg) PRINT(msg)
#define CALL(route) OPCODE_CALL,V(route),
#define CLOSE(id)  OPCODE_CLOSE,V(id),
#define DEACTIVATE(addr,subaddr) OPCODE_DCCACTIVATE,V(addr<<3 | subaddr<<1),
#define DEACTIVATEL(addr) OPCODE_DCCACTIVATE,V((addr+3)<<1),
#define DELAY(ms) ms<30000?OPCODE_DELAYMS:OPCODE_DELAY,V(ms/(ms<30000?1L:100L)),
#define DELAYMINS(mindelay) OPCODE_DELAYMINS,V(mindelay),
#define DELAYRANDOM(mindelay,maxdelay) DELAY(mindelay) OPCODE_RANDWAIT,V((maxdelay-mindelay)/100L),
#define DCC_SIGNAL(id,add,subaddr)
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
#define FORGET OPCODE_FORGET,0,0,
#define FREE(blockid) OPCODE_FREE,V(blockid),
#define FWD(speed) OPCODE_FWD,V(speed),
#define GREEN(signal_id) OPCODE_GREEN,V(signal_id),
#define HAL(haltype,params...)
#define IF(sensor_id) OPCODE_IF,V(sensor_id),
#define IFAMBER(signal_id) OPCODE_IFAMBER,V(signal_id),
#define IFCLOSED(turnout_id) OPCODE_IFCLOSED,V(turnout_id),
#define IFGREEN(signal_id) OPCODE_IFGREEN,V(signal_id),
#define IFGTE(sensor_id,value) OPCODE_IFGTE,V(sensor_id),OPCODE_PAD,V(value),
#define IFLOCO(loco_id) OPCODE_IFLOCO,V(loco_id),
#define IFLT(sensor_id,value) OPCODE_IFLT,V(sensor_id),OPCODE_PAD,V(value),
#define IFNOT(sensor_id) OPCODE_IFNOT,V(sensor_id),
#define IFRANDOM(percent) OPCODE_IFRANDOM,V(percent),
#define IFRED(signal_id) OPCODE_IFRED,V(signal_id),
#define IFRESERVE(block) OPCODE_IFRESERVE,V(block),
#define IFTHROWN(turnout_id) OPCODE_IFTHROWN,V(turnout_id),
#define IFTIMEOUT OPCODE_IFTIMEOUT,0,0,
#define IFRE(sensor_id,value) OPCODE_IFRE,V(sensor_id),OPCODE_PAD,V(value),
#define INVERT_DIRECTION OPCODE_INVERT_DIRECTION,0,0,
#define JOIN OPCODE_JOIN,0,0,
#define KILLALL OPCODE_KILLALL,0,0,
#define LATCH(sensor_id) OPCODE_LATCH,V(sensor_id),
#define LCD(id,msg) PRINT(msg)
#define SCREEN(display,id,msg) PRINT(msg)
#define LCN(msg) PRINT(msg)
#define MOVETT(id,steps,activity) OPCODE_SERVO,V(id),OPCODE_PAD,V(steps),OPCODE_PAD,V(EXTurntable::activity),OPCODE_PAD,V(0),
#define ONACTIVATE(addr,subaddr) OPCODE_ONACTIVATE,V(addr<<2|subaddr),
#define ONACTIVATEL(linear) OPCODE_ONACTIVATE,V(linear+3),
#define ONAMBER(signal_id) OPCODE_ONAMBER,V(signal_id),
#define ONCLOSE(turnout_id) OPCODE_ONCLOSE,V(turnout_id),
#define ONTIME(value) OPCODE_ONTIME,V(value),  
#define ONCLOCKTIME(hours,mins) OPCODE_ONTIME,V((STRIP_ZERO(hours)*60)+STRIP_ZERO(mins)),
#define ONCLOCKMINS(mins) ONCLOCKTIME(25,mins)
#define ONDEACTIVATE(addr,subaddr) OPCODE_ONDEACTIVATE,V(addr<<2|subaddr),
#define ONDEACTIVATEL(linear) OPCODE_ONDEACTIVATE,V(linear+3),
#define ONGREEN(signal_id) OPCODE_ONGREEN,V(signal_id),
#define ONRED(signal_id) OPCODE_ONRED,V(signal_id),
#define ONTHROW(turnout_id) OPCODE_ONTHROW,V(turnout_id),
#define ONCHANGE(sensor_id) OPCODE_ONCHANGE,V(sensor_id),
#define PAUSE OPCODE_PAUSE,0,0,
#define PIN_TURNOUT(id,pin,description...) OPCODE_PINTURNOUT,V(id),OPCODE_PAD,V(pin),
#ifndef DISABLE_PROG
#define POM(cv,value) OPCODE_POM,V(cv),OPCODE_PAD,V(value),
#endif
#define POWEROFF OPCODE_POWEROFF,0,0,
#define POWERON OPCODE_POWERON,0,0,
#define PRINT(msg) OPCODE_PRINT,V(__COUNTER__ - StringMacroTracker2),
#define PARSE(msg) PRINT(msg)
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
#define SERIAL4(msg) PRINT(msg)
#define SERIAL5(msg) PRINT(msg)
#define SERIAL6(msg) PRINT(msg)
#define SERVO(id,position,profile) OPCODE_SERVO,V(id),OPCODE_PAD,V(position),OPCODE_PAD,V(PCA9685::profile),OPCODE_PAD,V(0),
#define SERVO2(id,position,ms) OPCODE_SERVO,V(id),OPCODE_PAD,V(position),OPCODE_PAD,V(PCA9685::Instant),OPCODE_PAD,V(ms/100L),
#define SERVO_SIGNAL(vpin,redpos,amberpos,greenpos)
#define SERVO_TURNOUT(id,pin,activeAngle,inactiveAngle,profile,description...) OPCODE_SERVOTURNOUT,V(id),OPCODE_PAD,V(pin),OPCODE_PAD,V(activeAngle),OPCODE_PAD,V(inactiveAngle),OPCODE_PAD,V(PCA9685::ProfileType::profile),
#define SET(pin) OPCODE_SET,V(pin),
#define SET_TRACK(track,mode)  OPCODE_SET_TRACK,V(TRACK_MODE_##mode  <<8 | TRACK_NUMBER_##track),
#define SETLOCO(loco) OPCODE_SETLOCO,V(loco),
#define SIGNAL(redpin,amberpin,greenpin) 
#define SIGNALH(redpin,amberpin,greenpin) 
#define SPEED(speed) OPCODE_SPEED,V(speed),
#define START(route) OPCODE_START,V(route),
#define STOP OPCODE_SPEED,V(0), 
#define THROW(id)  OPCODE_THROW,V(id),
#define TURNOUT(id,addr,subaddr,description...) OPCODE_TURNOUT,V(id),OPCODE_PAD,V(addr),OPCODE_PAD,V(subaddr),
#define TURNOUTL(id,addr,description...) TURNOUT(id,(addr-1)/4+1,(addr-1)%4, description)
#define UNJOIN OPCODE_UNJOIN,0,0,
#define UNLATCH(sensor_id) OPCODE_UNLATCH,V(sensor_id),
#define VIRTUAL_SIGNAL(id) 
#define VIRTUAL_TURNOUT(id,description...) OPCODE_PINTURNOUT,V(id),OPCODE_PAD,V(0), 
#define WITHROTTLE(msg) PRINT(msg)
#define WAITFOR(pin) OPCODE_WAITFOR,V(pin),
#define XFOFF(cab,func) OPCODE_XFOFF,V(cab),OPCODE_PAD,V(func),
#define XFON(cab,func) OPCODE_XFON,V(cab),OPCODE_PAD,V(func),

// Build RouteCode
const int StringMacroTracker2=__COUNTER__;
const  HIGHFLASH  byte RMFT2::RouteCode[] = {
    #include "myAutomation.h"
    OPCODE_ENDTASK,0,0,OPCODE_ENDEXRAIL,0,0 };

// Restore normal code LCD & SERIAL  macro
#undef LCD
#define LCD   StringFormatter::lcd
#undef SCREEN
#define SCREEN  StringFormatter::lcd2
#undef SERIAL
#define SERIAL  0x0
#endif
