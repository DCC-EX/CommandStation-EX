/*
 *  © 2021 Neil McKechnie
 *  © 2020-2022 Chris Harlow
 *  © 2022-2023 Colin Murdoch
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
// helper macro for turntable descriptions, creates NULL for missing description
#define T_DESC(tid,pid,desc) if(turntableId==tid && positionId==pid) return ("" desc)[0]?F("" desc):NULL;
// helper macro for turnout description as HIDDEN 
#define HIDDEN "\x01"

// PLAYSOUND is alias of ANOUT to make the user experience of a Conductor beter for
// playing sounds with IO_I2CDFPlayer
#define PLAYSOUND ANOUT

// helper macro to strip leading zeros off time inputs
// (10#mins)%100)
#define STRIP_ZERO(value) 10##value%100

// These constants help EXRAIL macros convert Track Power e.g. SET_POWER(A ON|OFF).
//const byte TRACK_POWER_0=0, TRACK_POWER_OFF=0;    
//const byte TRACK_POWER_1=1, TRACK_POWER_ON=1;   


// Pass 1 Implements aliases 
#include "EXRAIL2MacroReset.h"
#undef ALIAS
#define ALIAS(name,value...) const int name= 1##value##0 ==10 ? -__COUNTER__  : value##0/10; 
#include "myAutomation.h"

// Pass 1d Detect sequence duplicates.
// This pass generates no runtime data or code 
#include "EXRAIL2MacroReset.h"
#undef AUTOMATION
#define AUTOMATION(id, description) id,
#undef ROUTE
#define ROUTE(id, description) id,
#undef SEQUENCE
#define SEQUENCE(id) id,
constexpr int16_t compileTimeSequenceList[]={
   #include "myAutomation.h"
   0
   };
constexpr int16_t stuffSize=sizeof(compileTimeSequenceList)/sizeof(int16_t) - 1;


// Compile time function to check for sequence nos.
constexpr bool hasseq(const int16_t value, const uint16_t pos=0 ) {
    return pos>=stuffSize? false :
          compileTimeSequenceList[pos]==value 
       || hasseq(value,pos+1); 
}

// Compile time function to check for duplicate sequence nos.
constexpr bool hasdup(const int16_t value, const uint16_t pos ) {
    return pos>=stuffSize? false :
          compileTimeSequenceList[pos]==value 
       || hasseq(value,pos+1) 
       || hasdup(compileTimeSequenceList[pos],pos+1);
}


static_assert(!hasdup(compileTimeSequenceList[0],1),"Duplicate SEQUENCE/ROUTE/AUTOMATION detected");

//pass 1s static asserts to
//  - check call and follows etc for existing sequence numbers
//  - check range on LATCH/UNLATCH
// This pass generates no runtime data or code 
#include "EXRAIL2MacroReset.h"
#undef ASPECT
#define ASPECT(address,value) static_assert(address <=2044, "invalid Address"); \
                              static_assert(address>=-3, "Invalid value");
#undef CALL
#define CALL(id) static_assert(hasseq(id),"Sequence not found");
#undef FOLLOW
#define FOLLOW(id)  static_assert(hasseq(id),"Sequence not found");
#undef START
#define START(id)  static_assert(hasseq(id),"Sequence not found");
#undef SENDLOCO
#define SENDLOCO(cab,id) static_assert(hasseq(id),"Sequence not found");
#undef LATCH
#define LATCH(id) static_assert(id>=0 && id<MAX_FLAGS,"Id out of valid range 0-255" );
#undef UNLATCH
#define UNLATCH(id) static_assert(id>=0 && id<MAX_FLAGS,"Id out of valid range 0-255" );
#undef RESERVE
#define RESERVE(id) static_assert(id>=0 && id<MAX_FLAGS,"Id out of valid range 0-255" );
#undef FREE
#define FREE(id) static_assert(id>=0 && id<MAX_FLAGS,"Id out of valid range 0-255" );
#undef SPEED
#define SPEED(speed) static_assert(speed>=0 && speed<128,"Speed out of valid range 0-127");
#undef FWD
#define FWD(speed) static_assert(speed>=0 && speed<128,"Speed out of valid range 0-127");
#undef REV
#define REV(speed) static_assert(speed>=0 && speed<128,"Speed out of valid range 0-127");

#include "myAutomation.h"

// Pass 1h Implements HAL macro by creating exrailHalSetup function
// Also allows creating EXTurntable object
#include "EXRAIL2MacroReset.h"
#undef HAL
#define HAL(haltype,params...)  haltype::create(params);
#undef HAL_IGNORE_DEFAULTS
#define HAL_IGNORE_DEFAULTS ignore_defaults=true;
#undef JMRI_SENSOR
#define JMRI_SENSOR(vpin,count...) Sensor::createMultiple(vpin,##count);
#undef  CONFIGURE_SERVO
#define CONFIGURE_SERVO(vpin,pos1,pos2,profile) IODevice::configureServo(vpin,pos1,pos2,PCA9685::profile);
bool exrailHalSetup() {
   bool ignore_defaults=false;
   #include "myAutomation.h"
   return ignore_defaults;
}

// Pass 1c detect compile time featurtes
#include "EXRAIL2MacroReset.h"
#undef SIGNAL
#define SIGNAL(redpin,amberpin,greenpin) | FEATURE_SIGNAL 
#undef SIGNALH
#define SIGNALH(redpin,amberpin,greenpin) | FEATURE_SIGNAL 
#undef SERVO_SIGNAL
#define SERVO_SIGNAL(vpin,redval,amberval,greenval) | FEATURE_SIGNAL 
#undef DCC_SIGNAL
#define DCC_SIGNAL(id,addr,subaddr) | FEATURE_SIGNAL
#undef DCCX_SIGNAL
#define DCCX_SIGNAL(id,redAspect,amberAspect,greenAspect) | FEATURE_SIGNAL
#undef VIRTUAL_SIGNAL
#define VIRTUAL_SIGNAL(id) | FEATURE_SIGNAL

#undef LCC
#define LCC(eventid)  | FEATURE_LCC
#undef LCCX
#define LCCX(senderid,eventid) | FEATURE_LCC 
#undef ONLCC
#define ONLCC(senderid,eventid) | FEATURE_LCC
#undef ROUTE_ACTIVE
#define ROUTE_ACTIVE(id) | FEATURE_ROUTESTATE
#undef ROUTE_INACTIVE
#define ROUTE_INACTIVE(id) | FEATURE_ROUTESTATE
#undef ROUTE_HIDDEN
#define ROUTE_HIDDEN(id) | FEATURE_ROUTESTATE
#undef ROUTE_DISABLED
#define ROUTE_DISABLED(id) | FEATURE_ROUTESTATE
#undef ROUTE_CAPTION
#define ROUTE_CAPTION(id,caption) | FEATURE_ROUTESTATE

#undef CLEAR_STASH
#define CLEAR_STASH(id) | FEATURE_STASH
#undef CLEAR_ALL_STASH
#define CLEAR_ALL_STASH | FEATURE_STASH
#undef PICKUP_STASH
#define PICKUP_STASH(id) | FEATURE_STASH
#undef STASH
#define STASH(id) | FEATURE_STASH

const byte RMFT2::compileFeatures = 0
   #include "myAutomation.h"
;

// Pass 2 create throttle route list 
#include "EXRAIL2MacroReset.h"
#undef ROUTE
#define ROUTE(id, description) id,
const int16_t HIGHFLASH  RMFT2::routeIdList[]= {
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
#undef ROUTE_CAPTION
#define ROUTE_CAPTION(id,caption) \
case (__COUNTER__ - StringMacroTracker1) : {\
   manageRouteCaption(id,F(caption));\
   return;\
   }
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
#undef STEALTH
#define STEALTH(code...) case (__COUNTER__ - StringMacroTracker1) : {code} return; 
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
#undef TURNOUTL
#define TURNOUTL(id,addr,description...) O_DESC(id,description)
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

// Pass to get turntable descriptions (optional)
#include "EXRAIL2MacroReset.h"
#undef DCC_TURNTABLE
#define DCC_TURNTABLE(id,home,description...) O_DESC(id,description)
#undef EXTT_TURNTABLE
#define EXTT_TURNTABLE(id,vpin,home,description...) O_DESC(id,description)

const FSH * RMFT2::getTurntableDescription(int16_t turntableId) {
   switch (turntableId) {
      #include "myAutomation.h"
   default:break;
   }
   return NULL;
}

// Pass to get turntable position descriptions (optional)
#include "EXRAIL2MacroReset.h"
#undef TT_ADDPOSITION
#define TT_ADDPOSITION(turntable_id,position,value,home,description...) T_DESC(turntable_id,position,description)

const FSH * RMFT2::getTurntablePositionDescription(int16_t turntableId, uint8_t positionId) {
   #include "myAutomation.h"
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
#undef DCCX_SIGNAL
#define DCCX_SIGNAL(id,redAspect,amberAspect,greenAspect) id | RMFT2::DCCX_SIGNAL_FLAG,redAspect,amberAspect,greenAspect,
#undef VIRTUAL_SIGNAL
#define VIRTUAL_SIGNAL(id) id,0,0,0,

const  HIGHFLASH  int16_t RMFT2::SignalDefinitions[] = {
    #include "myAutomation.h"
    0,0,0,0 };

// Pass 9 ONLCC counter and lookup array
#include "EXRAIL2MacroReset.h"
#undef ONLCC
#define ONLCC(sender,event) +1 

const int RMFT2::countLCCLookup=0
#include "myAutomation.h"
;
int RMFT2::onLCCLookup[RMFT2::countLCCLookup];

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
#define AFTEROVERLOAD(track_id) OPCODE_AFTEROVERLOAD,V(TRACK_NUMBER_##track_id),
#define ALIAS(name,value...) 
#define AMBER(signal_id) OPCODE_AMBER,V(signal_id),
#define ANOUT(vpin,value,param1,param2) OPCODE_SERVO,V(vpin),OPCODE_PAD,V(value),OPCODE_PAD,V(param1),OPCODE_PAD,V(param2),
#define ASPECT(address,value) OPCODE_ASPECT,V((address<<5) | (value & 0x1F)),
#define AT(sensor_id) OPCODE_AT,V(sensor_id),
#define ATGTE(sensor_id,value) OPCODE_ATGTE,V(sensor_id),OPCODE_PAD,V(value),  
#define ATLT(sensor_id,value) OPCODE_ATLT,V(sensor_id),OPCODE_PAD,V(value),  
#define ATTIMEOUT(sensor_id,timeout) OPCODE_ATTIMEOUT1,0,0,OPCODE_ATTIMEOUT2,V(sensor_id),OPCODE_PAD,V(timeout/100L),
#define AUTOMATION(id, description)  OPCODE_AUTOMATION, V(id), 
#define AUTOSTART OPCODE_AUTOSTART,0,0,
#define BROADCAST(msg) PRINT(msg)
#define CALL(route) OPCODE_CALL,V(route),
#define CLEAR_STASH(id) OPCODE_CLEAR_STASH,V(id),
#define CLEAR_ALL_STASH OPCODE_CLEAR_ALL_STASH,V(0),
#define CLOSE(id)  OPCODE_CLOSE,V(id),
#define CONFIGURE_SERVO(vpin,pos1,pos2,profile)
#ifndef IO_NO_HAL
#define DCC_TURNTABLE(id,home,description...) OPCODE_DCCTURNTABLE,V(id),OPCODE_PAD,V(home),
#endif
#define DEACTIVATE(addr,subaddr) OPCODE_DCCACTIVATE,V(addr<<3 | subaddr<<1),
#define DEACTIVATEL(addr) OPCODE_DCCACTIVATE,V((addr+3)<<1),
#define DELAY(ms) ms<30000?OPCODE_DELAYMS:OPCODE_DELAY,V(ms/(ms<30000?1L:100L)),
#define DELAYMINS(mindelay) OPCODE_DELAYMINS,V(mindelay),
#define DELAYRANDOM(mindelay,maxdelay) DELAY(mindelay) OPCODE_RANDWAIT,V((maxdelay-mindelay)/100L),
#define DCC_SIGNAL(id,add,subaddr)
#define DCCX_SIGNAL(id,redAspect,amberAspect,greenAspect)
#define DONE OPCODE_ENDTASK,0,0,
#define DRIVE(analogpin) OPCODE_DRIVE,V(analogpin),
#define ELSE OPCODE_ELSE,0,0,
#define ENDEXRAIL 
#define ENDIF  OPCODE_ENDIF,0,0,
#define ENDTASK OPCODE_ENDTASK,0,0,
#define ESTOP OPCODE_SPEED,V(1), 
#define EXRAIL
#ifndef IO_NO_HAL
#define EXTT_TURNTABLE(id,vpin,home,description...) OPCODE_EXTTTURNTABLE,V(id),OPCODE_PAD,V(vpin),OPCODE_PAD,V(home),
#endif
#define FADE(pin,value,ms) OPCODE_SERVO,V(pin),OPCODE_PAD,V(value),OPCODE_PAD,V(PCA9685::ProfileType::UseDuration|PCA9685::NoPowerOff),OPCODE_PAD,V(ms/100L),
#define FOFF(func) OPCODE_FOFF,V(func),
#define FOLLOW(route) OPCODE_FOLLOW,V(route),
#define FON(func) OPCODE_FON,V(func),
#define FORGET OPCODE_FORGET,0,0,
#define FREE(blockid) OPCODE_FREE,V(blockid),
#define FWD(speed) OPCODE_FWD,V(speed),
#define GREEN(signal_id) OPCODE_GREEN,V(signal_id),
#define HAL(haltype,params...)
#define HAL_IGNORE_DEFAULTS
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
#ifndef IO_NO_HAL
#define IFTTPOSITION(id,position) OPCODE_IFTTPOSITION,V(id),OPCODE_PAD,V(position),
#endif
#define IFRE(sensor_id,value) OPCODE_IFRE,V(sensor_id),OPCODE_PAD,V(value),
#define INVERT_DIRECTION OPCODE_INVERT_DIRECTION,0,0,
#define JMRI_SENSOR(vpin,count...)
#define JOIN OPCODE_JOIN,0,0,
#define KILLALL OPCODE_KILLALL,0,0,
#define LATCH(sensor_id) OPCODE_LATCH,V(sensor_id),
#define LCC(eventid) OPCODE_LCC,V(eventid),
#define LCCX(sender,event) OPCODE_LCCX,V(event),\
        OPCODE_PAD,V((((uint64_t)sender)>>32)&0xFFFF),\
        OPCODE_PAD,V((((uint64_t)sender)>>16)&0xFFFF),\
        OPCODE_PAD,V((((uint64_t)sender)>>0)&0xFFFF),  
#define LCD(id,msg) PRINT(msg)
#define SCREEN(display,id,msg) PRINT(msg)
#define STEALTH(code...) PRINT(dummy)
#define LCN(msg) PRINT(msg)
#define MOVETT(id,steps,activity) OPCODE_SERVO,V(id),OPCODE_PAD,V(steps),OPCODE_PAD,V(EXTurntable::activity),OPCODE_PAD,V(0),
#define ONACTIVATE(addr,subaddr) OPCODE_ONACTIVATE,V(addr<<2|subaddr),
#define ONACTIVATEL(linear) OPCODE_ONACTIVATE,V(linear+3),
#define ONAMBER(signal_id) OPCODE_ONAMBER,V(signal_id),
#define ONCLOSE(turnout_id) OPCODE_ONCLOSE,V(turnout_id),
#define ONLCC(sender,event) OPCODE_ONLCC,V(event),\
        OPCODE_PAD,V((((uint64_t)sender)>>32)&0xFFFF),\
        OPCODE_PAD,V((((uint64_t)sender)>>16)&0xFFFF),\
        OPCODE_PAD,V((((uint64_t)sender)>>0)&0xFFFF),        
#define ONTIME(value) OPCODE_ONTIME,V(value),  
#define ONCLOCKTIME(hours,mins) OPCODE_ONTIME,V((STRIP_ZERO(hours)*60)+STRIP_ZERO(mins)),
#define ONCLOCKMINS(mins) ONCLOCKTIME(25,mins)
#define ONOVERLOAD(track_id) OPCODE_ONOVERLOAD,V(TRACK_NUMBER_##track_id),
#define ONDEACTIVATE(addr,subaddr) OPCODE_ONDEACTIVATE,V(addr<<2|subaddr),
#define ONDEACTIVATEL(linear) OPCODE_ONDEACTIVATE,V(linear+3),
#define ONGREEN(signal_id) OPCODE_ONGREEN,V(signal_id),
#define ONRED(signal_id) OPCODE_ONRED,V(signal_id),
#ifndef IO_NO_HAL
#define ONROTATE(id) OPCODE_ONROTATE,V(id),
#endif
#define ONTHROW(turnout_id) OPCODE_ONTHROW,V(turnout_id),
#define ONCHANGE(sensor_id) OPCODE_ONCHANGE,V(sensor_id),
#define PAUSE OPCODE_PAUSE,0,0,
#define PICKUP_STASH(id) OPCODE_PICKUP_STASH,V(id),
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
#ifndef IO_NO_HAL
#define ROTATE(id,position,activity) OPCODE_ROTATE,V(id),OPCODE_PAD,V(position),OPCODE_PAD,V(EXTurntable::activity),
#define ROTATE_DCC(id,position) OPCODE_ROTATE,V(id),OPCODE_PAD,V(position),OPCODE_PAD,V(0),
#endif
#define ROUTE(id, description)  OPCODE_ROUTE, V(id), 
#define ROUTE_ACTIVE(id)  OPCODE_ROUTE_ACTIVE,V(id),
#define ROUTE_INACTIVE(id)  OPCODE_ROUTE_INACTIVE,V(id),
#define ROUTE_HIDDEN(id)  OPCODE_ROUTE_HIDDEN,V(id),
#define ROUTE_DISABLED(id)  OPCODE_ROUTE_DISABLED,V(id),
#define ROUTE_CAPTION(id,caption) PRINT(caption)
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
#define SET_POWER(track,onoff) OPCODE_SET_POWER,V(TRACK_POWER_##onoff),OPCODE_PAD, V(TRACK_NUMBER_##track),
#define SETLOCO(loco) OPCODE_SETLOCO,V(loco),
#define SIGNAL(redpin,amberpin,greenpin) 
#define SIGNALH(redpin,amberpin,greenpin) 
#define SPEED(speed) OPCODE_SPEED,V(speed),
#define START(route) OPCODE_START,V(route), 
#define STASH(id) OPCODE_STASH,V(id), 
#define STOP OPCODE_SPEED,V(0), 
#define THROW(id)  OPCODE_THROW,V(id),
#ifndef IO_NO_HAL
#define TT_ADDPOSITION(id,position,value,angle,description...) OPCODE_TTADDPOSITION,V(id),OPCODE_PAD,V(position),OPCODE_PAD,V(value),OPCODE_PAD,V(angle),
#endif
#define TURNOUT(id,addr,subaddr,description...) OPCODE_TURNOUT,V(id),OPCODE_PAD,V(addr),OPCODE_PAD,V(subaddr),
#define TURNOUTL(id,addr,description...) TURNOUT(id,(addr-1)/4+1,(addr-1)%4, description)
#define UNJOIN OPCODE_UNJOIN,0,0,
#define UNLATCH(sensor_id) OPCODE_UNLATCH,V(sensor_id),
#define VIRTUAL_SIGNAL(id) 
#define VIRTUAL_TURNOUT(id,description...) OPCODE_PINTURNOUT,V(id),OPCODE_PAD,V(0), 
#define WITHROTTLE(msg) PRINT(msg)
#define WAITFOR(pin) OPCODE_WAITFOR,V(pin),
#ifndef IO_NO_HAL
#define WAITFORTT(turntable_id) OPCODE_WAITFORTT,V(turntable_id),
#endif
#define XFOFF(cab,func) OPCODE_XFOFF,V(cab),OPCODE_PAD,V(func),
#define XFON(cab,func) OPCODE_XFON,V(cab),OPCODE_PAD,V(func),

// Build RouteCode
const int StringMacroTracker2=__COUNTER__;
const  HIGHFLASH3  byte RMFT2::RouteCode[] = {
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
