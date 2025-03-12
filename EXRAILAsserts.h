/*
 *  © 2020-2025 Chris Harlow
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

// This file checks the myAutomation for errors by generating a list of compile time asserts.

// Assert Pass 1 Collect sequence numbers.
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


// Compile time function to check for sequence number duplication
constexpr int16_t seqCount(const int16_t value, const int16_t pos=0, const int16_t count=0 ) {
   return pos>=stuffSize? count :
             seqCount(value,pos+1,count+((compileTimeSequenceList[pos]==value)?1:0));
}


// Build a compile time blacklist of pin numbers. 
// Includes those defined in defaults.h for the cpu (PIN_BLACKLIST)
// and cheats in the motor shield pins from config.h (MOTOR_SHIELD_TYPE)
// for reference the MotorDriver constructor is:
// MotorDriver(byte power_pin, byte signal_pin, byte signal_pin2, int8_t brake_pin, byte current_pin,
//             float senseFactor, unsigned int tripMilliamps, byte faultPin);

// create capture macros to reinterpret MOTOR_SHIELD_TYPE from configuration
#define new
#define MotorDriver(power_pin,signal_pin,signal_pin2, \
   brake_pin,current_pin,senseFactor,tripMilliamps,faultPin) \
   abs(power_pin),abs(signal_pin),abs(signal_pin2),abs(brake_pin),abs(current_pin),abs(faultPin)
#ifndef PIN_BLACKLIST 
  #define PIN_BLACKLIST UNUSED_PIN
#endif 
#define MDFURKLE(stuff) MDFURKLE2(stuff)
#define MDFURKLE2(description,...) REMOVE_TRAILING_COMMA(__VA_ARGS__)
#define REMOVE_TRAILING_COMMA(...) __VA_ARGS__


constexpr int16_t compileTimePinBlackList[]={
   PIN_BLACKLIST, MDFURKLE(MOTOR_SHIELD_TYPE)
   };
constexpr int16_t pbSize=sizeof(compileTimePinBlackList)/sizeof(int16_t) - 1;


// remove capture macros
#undef new
#undef MotorDriver

// Compile time function to check for dangerous pins.
constexpr bool unsafePin(const int16_t value, const uint16_t pos=0 ) {
   return pos>=pbSize? false :
      compileTimePinBlackList[pos]==value 
      || unsafePin(value,pos+1); 
}


//pass 2 apply static asserts:
// check call and follows etc for existing sequence numbers
// check sequence numbers for duplicates
// check range on LATCH/UNLATCH
// check range on RESERVE/FREE
// check range on SPEED/FWD/REV
// check range on SET/RESET   (pins that are not safe to use in EXRAIL)
// 
// This pass generates no runtime data or code 
#include "EXRAIL2MacroReset.h"
#undef ASPECT
#define ASPECT(address,value) static_assert(address <=2044, "invalid Address"); \
                              static_assert(address>=-3, "Invalid value");

// check references to sequences/routes/automations
#undef CALL
#define CALL(id) static_assert(seqCount(id)>0,"Sequence  not found");
#undef FOLLOW
#define FOLLOW(id)  static_assert(seqCount(id)>0,"Sequence not found");
#undef START
#define START(id)  static_assert(seqCount(id)>0,"Sequence not found");
#undef SENDLOCO
#define SENDLOCO(cab,id) static_assert(seqCount(id)>0,"Sequence not found");
#undef ROUTE_ACTIVE
#define ROUTE_ACTIVE(id)  static_assert(seqCount(id)>0,"Route not found");
#undef ROUTE_INACTIVE
#define ROUTE_INACTIVE(id)  static_assert(seqCount(id)>0,"Route not found");
#undef ROUTE_HIDDEN
#define ROUTE_HIDDEN(id)  static_assert(seqCount(id)>0,"Route not found");
#undef ROUTE_DISABLED
#define ROUTE_DISABLED(id)  static_assert(seqCount(id)>0,"Route not found");
#undef ROUTE_CAPTION 
#define ROUTE_CAPTION(id,caption) static_assert(seqCount(id)>0,"Route not found");


#undef LATCH
#define LATCH(id) static_assert(id>=0 && id<MAX_FLAGS,"Id out of valid range 0-255" );
#undef UNLATCH
#define UNLATCH(id) static_assert(id>=0 && id<MAX_FLAGS,"Id out of valid range 0-255" );
#undef RESERVE
#define RESERVE(id) static_assert(id>=0 && id<MAX_FLAGS,"Id out of valid range 0-255" );
#undef FREE
#define FREE(id) static_assert(id>=0 && id<MAX_FLAGS,"Id out of valid range 0-255" );
#undef IFRESERVE
#define IFRESERVE(id) static_assert(id>=0 && id<MAX_FLAGS,"Id out of valid range 0-255" );

//check speeds
#undef SPEED
#define SPEED(speed) static_assert(speed>=0 && speed<128,"\n\nUSER ERROR: Speed out of valid range 0-127\n");
#undef FWD
#define FWD(speed) static_assert(speed>=0 && speed<128,"\n\nUSER ERROR: Speed out of valid range 0-127\n");
#undef REV
#define REV(speed) static_assert(speed>=0 && speed<128,"\n\nUSER ERROR: Speed out of valid range 0-127\n");

// check duplicate sequences
#undef SEQUENCE
#define SEQUENCE(id) static_assert(seqCount(id)==1,"\n\nUSER ERROR: Duplicate ROUTE/AUTOMATION/SEQUENCE(" #id ")\n");
#undef AUTOMATION
#define AUTOMATION(id,description) static_assert(seqCount(id)==1,"\n\nUSER ERROR: Duplicate ROUTE/AUTOMATION/SEQUENCE(" #id ")\n");
#undef ROUTE
#define ROUTE(id,description) static_assert(seqCount(id)==1,"\n\nUSER ERROR: Duplicate ROUTE/AUTOMATION/SEQUENCE(" #id ")\n");

// check dangerous pins
#define _PIN_RESERVED_ "\n\nUSER ERROR: Pin is used by Motor Shield or other critical function.\n"
#undef SET
#define SET(vpin) static_assert(!unsafePin(vpin),"SET(" #vpin ")" _PIN_RESERVED_);
#undef RESET
#define RESET(vpin) static_assert(!unsafePin(vpin),"RESET(" #vpin ")" _PIN_RESERVED_);
#undef BLINK
#define BLINK(vpin,onDuty,offDuty) static_assert(!unsafePin(vpin),"BLINK(" #vpin ")" _PIN_RESERVED_);
#undef SIGNAL
#define SIGNAL(redpin,amberpin,greenpin) \
      static_assert(!unsafePin(redpin),"Red pin " #redpin _PIN_RESERVED_); \
      static_assert(amberpin==0 ||!unsafePin(amberpin),"Amber pin " #amberpin _PIN_RESERVED_); \
      static_assert(!unsafePin(greenpin),"Green pin " #greenpin _PIN_RESERVED_); 
#undef SIGNALH
#define SIGNALH(redpin,amberpin,greenpin) \
      static_assert(!unsafePin(redpin),"Red pin " #redpin _PIN_RESERVED_); \
      static_assert(amberpin==0 ||!unsafePin(amberpin),"Amber pin " #amberpin _PIN_RESERVED_); \
      static_assert(!unsafePin(greenpin),"Green pin " #greenpin _PIN_RESERVED_); 

// and run the assert pass.       
#include "myAutomation.h"
