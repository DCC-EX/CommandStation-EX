/*
 *  © 2020, Chris Harlow. All rights reserved.
 *  
 *  This file is part of Asbelos DCC API
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
#ifndef DCC_h
#define DCC_h
#include <Arduino.h>
#include "MotorDriver.h"
#include "MotorDrivers.h"

typedef void (*ACK_CALLBACK)(int result);

enum ackOp {  // Program opcodes for the ack Manager
BASELINE,  // ensure enough resets sent before starting and obtain baseline current
W0,W1,   // issue write bit (0..1) packet 
WB,     // issue write byte packet 
VB,     // Issue validate Byte packet
V0,     // Issue validate bit=0 packet
V1,     // issue validate bit=1 packlet
WACK,   // wait for ack (or absence of ack)
ITC1,   // If True Callback(1)  (if prevous WACK got an ACK)
ITC0,   // If True callback(0);
ITCB,   // If True callback(byte)
NAKFAIL, // if false callback(-1)
FAIL,   // callback(-1)
STARTMERGE,    // Clear bit and byte settings ready for merge pass 
MERGE,  // Merge previous wack response with byte value and decrement bit number (use for readimng CV bytes)
SETBIT, // sets bit number to next prog byte
SETCV, // sets cv number to next prog byte
STASHLOCOID, // keeps current byte value for later
COMBINELOCOID, // combines current value with stashed value and returns it
ITSKIP,        // skip to SKIPTARGET if ack true
SKIPTARGET=0xFF  // jump to target
};

// Allocations with memory implications..!
// Base system takes approx 900 bytes + 8 per loco. Turnouts, Sensors etc are dynamically created
 #ifdef ARDUINO_AVR_UNO 
  const byte MAX_LOCOS=20;
 #else 
  const byte MAX_LOCOS=50; 
 #endif              


class DCC {
  public:

  static void begin(MotorDriver * mainDriver, MotorDriver * progDriver);
  static void loop();

  // Public DCC API functions
  static void setThrottle( uint16_t cab, uint8_t tSpeed, bool tDirection);
  static uint8_t getThrottleSpeed(int cab);
  static bool getThrottleDirection(int cab);
  static void writeCVByteMain(int cab, int cv, byte bValue);
  static void writeCVBitMain(int cab, int cv, byte bNum, bool bValue);
  static void setFunction( int cab, byte fByte, byte eByte);
  static void setFn( int cab, byte functionNumber, bool on);
  static int  changeFn( int cab, byte functionNumber, bool pressed);
  static void updateGroupflags(byte & flags, int functionNumber);
  static void setAccessory(int aAdd, byte aNum, bool activate) ;
  static bool writeTextPacket( byte *b, int nBytes);
  static void setDebug(bool on);
  static void setProgTrackSyncMain(bool on); // when true, prog track becomes driveable 
  
  // ACKable progtrack calls  bitresults callback 0,0 or -1, cv returns value or -1 
  static void  readCV(int cv, ACK_CALLBACK callback, bool blocking=false);
  static void  readCVBit(int cv, byte bitNum, ACK_CALLBACK callback, bool blocking=false);  // -1 for error
  static void writeCVByte(int cv, byte byteValue, ACK_CALLBACK callback, bool blocking=false) ;
  static void writeCVBit(int cv, byte bitNum, bool bitValue, ACK_CALLBACK callback, bool blocking=false);
  
  static void getLocoId(ACK_CALLBACK callback, bool blocking=false);

  // Enhanced API functions
  static void forgetLoco(int cab);  // removes any speed reminders for this loco  
  static void forgetAllLocos();  // removes all speed reminders
    
private: 
  struct LOCO {
     int loco;
     byte speedCode;
     byte groupFlags;
     unsigned long functions;
  };
  static byte loopStatus;
  static void setThrottle2( uint16_t cab, uint8_t speedCode);
  static void updateLocoReminder(int loco, byte speedCode);
  static void setFunctionInternal( int cab, byte fByte, byte eByte);
  static bool issueReminder(int reg);
  static int nextLoco;
  static LOCO speedTable[MAX_LOCOS];
  static byte cv1(byte opcode, int cv);
  static byte cv2(int cv);
  static int lookupSpeedTable(int locoId);
  static void issueReminders();
  static void callback(int value);
  static bool debugMode;
  
// ACK MANAGER
  static ackOp  const * ackManagerProg;
  static byte   ackManagerByte;
  static byte   ackManagerBitNum;
  static int    ackManagerCv;
  static byte    ackManagerStash;
  static bool ackReceived;
  static ACK_CALLBACK  ackManagerCallback;
  static void ackManagerSetup(int cv, byte bitNumOrbyteValue, ackOp const program[], ACK_CALLBACK callback, bool blocking);
  static void ackManagerLoop(bool blocking);
  static bool checkResets(bool blocking);
  static const int PROG_REPEATS=8;   // repeats of programming commands (some decoders need at least 8 to be reliable) 

  

  // NMRA codes #
  static const byte SET_SPEED=0x3f;
  static const byte WRITE_BYTE_MAIN = 0xEC;
  static const byte WRITE_BIT_MAIN = 0xE8;
  static const byte WRITE_BYTE = 0x7C;
  static const byte VERIFY_BYTE= 0x74;
  static const byte BIT_MANIPULATE=0x78;
  static const byte WRITE_BIT=0xF0;
  static const byte VERIFY_BIT=0xE0;
  static const byte BIT_ON=0x08;
  static const byte BIT_OFF=0x00;
};

#endif
