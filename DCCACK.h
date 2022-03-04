/*
 *  © 2021 M Steve Todd
 *  © 2021 Mike S
 *  © 2021 Fred Decker
 *  © 2020-2021 Harald Barth
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
#ifndef DCCACK_h
#define DCCACK_h

#include "MotorDriver.h"

typedef void (*ACK_CALLBACK)(int16_t result);

enum ackOp : byte
{           // Program opcodes for the ack Manager
  BASELINE, // ensure enough resets sent before starting and obtain baseline current
  W0,
  W1,               // issue write bit (0..1) packet
  WB,               // issue write byte packet
  VB,               // Issue validate Byte packet
  V0,               // Issue validate bit=0 packet
  V1,               // issue validate bit=1 packlet
  WACK,             // wait for ack (or absence of ack)
  ITC1,             // If True Callback(1)  (if prevous WACK got an ACK)
  ITC0,             // If True callback(0);
  ITCB,             // If True callback(byte)
  ITCBV,            // If True callback(byte) - end of Verify Byte
  ITCB7,            // If True callback(byte &0x7F)
  NAKFAIL,          // if false callback(-1)
  CALLFAIL,         // callback(-1)
  BIV,              // Set ackManagerByte to initial value for Verify retry
  STARTMERGE,       // Clear bit and byte settings ready for merge pass
  MERGE,            // Merge previous wack response with byte value and decrement bit number (use for readimng CV bytes)
  SETBIT,           // sets bit number to next prog byte
  SETCV,            // sets cv number to next prog byte
  SETBYTE,          // sets current byte to next prog byte
  SETBYTEH,         // sets current byte to word high byte
  SETBYTEL,         // sets current byte to word low byte
  STASHLOCOID,      // keeps current byte value for later
  COMBINELOCOID,    // combines current value with stashed value and returns it
  ITSKIP,           // skip to SKIPTARGET if ack true
  SKIPTARGET = 0xFF // jump to target
};

enum   CALLBACK_STATE : byte {

  AFTER_READ,   // Start callback sequence after something was read from the decoder  
  AFTER_WRITE,  // Start callback sequence after something was written to the decoder  
  WAITING_100,        // Waiting for 100mS of stable power 
  WAITING_30,         // waiting to 30ms of power off gap. 
  READY,              // Ready to complete callback  
  }; 



class DCCACK {
  public:
    static byte getAck();               //prog track only 0=NACK, 1=ACK 2=keep waiting
    static void checkAck(byte sentResetsSincePacket); // Interrupt time ack checker
    static inline void setAckLimit(int mA) {
	ackLimitmA = mA;
    }
    static inline void setMinAckPulseDuration(unsigned int i) {
	minAckPulseDuration = i;
    }
    static inline void setMaxAckPulseDuration(unsigned int i) {
	maxAckPulseDuration = i;
    }

    static void  Setup(int cv, byte byteValueOrBitnum, ackOp const program[], ACK_CALLBACK callback);
    static void  Setup(int wordval, ackOp const program[], ACK_CALLBACK callback);
    static void loop();
    static bool isActive() { return ackManagerProg!=NULL;}
  static inline int16_t setAckRetry(byte retry) {
    ackRetry = retry;
    ackRetryPSum = ackRetrySum;
    ackRetrySum = 0;  // reset running total
    return ackRetryPSum;
  };


  private:
    static const byte SET_SPEED = 0x3f;
    static const byte WRITE_BYTE = 0x7C;
    static const byte VERIFY_BYTE = 0x74;
    static const byte BIT_MANIPULATE = 0x78;
    static const byte WRITE_BIT = 0xF0;
    static const byte VERIFY_BIT = 0xE0;
    static const byte BIT_ON = 0x08;
    static const byte BIT_OFF = 0x00;
 
    static void setAckBaseline();
    static void setAckPending();  
    static void callback(int value);
    
    static const int PROG_REPEATS = 8; // repeats of programming commands (some decoders need at least 8 to be reliable)
    
    // ACK management (Prog track only)  
    static void checkAck();
    static bool checkResets(uint8_t numResets);

    static volatile bool ackPending;
    static volatile bool ackDetected;
    static int  ackThreshold; 
    static int  ackLimitmA;
    static int ackMaxCurrent;
    static unsigned long ackCheckStart; // millis
    static unsigned int ackCheckDuration; // millis       
    
    static unsigned int ackPulseDuration;  // micros
    static unsigned long ackPulseStart; // micros

    static unsigned int minAckPulseDuration ; // micros
    static unsigned int maxAckPulseDuration ; // micros
    static MotorDriver* progDriver;
    static volatile uint8_t numAckGaps;
    static volatile uint8_t numAckSamples;
    static uint8_t trailingEdgeCounter;
    static ackOp  const *  ackManagerProg;
static ackOp  const *  ackManagerProgStart;
static byte   ackManagerByte;
static byte   ackManagerByteVerify;
static byte   ackManagerStash;
static int    ackManagerWord;
static byte   ackManagerRetry;
static byte   ackRetry;
static int16_t ackRetrySum;
static int16_t  ackRetryPSum;
static int    ackManagerCv;
static byte   ackManagerBitNum;
static bool   ackReceived;
static bool   ackManagerRejoin;
static bool   autoPowerOff;
static CALLBACK_STATE callbackState;
static ACK_CALLBACK ackManagerCallback;


};
#endif
