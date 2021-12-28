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
#include "FSH.h"

#include "defines.h"
#ifndef HIGHEST_SHORT_ADDR
#define HIGHEST_SHORT_ADDR 127
#else
#if HIGHEST_SHORT_ADDR > 127
#error short addr greater than 127 does not make sense
#endif
#endif
const uint16_t LONG_ADDR_MARKER = 0x4000;

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
  AFTER_WRITE,  // Start callback sequence after something was written to the decoder  
  WAITING_100,        // Waiting for 100mS of stable power 
  WAITING_30,         // waiting to 30ms of power off gap. 
  READY,              // Ready to complete callback  
  }; 


// Allocations with memory implications..!
// Base system takes approx 900 bytes + 8 per loco. Turnouts, Sensors etc are dynamically created
#if defined(ARDUINO_AVR_UNO)
const byte MAX_LOCOS = 20;
#elif defined(ARDUINO_AVR_NANO)
const byte MAX_LOCOS = 30;
#else
const byte MAX_LOCOS = 50;
#endif

class DCC
{
public:
  static void begin(const FSH * motorShieldName, MotorDriver *mainDriver, MotorDriver *progDriver);
  static void setJoinRelayPin(byte joinRelayPin);
  static void loop();

  // Public DCC API functions
  static void setThrottle(uint16_t cab, uint8_t tSpeed, bool tDirection);
  static uint8_t getThrottleSpeed(int cab);
  static bool getThrottleDirection(int cab);
  static void writeCVByteMain(int cab, int cv, byte bValue);
  static void writeCVBitMain(int cab, int cv, byte bNum, bool bValue);
  static void setFunction(int cab, byte fByte, byte eByte);
  static void setFn(int cab, int16_t functionNumber, bool on);
  static void changeFn(int cab, int16_t functionNumber);
  static int  getFn(int cab, int16_t functionNumber);
  static uint32_t getFunctionMap(int cab);
  static void updateGroupflags(byte &flags, int16_t functionNumber);
  static void setAccessory(int aAdd, byte aNum, bool activate);
  static bool writeTextPacket(byte *b, int nBytes);
  static void setProgTrackSyncMain(bool on); // when true, prog track becomes driveable
  static void setProgTrackBoost(bool on);    // when true, special prog track current limit does not apply

  // ACKable progtrack calls  bitresults callback 0,0 or -1, cv returns value or -1
  static void readCV(int16_t cv, ACK_CALLBACK callback);
  static void readCVBit(int16_t cv, byte bitNum, ACK_CALLBACK callback); // -1 for error
  static void writeCVByte(int16_t cv, byte byteValue, ACK_CALLBACK callback);
  static void writeCVBit(int16_t cv, byte bitNum, bool bitValue, ACK_CALLBACK callback);
  static void verifyCVByte(int16_t cv, byte byteValue, ACK_CALLBACK callback);
  static void verifyCVBit(int16_t cv, byte bitNum, bool bitValue, ACK_CALLBACK callback);

  static void getLocoId(ACK_CALLBACK callback);
  static void setLocoId(int id,ACK_CALLBACK callback);

  // Enhanced API functions
  static void forgetLoco(int cab); // removes any speed reminders for this loco
  static void forgetAllLocos();    // removes all speed reminders
  static void displayCabList(Print *stream);

  static FSH *getMotorShieldName();
  static inline void setGlobalSpeedsteps(byte s) {
    globalSpeedsteps = s;
  };
  static inline int16_t setAckRetry(byte retry) {
    ackRetry = retry;
    ackRetryPSum = ackRetrySum;
    ackRetrySum = 0;  // reset running total
    return ackRetryPSum;
  };

  struct LOCO
  {
    int loco;
    byte speedCode;
    byte groupFlags;
    unsigned long functions;
  };
 static LOCO speedTable[MAX_LOCOS];
 
private:
  static byte joinRelay;
  static byte loopStatus;
  static void setThrottle2(uint16_t cab, uint8_t speedCode);
  static void updateLocoReminder(int loco, byte speedCode);
  static void setFunctionInternal(int cab, byte fByte, byte eByte);
  static bool issueReminder(int reg);
  static int nextLoco;
  static FSH *shieldName;
  static byte globalSpeedsteps;

  static byte cv1(byte opcode, int cv);
  static byte cv2(int cv);
  static int lookupSpeedTable(int locoId);
  static void issueReminders();
  static void callback(int value);

  // ACK MANAGER
  static ackOp const *ackManagerProg;
  static ackOp const *ackManagerProgStart;
  static byte ackManagerByte;
  static byte ackManagerByteVerify;
  static byte ackManagerBitNum;
  static int ackManagerCv;
  static byte ackManagerRetry;
  static byte ackRetry;
  static int16_t ackRetrySum;
  static int16_t ackRetryPSum;
  static int ackManagerWord;
  static byte ackManagerStash;
  static bool ackReceived;
  static bool ackManagerRejoin;
  static ACK_CALLBACK ackManagerCallback;
  static CALLBACK_STATE callbackState;
  static void ackManagerSetup(int cv, byte bitNumOrbyteValue, ackOp const program[], ACK_CALLBACK callback);
  static void ackManagerSetup(int wordval, ackOp const program[], ACK_CALLBACK callback);
  static void ackManagerLoop();
  static bool checkResets( uint8_t numResets);
  static const int PROG_REPEATS = 8; // repeats of programming commands (some decoders need at least 8 to be reliable)
  
  // NMRA codes #
  static const byte SET_SPEED = 0x3f;
  static const byte WRITE_BYTE_MAIN = 0xEC;
  static const byte WRITE_BIT_MAIN = 0xE8;
  static const byte WRITE_BYTE = 0x7C;
  static const byte VERIFY_BYTE = 0x74;
  static const byte BIT_MANIPULATE = 0x78;
  static const byte WRITE_BIT = 0xF0;
  static const byte VERIFY_BIT = 0xE0;
  static const byte BIT_ON = 0x08;
  static const byte BIT_OFF = 0x00;
};

#ifdef ARDUINO_AVR_MEGA // is using Mega 1280, define as Mega 2560 (pinouts and functionality are identical)
#define ARDUINO_AVR_MEGA2560
#endif

#if defined(ARDUINO_AVR_UNO)
#define ARDUINO_TYPE "UNO"
#elif defined(ARDUINO_AVR_NANO)
#define ARDUINO_TYPE "NANO"
#elif defined(ARDUINO_AVR_MEGA2560)
#define ARDUINO_TYPE "MEGA"
#elif defined(ARDUINO_ARCH_MEGAAVR)
#define ARDUINO_TYPE "MEGAAVR"
#elif defined(ARDUINO_TEENSY32)
#define ARDUINO_TYPE "TEENSY32"
#elif defined(ARDUINO_TEENSY35)
#define ARDUINO_TYPE "TEENSY35"
#elif defined(ARDUINO_TEENSY36)
#define ARDUINO_TYPE "TEENSY36"
#elif defined(ARDUINO_TEENSY40)
#define ARDUINO_TYPE "TEENSY40"
#elif defined(ARDUINO_TEENSY41)
#define ARDUINO_TYPE "TEENSY41"
#elif defined(ARDUINO_ARCH_ESP8266)
#define ARDUINO_TYPE "ESP8266"
#elif defined(ARDUINO_ARCH_ESP32)
#define ARDUINO_TYPE "ESP32"
#else
#error CANNOT COMPILE - DCC++ EX ONLY WORKS WITH AN ARDUINO UNO, NANO 328, OR ARDUINO MEGA 1280/2560
#endif


#endif
