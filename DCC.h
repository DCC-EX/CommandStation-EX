/*
 *  © 2021 Mike S
 *  © 2021 Fred Decker
 *  © 2021 Herb Morton
 *  © 2020-2021 Harald Barth
 *  © 2020-2021 Chris Harlow
 *  All rights reserved.
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
#include "DCCACK.h"
const uint16_t LONG_ADDR_MARKER = 0x4000;


// Allocations with memory implications..!
// Base system takes approx 900 bytes + 8 per loco. Turnouts, Sensors etc are dynamically created
#if defined(HAS_ENOUGH_MEMORY)
const byte MAX_LOCOS = 50;
#else
const byte MAX_LOCOS = 30;
#endif

class DCC
{
public:
  static inline void setShieldName(const FSH * motorShieldName) {
    shieldName=(FSH *)motorShieldName;
  };
  static void begin();
  static void loop();

  // Public DCC API functions
  static void setThrottle(uint16_t cab, uint8_t tSpeed, bool tDirection);
  static int8_t getThrottleSpeed(int cab);
  static uint8_t getThrottleSpeedByte(int cab);
  static bool getThrottleDirection(int cab);
  static void writeCVByteMain(int cab, int cv, byte bValue);
  static void writeCVBitMain(int cab, int cv, byte bNum, bool bValue);
  static void setFunction(int cab, byte fByte, byte eByte);
  static bool setFn(int cab, int16_t functionNumber, bool on);
  static void changeFn(int cab, int16_t functionNumber);
  static int  getFn(int cab, int16_t functionNumber);
  static uint32_t getFunctionMap(int cab);
  static void updateGroupflags(byte &flags, int16_t functionNumber);
  static void setAccessory(int address, byte port, bool gate, byte onoff = 2);
  static bool setExtendedAccessory(int16_t address, int16_t value, byte repeats=3);
  static bool writeTextPacket(byte *b, int nBytes);
  
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
  
  struct LOCO
  {
    int loco;
    byte speedCode;
    byte groupFlags;
    unsigned long functions;
  };
 static LOCO speedTable[MAX_LOCOS];
 static int lookupSpeedTable(int locoId, bool autoCreate=true);
 static byte cv1(byte opcode, int cv);
 static byte cv2(int cv);
 
private:
  static byte loopStatus;
  static void setThrottle2(uint16_t cab, uint8_t speedCode);
  static void updateLocoReminder(int loco, byte speedCode);
  static void setFunctionInternal(int cab, byte fByte, byte eByte, byte count);
  static bool issueReminder(int reg);
  static int lastLocoReminder;
  static int highestUsedReg;
  static FSH *shieldName;
  static byte globalSpeedsteps;

  static void issueReminders();
  static void callback(int value);

  
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

#endif
