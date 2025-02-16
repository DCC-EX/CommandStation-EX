/*
 *  © 2021 Neil McKechnie
 *  © 2021 Mike S
 *  © 2021 Fred Decker
 *  © 2021 Herb Morton
 *  © 2020-2022 Harald Barth
 *  © 2020-2021 M Steve Todd
 *  © 2020-2025 Chris Harlow
 *  All rights reserved.
 *
 *  This file is part of DCC-EX
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
#include "DIAG.h"
#include "DCC.h"
#include "DCCWaveform.h"
#ifndef DISABLE_EEPROM
#include "EEStore.h"
#endif
#include "GITHUB_SHA.h"
#include "version.h"
#include "FSH.h"
#include "IODevice.h"
#include "EXRAIL2.h"
#include "CommandDistributor.h"
#include "TrackManager.h"
#include "DCCTimer.h"
#include "Railcom.h"
#include "DCCQueue.h"

// This module is responsible for converting API calls into
// messages to be sent to the waveform generator.
// It has no visibility of the hardware, timers, interrupts
// nor of the waveform issues such as preambles, start bits checksums or cutouts.
//
// Nor should it have to deal with JMRI responsess other than the OK/FAIL
// or cv value returned. I will move that back to the JMRI interface later
//
// The interface to the waveform generator is narrowed down to merely:
//   Scheduling a message on the prog or main track using a function
//   Obtaining ACKs from the prog track using a function
//   There are no volatiles here.

const byte FN_GROUP_1=0x01;
const byte FN_GROUP_2=0x02;
const byte FN_GROUP_3=0x04;
const byte FN_GROUP_4=0x08;
const byte FN_GROUP_5=0x10;

FSH* DCC::shieldName=NULL;
byte DCC::globalSpeedsteps=128;

#define SLOTLOOP for (auto slot=&speedTable[0];slot!=&speedTable[MAX_LOCOS];slot++)

void DCC::begin() {
  StringFormatter::send(&USB_SERIAL,F("<iDCC-EX V-%S / %S / %S G-%S>\n"), F(VERSION), F(ARDUINO_TYPE), shieldName, F(GITHUB_SHA));
#ifndef DISABLE_EEPROM
  // Load stuff from EEprom
  (void)EEPROM; // tell compiler not to warn this is unused
  EEStore::init();
#endif
#ifndef ARDUINO_ARCH_ESP32 /* On ESP32 started in TrackManager::setTrackMode() */
  DCCWaveform::begin();
#endif
}

byte DCC::defaultMomentumA=0;
byte DCC::defaultMomentumD=0;
bool DCC::linearAcceleration=false; 

byte DCC::getMomentum(LOCO * slot) {
   auto target=slot->targetSpeed & 0x7f;
   auto current=slot->speedCode & 0x7f;
   if (target > current) {
    // accelerating
    auto momentum=slot->momentumA==MOMENTUM_USE_DEFAULT ? defaultMomentumA : slot->momentumA;
    // if nonlinear acceleration, momentum is reduced according to 
    // gap between throttle and speed.
    // ie. Loco takes accelerates faster if high throttle
    if (momentum==0 || linearAcceleration) return momentum;
    auto powerDifference= (target-current)/8;
    if (momentum-powerDifference <0) return 0;
    return momentum-powerDifference; 
   }
   return slot->momentumD==MOMENTUM_USE_DEFAULT ? defaultMomentumD : slot->momentumD;
}

void DCC::setThrottle( uint16_t cab, uint8_t tSpeed, bool tDirection)  {
  if (tSpeed==1) {
    if (cab==0) {
      estopAll(); // ESTOP broadcast fix 
      return;
    }
  } 
  byte speedCode = (tSpeed & 0x7F)  + tDirection * 128;
  LOCO * slot=lookupSpeedTable(cab);
  if (slot->targetSpeed==speedCode) return; 
  slot->targetSpeed=speedCode;
  byte momentum=getMomentum(slot);
  if (momentum && tSpeed!=1) { // not ESTOP
    // we dont throttle speed, we just let the reminders take it to target
    slot->momentum_base=millis();
  } 
  else {  // Momentum not involved, throttle now.
    slot->speedCode = speedCode;
    setThrottle2(cab, speedCode);
    TrackManager::setDCSignal(cab,speedCode); // in case this is a dcc track on this addr
  }
  CommandDistributor::broadcastLoco(slot);
}

void DCC::setThrottle2( uint16_t cab, byte speedCode)  {

  uint8_t b[4];
  uint8_t nB = 0;
  // DIAG(F("setSpeedInternal %d %x"),cab,speedCode);

  if (cab > HIGHEST_SHORT_ADDR)
    b[nB++] = highByte(cab) | 0xC0;    // convert train number into a two-byte address
  b[nB++] = lowByte(cab);

  if (globalSpeedsteps <= 28) {

    uint8_t speed128 = speedCode & 0x7F;
    uint8_t speed28;
    uint8_t code28;

    if (speed128 == 0 || speed128 == 1) { // stop or emergency stop
      code28 = speed128;
    } else {
      speed28= (speed128*10+36)/46;                 // convert 2-127 to 1-28
/*
      if (globalSpeedsteps <= 14)                   // Don't want to do 14 steps, to get F0 there is ugly
        code28 = (speed28+3)/2 | (Value of F0);     // convert 1-28 to DCC 14 step speed code
      else
*/
      code28 = (speed28+3)/2 | ( (speed28 & 1) ? 0 : 0b00010000 ); // convert 1-28 to DCC 28 step speed code
    }
    //        Construct command byte from:
    //        command      speed    direction
    b[nB++] = 0b01000000 | code28 | ((speedCode & 0x80) ? 0b00100000 : 0);

  } else { // 128 speedsteps

    b[nB++] = SET_SPEED;                      // 128-step speed control byte
    b[nB++] = speedCode; // for encoding see setThrottle

  }
  if ((speedCode & 0x7F) == 1) DCCQueue::scheduleEstopPacket(b, nB, 4, cab); // highest priority
  else DCCQueue::scheduleDCCSpeedPacket( b, nB, 4, cab);
}

void DCC::setFunctionInternal(int cab, byte byte1, byte byte2, byte count) {
  // DIAG(F("setFunctionInternal %d %x %x"),cab,byte1,byte2);
  byte b[4];
  byte nB = 0;

  if (cab > HIGHEST_SHORT_ADDR)
    b[nB++] = highByte(cab) | 0xC0;    // convert train number into a two-byte address
  b[nB++] = lowByte(cab);
  if (byte1!=0) b[nB++] = byte1;
  b[nB++] = byte2;

  DCCQueue::scheduleDCCPacket(b, nB, count);
}

// returns speed steps 0 to 127 (1 == emergency stop)
// or -1 on "loco not found"
int8_t DCC::getThrottleSpeed(int cab) {
  return getThrottleSpeedByte(cab) & 0x7F;
}

// returns speed code byte
// or 128 (speed 0, dir forward) on "loco not found".
// This is the throttle set speed
uint8_t DCC::getThrottleSpeedByte(int cab) {
  LOCO * slot=lookupSpeedTable(cab,false);
  return slot?slot->targetSpeed:128;
}
// returns speed code byte for loco. 
// This is the most recently send DCC speed packet byte
// or 128 (speed 0, dir forward) on "loco not found".
uint8_t DCC::getLocoSpeedByte(int cab) {
  LOCO* slot=lookupSpeedTable(cab,false);
  return slot?slot->speedCode:128;
}

// returns 0 to 7 for frequency
uint8_t DCC::getThrottleFrequency(int cab) {
#if defined(ARDUINO_AVR_UNO)
  (void)cab;
  return 0;
#else
  LOCO* slot=lookupSpeedTable(cab);
  if (!slot)  return 0; // use default frequency
  // shift out first 29 bits so we have the 3 "frequency bits" left
  uint8_t res = (uint8_t)(slot->functions >>29);
  //DIAG(F("Speed table %d functions %l shifted %d"), reg, slot->functions, res);
  return res;
#endif
}

// returns direction on loco
// or true/forward on "loco not found"
bool DCC::getThrottleDirection(int cab) {
  return getThrottleSpeedByte(cab) & 0x80;
}

// Set function to value on or off
bool DCC::setFn( int cab, int16_t functionNumber, bool on) {
  if (cab<=0 ) return false;
  if (functionNumber < 0) return false;

  if (functionNumber>28) {
    //non reminding advanced binary bit set
    byte b[5];
    byte nB = 0;
    if (cab > HIGHEST_SHORT_ADDR)
      b[nB++] = highByte(cab) | 0xC0;    // convert train number into a two-byte address
    b[nB++] = lowByte(cab);
    if (functionNumber <= 127) {
       b[nB++] = 0b11011101;   // Binary State Control Instruction short form
       b[nB++] = functionNumber | (on ? 0x80 : 0);
    }
    else  {
       b[nB++] = 0b11000000;   // Binary State Control Instruction long form
       b[nB++] = (functionNumber & 0x7F) | (on ? 0x80 : 0);  // low order bits and state flag
       b[nB++] = functionNumber >>7 ;  // high order bits
    }
    DCCQueue::scheduleDCCPacket(b, nB, 4);
  }
  // We use the reminder table up to 28 for normal functions.
  // We use 29 to 31 for DC frequency as well so up to 28
  // are "real" functions and 29 to 31 are frequency bits
  // controlled by function buttons
  if (functionNumber > 31)
    return true;
  
  LOCO * slot = lookupSpeedTable(cab);
  
  // Take care of functions:
  // Set state of function
  uint32_t previous=slot->functions;
  uint32_t funcmask = (1UL<<functionNumber);
  if (on) {
      slot->functions |= funcmask;
  } else {
      slot->functions &= ~funcmask;
  }
  if (slot->functions != previous) {
    if (functionNumber <= 28)
      updateGroupflags(slot->groupFlags, functionNumber);
    CommandDistributor::broadcastLoco(slot);
  }
  return true;
}

// Flip function state (used from withrottle protocol)
void DCC::changeFn( int cab, int16_t functionNumber) {
  auto currentValue=getFn(cab,functionNumber);
  if (currentValue<0) return;  // function not valid for change  
  setFn(cab,functionNumber, currentValue?false:true);
}

// Report function state (used from withrottle protocol)
// returns 0 false, 1 true or -1 for do not know
int8_t DCC::getFn( int cab, int16_t functionNumber) {
  if (cab<=0 || functionNumber>31)
    return -1;  // unknown
  auto slot = lookupSpeedTable(cab);
  
  unsigned long funcmask = (1UL<<functionNumber);
  return  (slot->functions & funcmask)? 1 : 0;
}

// Set the group flag to say we have touched the particular group.
// A group will be reminded only if it has been touched.
void DCC::updateGroupflags(byte & flags, int16_t functionNumber) {
  byte groupMask;
  if (functionNumber<=4)       groupMask=FN_GROUP_1;
  else if (functionNumber<=8)  groupMask=FN_GROUP_2;
  else if (functionNumber<=12) groupMask=FN_GROUP_3;
  else if (functionNumber<=20) groupMask=FN_GROUP_4;
  else                         groupMask=FN_GROUP_5;
  flags |= groupMask;
}

uint32_t DCC::getFunctionMap(int cab) {
  if (cab<=0) return 0;  // unknown pretend all functions off
  auto slot = lookupSpeedTable(cab,false);
  return slot?slot->functions:0;
}

// saves DC frequency (0..3) in spare functions 29,30,31
void DCC::setDCFreq(int cab,byte freq) {
  if (cab==0 || freq>3) return;
  auto slot=lookupSpeedTable(cab,true);
  // drop and replace F29,30,31 (top 3 bits) 
  auto newFunctions=slot->functions & 0x1FFFFFFFUL;
  if (freq==1)      newFunctions |= (1UL<<29); // F29
  else if (freq==2) newFunctions |= (1UL<<30); // F30
  else if (freq==3) newFunctions |= (1UL<<31); // F31
  if (newFunctions==slot->functions) return; // no change 
  slot->functions=newFunctions;
  CommandDistributor::broadcastLoco(slot);
}

void DCC::setAccessory(int address, byte port, bool gate, byte onoff /*= 2*/) {
  // onoff is tristate:
  // 0  => send off packet
  // 1  => send on packet
  // >1 => send both on and off packets.

  // An accessory has an address, 4 ports and 2 gates (coils) each. That's how
  // the initial decoders were orgnized and that influenced how the DCC
  // standard was made.
  #ifdef DIAG_IO
  DIAG(F("DCC::setAccessory(%d,%d,%d)"), address, port, gate);
  #endif
  // use masks to detect wrong values and do nothing
  if(address != (address & 511))
    return;
  if(port != (port & 3))
    return;
  byte b[2];

  // first byte is of the form 10AAAAAA, where AAAAAA represent 6 least signifcant bits of accessory address
  // second byte is of the form 1AAACPPG, where C is 1 for on, PP the ports 0 to 3 and G the gate (coil).
  b[0] = address % 64 + 128;
  b[1] = ((((address / 64) % 8) << 4) + (port % 4 << 1) + gate % 2) ^ 0xF8;
  if (onoff==0) {   // off packet only
    b[1] &= ~0x08; // set C to 0
    DCCQueue::scheduleDCCPacket(b, 2, 3);
  } else if (onoff==1) { // on packet only
    DCCQueue::scheduleDCCPacket(b, 2, 3);
  } else { // auto timed on then off  
    DCCQueue::scheduleAccOnOffPacket(b, 2, 3, 100); // On then off after 100mS
  } 
#if defined(EXRAIL_ACTIVE)
  if (onoff !=0) RMFT2::activateEvent(address<<2|port,gate);
#endif
}

bool DCC::setExtendedAccessory(int16_t address, int16_t value, byte repeats) {

/* From https://www.nmra.org/sites/default/files/s-9.2.1_2012_07.pdf

The Extended Accessory Decoder Control Packet is included for the purpose of transmitting aspect control to signal 
decoders or data bytes to more complex accessory decoders. Each signal head can display one aspect at a time. 
{preamble} 0 10AAAAAA 0 0AAA0AA1 0 000XXXXX 0 EEEEEEEE 1

XXXXX is for a single head. A value of 00000 for XXXXX indicates the absolute stop aspect. All other aspects 
represented by the values for XXXXX are determined by the signaling system used and the prototype being 
modeled.

From https://normen.railcommunity.de/RCN-213.pdf:

More information is in RCN-213 about how the address bits are organized.
preamble -0- 1 0 A7 A6 A5 A4 A3 A2 -0- 0 ^A10 ^A9 ^A8 0 A1 A0 1 -0- ....

Thus in byte packet form the format is 10AAAAAA, 0AAA0AA1, 000XXXXX

Die Adresse f�r den ersten erweiterten Zubeh�rdecoder ist wie bei den einfachen
Zubeh�rdecodern die Adresse 4 = 1000-0001 0111-0001 . Diese Adresse wird in
Anwenderdialogen als Adresse 1 dargestellt.

This means that the first address shown to the user as "1" is mapped
to internal address 4.

Note that the Basic accessory format mentions "By convention these
bits (bits 4-6 of the second data byte) are in ones complement" but
this note is absent from the advanced packet description. The
english translation does not mention that the address format for
the advanced packet follows the one for the basic packet but
according to the RCN-213 this is the case.

We allow for addresses from -3 to 2047-3 as that allows to address the
whole range of the 11 bits sent to track.
*/
  if ((address > 2044) || (address < -3)) return false; // 2047-3, 11 bits but offset 3
  if (value != (value & 0x1F)) return false;            // 5 bits

  address+=3;                        // +3 offset according to RCN-213
  byte b[3];
  b[0]= 0x80                         // bits always on
    | ((address>>2) & 0x3F);         // shift out 2, mask out used bits
  b[1]= 0x01                         // bits always on
    | (((~(address>>8)) & 0x07)<<4)  // shift out 8, invert, mask 3 bits, shift up 4
    | ((address & 0x03)<<1);         // mask 2 bits, shift up 1
  b[2]=value;
  DCCQueue::scheduleDCCPacket(b, sizeof(b), repeats);
  return true;
}

//
// writeCVByteMain: Write a byte with PoM on main. This writes
// the 5 byte sized packet to implement this DCC function
//
void DCC::writeCVByteMain(int cab, int cv, byte bValue)  {
  byte b[5];
  byte nB = 0;
  if (cab > HIGHEST_SHORT_ADDR)
    b[nB++] = highByte(cab) | 0xC0;    // convert train number into a two-byte address

  b[nB++] = lowByte(cab);
  b[nB++] = cv1(WRITE_BYTE_MAIN, cv); // any CV>1023 will become modulus(1024) due to bit-mask of 0x03
  b[nB++] = cv2(cv);
  b[nB++] = bValue;

  DCCQueue::scheduleDCCPacket(b, nB, 4);
}

//
// readCVByteMain: Read a byte with PoM on main.
// This requires Railcom active 
//
void DCC::readCVByteMain(int cab, int cv, ACK_CALLBACK callback)  {
  byte b[5];
  byte nB = 0;
  if (cab > HIGHEST_SHORT_ADDR)
    b[nB++] = highByte(cab) | 0xC0;    // convert train number into a two-byte address

  b[nB++] = lowByte(cab);
  b[nB++] = cv1(READ_BYTE_MAIN, cv); // any CV>1023 will become modulus(1024) due to bit-mask of 0x03
  b[nB++] = cv2(cv);
  b[nB++] = 0;

  DCCQueue::scheduleDCCPacket(b, nB, 4);
  Railcom::anticipate(cab,cv,callback);
}

//
// writeCVBitMain: Write a bit of a byte with PoM on main. This writes
// the 5 byte sized packet to implement this DCC function
//
void DCC::writeCVBitMain(int cab, int cv, byte bNum, bool bValue)  {
  byte b[5];
  byte nB = 0;
  bValue = bValue % 2;
  bNum = bNum % 8;

  if (cab > HIGHEST_SHORT_ADDR)
    b[nB++] = highByte(cab) | 0xC0;    // convert train number into a two-byte address

  b[nB++] = lowByte(cab);
  b[nB++] = cv1(WRITE_BIT_MAIN, cv); // any CV>1023 will become modulus(1024) due to bit-mask of 0x03
  b[nB++] = cv2(cv);
  b[nB++] = WRITE_BIT | (bValue ? BIT_ON : BIT_OFF) | bNum;

  DCCQueue::scheduleDCCPacket(b, nB, 4);
}

bool DCC::setTime(uint16_t minutes,uint8_t speed, bool suddenChange) {
  /* see rcn-122
  5 Global commands 
These commands are sent and begin exclusively with a broadcast address 0 
always with {synchronous bits} 0 0000-0000 … and end with the checksum 
... PPPPPPPP 1. Therefore, only the bytes of the commands and not that of 
shown below whole package shown. The commands can be used by vehicle and 
accessory decoders alike. 

  5.1 Time command 
This command is four bytes long and has the format: 
1100-0001 CCxx-xxxx xxxx-xxxxx xxxx-xxxx 
CC indicates what data is transmitted in the packet: 
CC = 00 Model Time 
1100-0001 00MM-MMMM WWWH-HHHH U0BB-BBBB with: 
MMMMMM = Minutes, Value range: 0..59 
WWW =     Day of the Week, Value range: 0 = Monday, 1 = Tuesday, 2 = Wednesday, 
3 = Thursday, 4 = Friday, 5 = Saturday, 6 = Sunday, 7 = Weekday 
is not supported. 
HHHHH =   Hours, value range: 0..23 
U =         
Update, i.e. the time has changed suddenly, e.g. by a new one timetable to start.        
Up to 4 can occur per sudden change commands can be marked like this. 
BBBBBB =    Acceleration factor, value range 0..63. An acceleration factor of 0 means the  
model clock has been stopped, a factor of 1 corresponds to real time, at 2 the 
clock runs twice as fast, at three times as fast as real time, etc. 
*/
if (minutes>=1440 || speed>63 ) return false; 
byte b[5];
b[0]=0; // broadcast address
b[1]=0b11000001; // 1100-0001 (model time)
b[2]=minutes % 60 ;  // MM
b[3]= 0b11100000 | (minutes/60); // 111H-HHHH weekday not supported
b[4]= (suddenChange ? 0b10000000 : 0) | speed;
DCCQueue::scheduleDCCPacket(b, sizeof(b), 2);
return true; 
}

FSH* DCC::getMotorShieldName() {
  return shieldName;
}

const ackOp FLASH WRITE_BIT0_PROG[] = {
     BASELINE,
     W0,WACK,
     V0, WACK,  // validate bit is 0
     ITC1,      // if acked, callback(1)
     CALLFAIL  // callback (-1)
};
const ackOp FLASH WRITE_BIT1_PROG[] = {
     BASELINE,
     W1,WACK,
     V1, WACK,  // validate bit is 1
     ITC1,      // if acked, callback(1)
     CALLFAIL  // callback (-1)
};

const ackOp FLASH VERIFY_BIT0_PROG[] = {
     BASELINE,
     V0, WACK,  // validate bit is 0
     ITC0,      // if acked, callback(0)
     V1, WACK,  // validate bit is 1
     ITC1,
     CALLFAIL  // callback (-1)
};
const ackOp FLASH VERIFY_BIT1_PROG[] = {
     BASELINE,
     V1, WACK,  // validate bit is 1
     ITC1,      // if acked, callback(1)
     V0, WACK,
     ITC0,
     CALLFAIL  // callback (-1)
};

const ackOp FLASH READ_BIT_PROG[] = {
     BASELINE,
     V1, WACK,  // validate bit is 1
     ITC1,      // if acked, callback(1)
     V0, WACK,  // validate bit is zero
     ITC0,      // if acked callback 0
     CALLFAIL       // bit not readable
     };

const ackOp FLASH WRITE_BYTE_PROG[] = {
      BASELINE,
      WB,WACK,ITC1,    // Write and callback(1) if ACK
      // handle decoders that dont ack a write
      VB,WACK,ITC1,    // validate byte and callback(1) if correct
      CALLFAIL        // callback (-1)
      };

const ackOp FLASH VERIFY_BYTE_PROG[] = {
      BASELINE,
      BIV,         // ackManagerByte initial value
      VB,WACK,     // validate byte
      ITCB,       // if ok callback value	
      STARTMERGE,    //clear bit and byte values ready for merge pass
      // each bit is validated against 0 and the result inverted in MERGE
      // this is because there tend to be more zeros in cv values than ones.
      // There is no need for one validation as entire byte is validated at the end
      V0, WACK, MERGE,        // read and merge first tested bit (7)
      ITSKIP,                 // do small excursion if there was no ack
        SETBIT,(ackOp)7,
        V1, WACK, NAKFAIL,    // test if there is an ack on the inverse of this bit (7)
        SETBIT,(ackOp)6,      // and abort whole test if not else continue with bit (6)
      SKIPTARGET,
      V0, WACK, MERGE,        // read and merge second tested bit (6)
      V0, WACK, MERGE,        // read and merge third  tested bit (5) ...
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      VB, WACK, ITCBV,  // verify merged byte and return it if acked ok - with retry report
      CALLFAIL };


const ackOp FLASH READ_CV_PROG[] = {
      BASELINE,
      STARTMERGE,    //clear bit and byte values ready for merge pass
      // each bit is validated against 0 and the result inverted in MERGE
      // this is because there tend to be more zeros in cv values than ones.
      // There is no need for one validation as entire byte is validated at the end
      V0, WACK, MERGE,        // read and merge first tested bit (7)
      ITSKIP,                 // do small excursion if there was no ack
        SETBIT,(ackOp)7,
        V1, WACK, NAKFAIL,    // test if there is an ack on the inverse of this bit (7)
        SETBIT,(ackOp)6,      // and abort whole test if not else continue with bit (6)
      SKIPTARGET,
      V0, WACK, MERGE,        // read and merge second tested bit (6)
      V0, WACK, MERGE,        // read and merge third  tested bit (5) ...
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      VB, WACK, ITCB,  // verify merged byte and return it if acked ok
      CALLFAIL };          // verification failed


const ackOp FLASH LOCO_ID_PROG[] = {
      BASELINE,
      // first check cv20 for extended addressing
      SETCV, (ackOp)20,     // CV 19 is extended
      SETBYTE, (ackOp)0,
      VB, WACK, ITSKIP,     // skip past extended section if cv20 is zero
      // read cv20 and 19 and merge 
      STARTMERGE,           // Setup to read cv 20
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      VB, WACK, NAKSKIP, // bad read of cv20, assume its 0 
      STASHLOCOID,   // keep cv 20 until we have cv19 as well.
      SETCV, (ackOp)19, 
      STARTMERGE,           // Setup to read cv 19
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      VB, WACK, NAKFAIL,  // cant recover if cv 19 unreadable
      COMBINE1920,  // Combile byte with stash and callback
// end of advanced 20,19 check
      SKIPTARGET,
      SETCV, (ackOp)19,     // CV 19 is consist setting
      SETBYTE, (ackOp)0,
      VB, WACK, ITSKIP,     // ignore consist if cv19 is zero (no consist)
      SETBYTE, (ackOp)128,
      VB, WACK, ITSKIP,     // ignore consist if cv19 is 128 (no consist, direction bit set)
      STARTMERGE,           // Setup to read cv 19
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      VB, WACK, ITCB7,  // return 7 bits only, No_ACK means CV19 not supported so ignore it

      SKIPTARGET,     // continue here if CV 19 is zero or fails all validation
      SETCV,(ackOp)29,
      SETBIT,(ackOp)5,
      V0, WACK, ITSKIP,  // Skip to SKIPTARGET if bit 5 of CV29 is zero

      // Long locoid
      SETCV, (ackOp)17,       // CV 17 is part of locoid
      STARTMERGE,
      V0, WACK, MERGE,  // read and merge bit 1 etc
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      VB, WACK, NAKFAIL,  // verify merged byte and return -1 it if not acked ok
      STASHLOCOID,         // keep stashed cv 17 for later
      // Read 2nd part from CV 18
      SETCV, (ackOp)18,
      STARTMERGE,
      V0, WACK, MERGE,  // read and merge bit 1 etc
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      VB, WACK, NAKFAIL,  // verify merged byte and return -1 it if not acked ok
      COMBINELOCOID,        // Combile byte with stash to make long locoid and callback

      // ITSKIP Skips to here if CV 29 bit 5 was zero. so read CV 1 and return that
      SKIPTARGET,
      SETCV, (ackOp)1,
      STARTMERGE,
      SETBIT, (ackOp)6,  // skip over first bit as we know its a zero
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      VB, WACK, ITCB,  // verify merged byte and callback
      CALLFAIL
      };

const ackOp FLASH SHORT_LOCO_ID_PROG[] = {
      BASELINE,
      // Clear consist CV 19,20
      SETCV,(ackOp)20,
      SETBYTE, (ackOp)0,
      WB,WACK,     // ignore dedcoder without cv20 support
      SETCV,(ackOp)19,
      SETBYTE, (ackOp)0,
      WB,WACK,     // ignore dedcoder without cv19 support
      // Turn off long address flag
      SETCV,(ackOp)29,
      SETBIT,(ackOp)5,
      W0,WACK,
      V0,WACK,NAKFAIL,
      SETCV, (ackOp)1,
      SETBYTEL,   // low byte of word
      WB,WACK,ITC1,   // If ACK, we are done - callback(1) means Ok
      VB,WACK,ITC1,   // Some decoders do not ack and need verify
      CALLFAIL
};

// for CONSIST_ID_PROG the 20,19 values are already calculated
const ackOp FLASH CONSIST_ID_PROG[] = {
      BASELINE,
      SETCV,(ackOp)20,
      SETBYTEH,    // high byte to CV 20
      WB,WACK,     // ignore dedcoder without cv20 support
      SETCV,(ackOp)19,
      SETBYTEL,   // low byte of word
      WB,WACK,ITC1,   // If ACK, we are done - callback(1) means Ok
      VB,WACK,ITC1,   // Some decoders do not ack and need verify
      CALLFAIL
};

const ackOp FLASH LONG_LOCO_ID_PROG[] = {
      BASELINE,
      // Clear consist CV 19,20
      SETCV,(ackOp)20,
      SETBYTE, (ackOp)0,
      WB,WACK,     // ignore dedcoder without cv20 support
      SETCV,(ackOp)19,
      SETBYTE, (ackOp)0,
      WB,WACK,     // ignore decoder without cv19 support
      // Turn on long address flag cv29 bit 5
      SETCV,(ackOp)29,
      SETBIT,(ackOp)5,
      W1,WACK,
      V1,WACK,NAKFAIL,
      // Store high byte of address in cv 17
      SETCV, (ackOp)17,
      SETBYTEH, // high byte of word
      WB,WACK,      // do write
      ITSKIP,       // if ACK, jump to SKIPTARGET
        VB,WACK,    // try verify instead
        ITSKIP,     // if ACK, jump to SKIPTARGET
          CALLFAIL, // if still here, fail
      SKIPTARGET,
      // store
      SETCV, (ackOp)18,
      SETBYTEL,   // low byte of word
      WB,WACK,ITC1,   // If ACK, we are done - callback(1) means Ok
      VB,WACK,ITC1,   // Some decoders do not ack and need verify
      CALLFAIL
};

void  DCC::writeCVByte(int16_t cv, byte byteValue, ACK_CALLBACK callback)  {
  DCCACK::Setup(cv, byteValue,  WRITE_BYTE_PROG, callback);
}

void DCC::writeCVBit(int16_t cv, byte bitNum, bool bitValue, ACK_CALLBACK callback)  {
  if (bitNum >= 8) callback(-1);
  else DCCACK::Setup(cv, bitNum, bitValue?WRITE_BIT1_PROG:WRITE_BIT0_PROG, callback);
}

void  DCC::verifyCVByte(int16_t cv, byte byteValue, ACK_CALLBACK callback)  {
  DCCACK::Setup(cv, byteValue,  VERIFY_BYTE_PROG, callback);
}

void DCC::verifyCVBit(int16_t cv, byte bitNum, bool bitValue, ACK_CALLBACK callback)  {
  if (bitNum >= 8) callback(-1);
  else DCCACK::Setup(cv, bitNum, bitValue?VERIFY_BIT1_PROG:VERIFY_BIT0_PROG, callback);
}


void DCC::readCVBit(int16_t cv, byte bitNum, ACK_CALLBACK callback)  {
  if (bitNum >= 8) callback(-1);
  else DCCACK::Setup(cv, bitNum,READ_BIT_PROG, callback);
}

void DCC::readCV(int16_t cv, ACK_CALLBACK callback)  {
  DCCACK::Setup(cv, 0,READ_CV_PROG, callback);
}

void DCC::getLocoId(ACK_CALLBACK callback) {
  DCCACK::Setup(0,0, LOCO_ID_PROG, callback);
}

void DCC::setLocoId(int id,ACK_CALLBACK callback) {
  if (id<1 || id>10239) { //0x27FF according to standard
    callback(-1);
    return;
  }
  if (id<=HIGHEST_SHORT_ADDR)
      DCCACK::Setup(id, SHORT_LOCO_ID_PROG, callback);
  else
      DCCACK::Setup(id | 0xc000,LONG_LOCO_ID_PROG, callback);
}

void DCC::setConsistId(int id,bool reverse,ACK_CALLBACK callback) {
  if (id<0 || id>10239) { //0x27FF according to standard
    callback(-1);
    return;
  }
  byte cv20;
  byte cv19;

  if (id<=HIGHEST_SHORT_ADDR) {
    cv19=id;
    cv20=0;
  }
  else {
    cv20=id/100;
    cv19=id%100;
  }
  if (reverse) cv19|=0x80;
  DCCACK::Setup((cv20<<8)|cv19, CONSIST_ID_PROG, callback);
}

void DCC::forgetLoco(int cab) {  // removes any speed reminders for this loco
  setThrottle2(cab,1); // ESTOP this loco if still on track
  auto slot=lookupSpeedTable(cab, false);
  if (slot) {
    slot->loco=-1;  // no longer used but not end of world
    CommandDistributor::broadcastForgetLoco(cab);
  }
}
void DCC::forgetAllLocos() {  // removes all speed reminders
  setThrottle2(0,1); // ESTOP all locos still on track
  for (int i=0;i<MAX_LOCOS;i++) {
    if (speedTable[i].loco) CommandDistributor::broadcastForgetLoco(speedTable[i].loco);
    speedTable[i].loco=0;  // no longer used and looks like end
  }
}

byte DCC::loopStatus=0;

void DCC::loop()  {
  TrackManager::loop(); // power overload checks
  if (DCCWaveform::mainTrack.isReminderWindowOpen()) {
    // Now is a good time to choose a packet to be sent
    // Either highest priority from the queues or a reminder
    if (!DCCQueue::scheduleNext()) {
      issueReminders();
      DCCQueue::scheduleNext(); // push through any just created reminder
    }
  }
}

void DCC::issueReminders() {
  // Move to next loco slot.  If occupied, send a reminder.
  auto slot = nextLocoReminder;
  if (slot >= &speedTable[MAX_LOCOS]) slot=&speedTable[0];  // Go to start of table
  if (slot->loco > 0) 
    if (!issueReminder(slot)) 
      return;
  // a loco=0 is at the end of the list, a loco <0 is deleted
  if (slot->loco==0) nextLocoReminder = &speedTable[0];
  else nextLocoReminder=slot+1;
}

int16_t normalize(byte speed) {
   if (speed & 0x80) return speed & 0x7F;
   return 0-1-speed; 
}
byte dccalize(int16_t speed) {
   if (speed>127) return 0xFF;  // 127 forward
   if (speed<-127) return 0x7F;  // 127 reverse
   if (speed >=0) return speed | 0x80;
   // negative speeds... -1==dcc 0, -2==dcc 1 
   return (int16_t)-1 - speed; 
}

bool DCC::issueReminder(LOCO * slot) {
  unsigned long functions=slot->functions;
  int loco=slot->loco;
  byte flags=slot->groupFlags;

  switch (loopStatus) {
        case 0: {
         // calculate any momentum change going on
         auto sc=slot->speedCode;
         if (slot->targetSpeed!=sc) {
            // calculate new speed code 
            auto now=millis();
            int16_t delay=now-slot->momentum_base;
            auto millisPerNotch=MOMENTUM_FACTOR * (int16_t)getMomentum(slot);
            // allow for momentum change to 0 while accelerating/slowing
            auto ticks=(millisPerNotch>0)?(delay/millisPerNotch):500;
            if (ticks>0) {
              auto current=normalize(sc);  // -128..+127
              auto target=normalize(slot->targetSpeed);
              // DIAG(F("Momentum l=%d ti=%d sc=%d c=%d t=%d"),loco,ticks,sc,current,target);
              if (current<target) { // accelerate
                current+=ticks;
                if (current>target) current=target;
              }
              else  { // slow
                current-=ticks;
                if (current<target) current=target;
              }
              sc=dccalize(current);
              //DIAG(F("c=%d newsc=%d"),current,sc);
              slot->speedCode=sc;
              TrackManager::setDCSignal(loco,sc); // in case this is a dcc track on this addr
              slot->momentum_base=now;  
            }
          }
          // DIAG(F("Reminder %d speed %d"),loco,slot->speedCode);
          setThrottle2(loco, sc);
        }
        break;
       case 1: // remind function group 1 (F0-F4)
          if (flags & FN_GROUP_1)
#ifndef DISABLE_FUNCTION_REMINDERS
	    setFunctionInternal(loco,0, 128 | ((functions>>1)& 0x0F) | ((functions & 0x01)<<4),0); // 100D DDDD
#else
	    setFunctionInternal(loco,0, 128 | ((functions>>1)& 0x0F) | ((functions & 0x01)<<4),2);
          flags&= ~FN_GROUP_1;  // dont send them again
#endif
          break;
       case 2: // remind function group 2 F5-F8
          if (flags & FN_GROUP_2)
#ifndef DISABLE_FUNCTION_REMINDERS
  	    setFunctionInternal(loco,0, 176 | ((functions>>5)& 0x0F),0);                           // 1011 DDDD
#else
	    setFunctionInternal(loco,0, 176 | ((functions>>5)& 0x0F),2);
          flags&= ~FN_GROUP_2;  // dont send them again
#endif
          break;
       case 3: // remind function group 3 F9-F12
          if (flags & FN_GROUP_3)
#ifndef DISABLE_FUNCTION_REMINDERS
	    setFunctionInternal(loco,0, 160 | ((functions>>9)& 0x0F),0);                           // 1010 DDDD
#else
	    setFunctionInternal(loco,0, 160 | ((functions>>9)& 0x0F),2);
          flags&= ~FN_GROUP_3;  // dont send them again
#endif
          break;
       case 4: // remind function group 4 F13-F20
          if (flags & FN_GROUP_4)
	    setFunctionInternal(loco,222, ((functions>>13)& 0xFF),2);
          flags&= ~FN_GROUP_4;  // dont send them again
          break;
       case 5: // remind function group 5 F21-F28
          if (flags & FN_GROUP_5)
	    setFunctionInternal(loco,223, ((functions>>21)& 0xFF),2);
          flags&= ~FN_GROUP_5;  // dont send them again
          break;
      }
      loopStatus++;
      // if we reach status 6 then this loco is done so
      // reset status to 0 for next loco and return true so caller
      // moves on to next loco.
      if (loopStatus>5) loopStatus=0;
      return loopStatus==0;
    }




///// Private helper functions below here /////////////////////

byte DCC::cv1(byte opcode, int cv)  {
  cv--;
  return (highByte(cv) & (byte)0x03) | opcode;
}
byte DCC::cv2(int cv)  {
  cv--;
  return lowByte(cv);
}

DCC::LOCO *  DCC::lookupSpeedTable(int locoId, bool autoCreate) {
  // determine speed reg for this loco
  LOCO * firstEmpty=nullptr;
  SLOTLOOP {
    if (firstEmpty==nullptr && slot->loco<=0) firstEmpty=slot;
    if (slot->loco == locoId) return slot;
    if (slot->loco==0) break; 
  }
  if (!autoCreate) return nullptr;
  if (firstEmpty==nullptr) { 
    // return last slot if full
    DIAG(F("Too many locos, reusing last slot"));
    firstEmpty=&speedTable[MAX_LOCOS-1];
  }
  // fill first empty slot with new entry
  firstEmpty->loco = locoId;
  firstEmpty->speedCode=128;  // default direction forward
  firstEmpty->targetSpeed=128;  // default direction forward
  firstEmpty->groupFlags=0;
  firstEmpty->functions=0;
  firstEmpty->momentumA=MOMENTUM_USE_DEFAULT; 
  firstEmpty->momentumD=MOMENTUM_USE_DEFAULT; 
  return firstEmpty;
}

bool DCC::setMomentum(int locoId,int16_t accelerating, int16_t decelerating) {
  if (locoId<0) return false;
  if (locoId==0) {
    if (accelerating<0 || decelerating<0) return false;
    defaultMomentumA=accelerating/MOMENTUM_FACTOR;
    defaultMomentumD=decelerating/MOMENTUM_FACTOR;
    return true;
  }
  // -1 is ok and means this loco should use the default.
  if (accelerating<-1 || decelerating<-1) return false;
  if (accelerating/MOMENTUM_FACTOR >= MOMENTUM_USE_DEFAULT || 
      decelerating/MOMENTUM_FACTOR >= MOMENTUM_USE_DEFAULT) return false;
  
  // Values stored are 255=MOMENTUM_USE_DEFAULT, or millis/MOMENTUM_FACTOR.
  // This is to keep the values in a byte rather than int16
  // thus saving 2 bytes RAM per loco slot.   
  LOCO* slot=lookupSpeedTable(locoId,true);
  slot->momentumA=(accelerating<0)? MOMENTUM_USE_DEFAULT: (accelerating/MOMENTUM_FACTOR);
  slot->momentumD=(decelerating<0)? MOMENTUM_USE_DEFAULT: (decelerating/MOMENTUM_FACTOR);
  return true; 
}


void  DCC::estopAll() {
  setThrottle2(0,1); // estop all locos
  TrackManager::setDCSignal(0,1); 
    
  // remind stop/estop but dont change direction
  SLOTLOOP {
    if (slot->loco<=0) continue;
    byte newspeed=(slot->targetSpeed & 0x80) |  0x01;
    slot->speedCode = newspeed;
    slot->targetSpeed = newspeed;
    CommandDistributor::broadcastLoco(slot);  
  }
}


DCC::LOCO DCC::speedTable[MAX_LOCOS];
DCC::LOCO *  DCC::nextLocoReminder = &DCC::speedTable[0];


void DCC::displayCabList(Print * stream) {
    StringFormatter::send(stream,F("<*\n")); 
    int used=0;
    SLOTLOOP {
       if (slot->loco==0) break;  // no more locos
       if (slot->loco>0) {
        used ++;
        StringFormatter::send(stream,F("cab=%d, speed=%d, target=%d, momentum=%d/%d, block=%d\n"),
           slot->loco,  slot->speedCode, slot->targetSpeed,
           slot->momentumA, slot->momentumD, slot->blockOccupied);
       }
     }
     StringFormatter::send(stream,F("Used=%d, max=%d, momentum=%d/%d *>\n"),
                            used,MAX_LOCOS, DCC::defaultMomentumA,DCC::defaultMomentumD);
}

void DCC::setLocoInBlock(int loco, uint16_t blockid, bool exclusive) {
  // update block loco is in, tell exrail leaving old block, and entering new.

  // NOTE: The loco table scanning is really inefficient and needs rewriting
  //   This was done once in the momentum poc.  
  #ifdef EXRAIL_ACTIVE
  auto slot=lookupSpeedTable(loco,true);
  if (!slot) return;
  auto oldBlock=slot->blockOccupied; 
  if (oldBlock==blockid) return; 
  if (oldBlock) RMFT2::blockEvent(oldBlock,loco,false);
  slot->blockOccupied=blockid;
  if (blockid) RMFT2::blockEvent(blockid,loco,true);

  if (exclusive) {
    SLOTLOOP {
       if (slot->loco==0) break;  // no more locos
       if (slot->loco>0) {
          if (slot->loco!=loco &&  slot->blockOccupied==blockid) {
            RMFT2::blockEvent(blockid,slot->loco,false);
            slot->blockOccupied=0;
          }
      }
    }
  }
  #endif 
}

void DCC::clearBlock(uint16_t blockid) {
  // Railcom reports block empty... tell Exrail about all leavers 
  #ifdef EXRAIL_ACTIVE
  SLOTLOOP {
       if (slot->loco==0) break;  // no more locos
       if (slot->loco>0) {
        if (slot->blockOccupied==blockid) {
        RMFT2::blockEvent(blockid,slot->loco,false);
        slot->blockOccupied=0;
        }
      }
  }
  #endif 
}
