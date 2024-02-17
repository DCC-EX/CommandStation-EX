/*
 *  © 2021 Neil McKechnie
 *  © 2021 Mike S
 *  © 2021 Fred Decker
 *  © 2021 Herb Morton
 *  © 2020-2022 Harald Barth
 *  © 2020-2021 M Steve Todd
 *  © 2020-2021 Chris Harlow
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


void DCC::setThrottle( uint16_t cab, uint8_t tSpeed, bool tDirection)  {
  byte speedCode = (tSpeed & 0x7F)  + tDirection * 128;
  setThrottle2(cab, speedCode);
  TrackManager::setDCSignal(cab,speedCode); // in case this is a dcc track on this addr
  // retain speed for loco reminders
  updateLocoReminder(cab, speedCode );
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

  DCCWaveform::mainTrack.schedulePacket(b, nB, 0);
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

  DCCWaveform::mainTrack.schedulePacket(b, nB, count);
}

// returns speed steps 0 to 127 (1 == emergency stop)
// or -1 on "loco not found"
int8_t DCC::getThrottleSpeed(int cab) {
  int reg=lookupSpeedTable(cab);
  if (reg<0) return -1;
  return speedTable[reg].speedCode & 0x7F;
}

// returns speed code byte
// or 128 (speed 0, dir forward) on "loco not found".
uint8_t DCC::getThrottleSpeedByte(int cab) {
  int reg=lookupSpeedTable(cab);
  if (reg<0)
    return 128;
  return speedTable[reg].speedCode;
}

// returns direction on loco
// or true/forward on "loco not found"
bool DCC::getThrottleDirection(int cab) {
  int reg=lookupSpeedTable(cab);
  if (reg<0) return true;
  return (speedTable[reg].speedCode & 0x80) !=0;
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
    DCCWaveform::mainTrack.schedulePacket(b, nB, 4);
    return true;
  }

  int reg = lookupSpeedTable(cab);
  if (reg<0) return false;

  // Take care of functions:
  // Set state of function
  unsigned long previous=speedTable[reg].functions;
  unsigned long funcmask = (1UL<<functionNumber);
  if (on) {
      speedTable[reg].functions |= funcmask;
  } else {
      speedTable[reg].functions &= ~funcmask;
  }
  if (speedTable[reg].functions != previous) {
    updateGroupflags(speedTable[reg].groupFlags, functionNumber);
    CommandDistributor::broadcastLoco(reg);
  }
  return true;
}

// Flip function state
void DCC::changeFn( int cab, int16_t functionNumber) {
  if (cab<=0 || functionNumber>28) return;
  int reg = lookupSpeedTable(cab);
  if (reg<0) return;
  unsigned long funcmask = (1UL<<functionNumber);
  speedTable[reg].functions ^= funcmask;
  updateGroupflags(speedTable[reg].groupFlags, functionNumber);
  CommandDistributor::broadcastLoco(reg);
}

int DCC::getFn( int cab, int16_t functionNumber) {
  if (cab<=0 || functionNumber>28) return -1;  // unknown
  int reg = lookupSpeedTable(cab);
  if (reg<0) return -1;

  unsigned long funcmask = (1UL<<functionNumber);
  return  (speedTable[reg].functions & funcmask)? 1 : 0;
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
  int reg = lookupSpeedTable(cab);
  return (reg<0)?0:speedTable[reg].functions;
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
  if (onoff != 0) {
    DCCWaveform::mainTrack.schedulePacket(b, 2, 3);      // Repeat on packet three times
#if defined(EXRAIL_ACTIVE)
    RMFT2::activateEvent(address<<2|port,gate);
#endif
  }
  if (onoff != 1) {
    b[1] &= ~0x08; // set C to 0
    DCCWaveform::mainTrack.schedulePacket(b, 2, 3);      // Repeat off packet three times
  }
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
  DCCWaveform::mainTrack.schedulePacket(b, sizeof(b), repeats);
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

  DCCWaveform::mainTrack.schedulePacket(b, nB, 4);
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

  DCCWaveform::mainTrack.schedulePacket(b, nB, 4);
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

const ackOp FLASH LONG_LOCO_ID_PROG[] = {
      BASELINE,
      // Clear consist CV 19
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

void DCC::forgetLoco(int cab) {  // removes any speed reminders for this loco
  setThrottle2(cab,1); // ESTOP this loco if still on track
  int reg=lookupSpeedTable(cab, false);
  if (reg>=0) {
    speedTable[reg].loco=0;
    setThrottle2(cab,1); // ESTOP if this loco still on track
  }
}
void DCC::forgetAllLocos() {  // removes all speed reminders
  setThrottle2(0,1); // ESTOP all locos still on track
  for (int i=0;i<MAX_LOCOS;i++) speedTable[i].loco=0;
}

byte DCC::loopStatus=0;

void DCC::loop()  {
  TrackManager::loop(); // power overload checks
  issueReminders();
}

void DCC::issueReminders() {
  // if the main track transmitter still has a pending packet, skip this time around.
  if (!DCCWaveform::mainTrack.isReminderWindowOpen()) return;
  // Move to next loco slot.  If occupied, send a reminder.
  int reg = lastLocoReminder+1;
  if (reg > highestUsedReg) reg = 0;  // Go to start of table
  if (speedTable[reg].loco > 0) {
    // have found loco to remind
    if (issueReminder(reg))
      lastLocoReminder = reg;
  } else
    lastLocoReminder = reg;
}

bool DCC::issueReminder(int reg) {
  unsigned long functions=speedTable[reg].functions;
  int loco=speedTable[reg].loco;
  byte flags=speedTable[reg].groupFlags;

  switch (loopStatus) {
        case 0:
      //   DIAG(F("Reminder %d speed %d"),loco,speedTable[reg].speedCode);
         setThrottle2(loco, speedTable[reg].speedCode);
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

int DCC::lookupSpeedTable(int locoId, bool autoCreate) {
  // determine speed reg for this loco
  int firstEmpty = MAX_LOCOS;
  int reg;
  for (reg = 0; reg < MAX_LOCOS; reg++) {
    if (speedTable[reg].loco == locoId) break;
    if (speedTable[reg].loco == 0 && firstEmpty == MAX_LOCOS) firstEmpty = reg;
  }

  // return -1 if not found and not auto creating
  if (reg== MAX_LOCOS && !autoCreate) return -1; 
  if (reg == MAX_LOCOS) reg = firstEmpty;
  if (reg >= MAX_LOCOS) {
    DIAG(F("Too many locos"));
    return -1;
  }
  if (reg==firstEmpty){
        speedTable[reg].loco = locoId;
        speedTable[reg].speedCode=128;  // default direction forward
        speedTable[reg].groupFlags=0;
        speedTable[reg].functions=0;
  }
  if (reg > highestUsedReg) highestUsedReg = reg;
  return reg;
}

void  DCC::updateLocoReminder(int loco, byte speedCode) {

  if (loco==0) {
     // broadcast stop/estop but dont change direction
     for (int reg = 0; reg <= highestUsedReg; reg++) {
       if (speedTable[reg].loco==0) continue;
       byte newspeed=(speedTable[reg].speedCode & 0x80) |  (speedCode & 0x7f);
       if (speedTable[reg].speedCode != newspeed) {
         speedTable[reg].speedCode = newspeed;
         CommandDistributor::broadcastLoco(reg);
       }
     }
     return;
  }

  // determine speed reg for this loco
  int reg=lookupSpeedTable(loco);
  if (reg>=0 && speedTable[reg].speedCode!=speedCode) {
    speedTable[reg].speedCode = speedCode;
    CommandDistributor::broadcastLoco(reg);
  }
}

DCC::LOCO DCC::speedTable[MAX_LOCOS];
int DCC::lastLocoReminder = 0;
int DCC::highestUsedReg = 0;


void DCC::displayCabList(Print * stream) {

    int used=0;
    for (int reg = 0; reg <= highestUsedReg; reg++) {
       if (speedTable[reg].loco>0) {
        used ++;
        StringFormatter::send(stream,F("cab=%d, speed=%d, dir=%c \n"),
           speedTable[reg].loco,  speedTable[reg].speedCode & 0x7f,(speedTable[reg].speedCode & 0x80) ? 'F':'R');
       }
     }
     StringFormatter::send(stream,F("Used=%d, max=%d\n"),used,MAX_LOCOS);

}
