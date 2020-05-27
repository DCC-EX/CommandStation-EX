#include "DCC.h"
#include "DCCWaveform.h"
#include "DIAG.h"

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

void DCC::begin() {
  DCCWaveform::begin();
}

void DCC::setThrottle( uint16_t cab, uint8_t tSpeed, bool tDirection)  {
  setThrottle2(cab, tSpeed, tDirection);
  // retain speed for loco reminders
  updateLocoReminder(cab, tSpeed, tDirection );
}

void DCC::setThrottle2( uint16_t cab, uint8_t tSpeed, bool tDirection)  {
  uint8_t b[5];
  uint8_t nB = 0;

  if (cab > 127)
    b[nB++] = highByte(cab) | 0xC0;    // convert train number into a two-byte address

  b[nB++] = lowByte(cab);
  b[nB++] = 0x3F;                      // 128-step speed control byte
  if (tSpeed > 0)
    b[nB++] = tSpeed + (tSpeed > 0) + tDirection * 128; // max speed is 126, but speed codes range from 2-127 (0=stop, 1=emergency stop)
  else {
    b[nB++] = 1;
    tSpeed = 0;
  }

  DCCWaveform::mainTrack.schedulePacket(b, nB, 0);
}

void DCC::setFunction(int cab, byte byte1)  {
  uint8_t b[4];
  uint8_t nB = 0;

  if (cab > 127)
    b[nB++] = highByte(cab) | 0xC0;    // convert train number into a two-byte address
  b[nB++] = lowByte(cab);
  b[nB++] = (byte1 | 0x80) & 0xBF;

  DCCWaveform::mainTrack.schedulePacket(b, nB, 4);     // Repeat the packet four times
}

void DCC::setFunction(int cab, byte byte1, byte byte2)  {
  byte b[5];
  byte nB = 0;

  if (cab > 127)
    b[nB++] = highByte(cab) | 0xC0;    // convert train number into a two-byte address
  b[nB++] = lowByte(cab);
  b[nB++] = (byte1 | 0xDE) & 0xDF;   // for safety this guarantees that first byte will either be 0xDE (for F13-F20) or 0xDF (for F21-F28)
  b[nB++] = byte2;

  DCCWaveform::mainTrack.schedulePacket(b, nB, 4);     // Repeat the packet four times
}

void DCC::setAccessory(int address, byte number, bool activate) {
  byte b[3];                      // save space for checksum byte

  b[0] = address % 64 + 128;                                     // first byte is of the form 10AAAAAA, where AAAAAA represent 6 least signifcant bits of accessory address
  b[1] = ((((address / 64) % 8) << 4) + (number % 4 << 1) + activate % 2) ^ 0xF8; // second byte is of the form 1AAACDDD, where C should be 1, and the least significant D represent activate/deactivate

  DCCWaveform::mainTrack.schedulePacket(b, 2, 4);      // Repeat the packet four times
}

void DCC::writeCVByteMain(int cab, int cv, byte bValue)  {
  byte b[6];                      // save space for checksum byte
  byte nB = 0;
  if (cab > 127)
    b[nB++] = highByte(cab) | 0xC0;    // convert train number into a two-byte address

  b[nB++] = lowByte(cab);
  b[nB++] = cv1(0xEC, cv); // any CV>1023 will become modulus(1024) due to bit-mask of 0x03
  b[nB++] = cv2(cv);
  b[nB++] = bValue;

  DCCWaveform::mainTrack.schedulePacket(b, nB, 4);
}

void DCC::writeCVBitMain(int cab, int cv, byte bNum, bool bValue)  {
  byte b[6];                      // save space for checksum byte
  byte nB = 0;
  bValue = bValue % 2;
  bNum = bNum % 8;

  if (cab > 127)
    b[nB++] = highByte(cab) | 0xC0;    // convert train number into a two-byte address

  b[nB++] = lowByte(cab);
  b[nB++] = cv1(0xE8, cv); // any CV>1023 will become modulus(1024) due to bit-mask of 0x03
  b[nB++] = cv2(cv);
  b[nB++] = 0xF0 + bValue * 8 + bNum;

  DCCWaveform::mainTrack.schedulePacket(b, nB, 4);
}

bool  DCC::writeCVByte(int cv, byte bValue)  {
  uint8_t message[] = {cv1(0x7C, cv), cv2(cv), bValue};
  DCCWaveform::progTrack.schedulePacket(message, sizeof(message), 6);           // NMRA recommends 6 write or reset packets for decoder recovery time
  return verifyCV(cv, bValue);
}


bool DCC::writeCVBit(int cv, byte bNum, bool bValue)  {
  bValue = bValue % 2;
  bNum = bNum % 8;
  uint8_t message[] = {cv1(0x78, cv), cv2(cv), 0xF0 + bValue * 8 + bNum};
  DCCWaveform::progTrack.schedulePacket(message, sizeof(message), 6);           // NMRA recommends 6 write or reset packets for decoder recovery time

  /* TODO... what is the verify opcode here?
    bitWrite(message[2],4,1);              // change instruction code from Write Bit to Verify Bit
    DCCWaveform::progTrack.schedulePacket(message,sizeof(message),6);             // NMRA recommends 6 write or reset packets for decoder recovery time
  */
 return true; // <<<< NOT  ACCURATE... see comment above 
}


int DCC::readCV(int cv)  {
  byte message[] = {  cv1(0x78, cv)  , // any CV>1023 will become modulus(1024) due to bit-mask of 0x03
                      cv2(cv),
                      0
                   };    // trailing zero will be updated in loop below

  byte value = 0;

  // get each bit individually
  for (int i = 0; i < 8; i++) {
    message[2] = 0xE8 + i;
    bool one=DCCWaveform::progTrack.schedulePacketWithAck(message, sizeof(message), 4);     // NMRA recommends 5 read packets
    value += one << i;
  }

  return verifyCV(cv, value) ? value : -1;
}

void DCC::loop()  {
  DCCWaveform::loop(); // powwer overload checks
  // if the main track transmitter still has a pending packet, skip this loop.
  if ( DCCWaveform::mainTrack.packetPending) return;

  // each time around the Arduino loop, we resend a loco speed packet reminder
  for (; nextLoco < MAX_LOCOS; nextLoco++) {
    if (speedTable[nextLoco].loco > 0) {
      setThrottle2(speedTable[nextLoco].loco, speedTable[nextLoco].speed, speedTable[nextLoco].forward);
      nextLoco++;
      return;
    }
  }
  for (nextLoco = 0; nextLoco < MAX_LOCOS; nextLoco++) {
    if (speedTable[nextLoco].loco > 0) {
      setThrottle2(speedTable[nextLoco].loco, speedTable[nextLoco].speed, speedTable[nextLoco].forward);
      nextLoco++;
      return;
    }
  }
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

bool DCC::verifyCV(int cv, byte value) {
  byte message[] = { cv1(0x74, cv), cv2(cv), value};
  DIAG(F("\n\nVerifying cv %d = %d"), cv, value);
  return DCCWaveform::progTrack.schedulePacketWithAck(message, sizeof(message), 5);
  }

void  DCC::updateLocoReminder(int loco, byte tSpeed, bool forward) {
  // determine speed reg for this loco
  int reg;
  int firstEmpty = MAX_LOCOS;
  for (reg = 0; reg < MAX_LOCOS; reg++) {
    if (speedTable[reg].loco == loco) break;
    if (speedTable[reg].loco == 0 && firstEmpty == MAX_LOCOS) firstEmpty = reg;
  }
  if (reg == MAX_LOCOS) reg = firstEmpty;
  if (reg >= MAX_LOCOS) {
    DIAG(F("\nToo many locos\n"));
    return;
  }
  speedTable[reg].loco = loco;
  speedTable[reg].speed = tSpeed;
  speedTable[reg].forward = forward;
}
DCC::LOCO DCC::speedTable[MAX_LOCOS];
int DCC::nextLoco = 0;
