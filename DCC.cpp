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
  byte speedCode= tSpeed + (tSpeed > 0) + tDirection * 128; // max speed is 126, but speed codes range from 2-127 (0=stop, 1=emergency stop)
  setThrottle2(cab, speedCode);
  // retain speed for loco reminders
  updateLocoReminder(cab, speedCode );
}

void DCC::setThrottle2( uint16_t cab, byte speedCode)  {
 
  uint8_t b[4];
  uint8_t nB = 0;

  if (cab > 127)
    b[nB++] = highByte(cab) | 0xC0;    // convert train number into a two-byte address
  b[nB++] = lowByte(cab);
  b[nB++] = SET_SPEED;                      // 128-step speed control byte
  b[nB++] = speedCode; // for encoding see setThrottle
 
  DCCWaveform::mainTrack.schedulePacket(b, nB, 0);
}

void DCC::setFunction(int cab, byte byte1)  {
  uint8_t b[3];
  uint8_t nB = 0;

  if (cab > 127)
    b[nB++] = highByte(cab) | 0xC0;    // convert train number into a two-byte address
  b[nB++] = lowByte(cab);
  b[nB++] = (byte1 | 0x80) & 0xBF;

  DCCWaveform::mainTrack.schedulePacket(b, nB, 4);     // Repeat the packet four times
}

void DCC::setFunction(int cab, byte byte1, byte byte2)  {
  byte b[4];
  byte nB = 0;

  if (cab > 127)
    b[nB++] = highByte(cab) | 0xC0;    // convert train number into a two-byte address
  b[nB++] = lowByte(cab);
  b[nB++] = (byte1 | 0xDE) & 0xDF;   // for safety this guarantees that first byte will either be 0xDE (for F13-F20) or 0xDF (for F21-F28)
  b[nB++] = byte2;

  DCCWaveform::mainTrack.schedulePacket(b, nB, 4);     // Repeat the packet four times
}

void DCC::setAccessory(int address, byte number, bool activate) {
  byte b[2];                    

  b[0] = address % 64 + 128;                                     // first byte is of the form 10AAAAAA, where AAAAAA represent 6 least signifcant bits of accessory address
  b[1] = ((((address / 64) % 8) << 4) + (number % 4 << 1) + activate % 2) ^ 0xF8; // second byte is of the form 1AAACDDD, where C should be 1, and the least significant D represent activate/deactivate

  DCCWaveform::mainTrack.schedulePacket(b, 2, 4);      // Repeat the packet four times
}

void DCC::writeCVByteMain(int cab, int cv, byte bValue)  {
  byte b[5];                     
  byte nB = 0;
  if (cab > 127)
    b[nB++] = highByte(cab) | 0xC0;    // convert train number into a two-byte address

  b[nB++] = lowByte(cab);
  b[nB++] = cv1(WRITE_BYTE_MAIN, cv); // any CV>1023 will become modulus(1024) due to bit-mask of 0x03
  b[nB++] = cv2(cv);
  b[nB++] = bValue;

  DCCWaveform::mainTrack.schedulePacket(b, nB, 4);
}

void DCC::writeCVBitMain(int cab, int cv, byte bNum, bool bValue)  {
  byte b[5];                      
  byte nB = 0;
  bValue = bValue % 2;
  bNum = bNum % 8;

  if (cab > 127)
    b[nB++] = highByte(cab) | 0xC0;    // convert train number into a two-byte address

  b[nB++] = lowByte(cab);
  b[nB++] = cv1(WRITE_BIT_MAIN, cv); // any CV>1023 will become modulus(1024) due to bit-mask of 0x03
  b[nB++] = cv2(cv);
  b[nB++] = WRITE_BIT | (bValue?BIT_ON:BIT_OFF) | bNum;

  DCCWaveform::mainTrack.schedulePacket(b, nB, 4);
}

bool  DCC::writeCVByte(int cv, byte bValue)  {
  uint8_t message[] = {cv1(WRITE_BYTE, cv), cv2(cv), bValue};
  DCCWaveform::progTrack.schedulePacket(message, sizeof(message), 6);           // NMRA recommends 6 write or reset packets for decoder recovery time
  return verifyCVByte(cv, bValue);
}

bool DCC::verifyCVByte(int cv, byte value) {
  byte message[] = { cv1(VERIFY_BYTE, cv), cv2(cv), value};
  return DCCWaveform::progTrack.schedulePacketWithAck(message, sizeof(message), 5);
  }

bool DCC::writeCVBit(int cv, byte bNum, bool bValue)  {
  if (bNum>=8) return false;
  byte instruction=WRITE_BIT | bValue?BIT_ON:BIT_OFF | bNum;
  byte message[] = {cv1(BIT_MANIPULATE, cv), cv2(cv), instruction };
  DCCWaveform::progTrack.schedulePacket(message, sizeof(message), 6);           // NMRA recommends 6 write or reset packets for decoder recovery time 
  return verifyCVBit(cv, bNum, bValue); 
}

bool DCC::verifyCVBit(int cv, byte bNum, bool bValue)  {
  if (bNum>=8) return false;
  byte instruction=VERIFY_BIT | bValue?BIT_ON:BIT_OFF | bNum;
  byte message[] = {cv1(BIT_MANIPULATE, cv), cv2(cv), instruction };
  return DCCWaveform::progTrack.schedulePacketWithAck(message, sizeof(message), 5);           // NMRA recommends 6 write or reset packets for decoder recovery time
}

int DCC::readCVBit(int cv, byte bNum)  {
  if (bNum>=8) return -1;
   if (verifyCVBit(cv, bNum,true)) return 1; 
   // failed verify might be a zero, or an error so must check again  
   if (verifyCVBit(cv, bNum,false)) return 0; 
   return -1;
}


int DCC::readCV(int cv)  {
  byte value = 0;
  // get each bit individually by validating against a one. 
  for (int bNum = 0; bNum < 8; bNum++) {
    value += verifyCVBit(cv,bNum,true) << bNum;
  }
  return verifyCVByte(cv, value) ? value : -1;
}

  
void DCC::loop()  {
  DCCWaveform::loop(); // powwer overload checks
  // if the main track transmitter still has a pending packet, skip this loop.
  if ( DCCWaveform::mainTrack.packetPending) return;

  // each time around the Arduino loop, we resend a loco speed packet reminder
  for (; nextLoco < MAX_LOCOS; nextLoco++) {
    if (speedTable[nextLoco].loco > 0) {
      setThrottle2(speedTable[nextLoco].loco, speedTable[nextLoco].speedCode);
      nextLoco++;
      return;
    }
  }
  for (nextLoco = 0; nextLoco < MAX_LOCOS; nextLoco++) {
    if (speedTable[nextLoco].loco > 0) {
      setThrottle2(speedTable[nextLoco].loco, speedTable[nextLoco].speedCode);
      nextLoco++;
      return;
    }
  }
}

int DCC::getLocoId() {
  switch (readCVBit(29,5)) {
    case 1:  
           // long address : get CV#17 and CV#18
           {
            int cv17=readCV(17);
           
           if  (cv17<0) break;
           int cv18=readCV(18);
           if  (cv18<0) break;
           return cv18 + ((cv17 - 192) <<8);
           }
    case 0: // short address in CV1  
           return readCV(1);
    default: // No response or loco
           break;
  }
  return -1;
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



void  DCC::updateLocoReminder(int loco, byte speedCode) {
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
  speedTable[reg].speedCode = speedCode;
  }
  
DCC::LOCO DCC::speedTable[MAX_LOCOS];
int DCC::nextLoco = 0;
