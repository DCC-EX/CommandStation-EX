/*
 *  DCCMainRoutines.cpp
 * 
 *  This file is part of CommandStation-EX.
 *
 *  CommandStation-EX is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  CommandStation-EX is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with CommandStation-EX.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "DCC.h"

void DCC::issueReminders() {
  if (packetQueue.count() > 0) return;  // Only update speed if queue is empty

  // This loop searches for a loco in the speed table starting at nextLoco and cycling back around
  for (int reg = 0; reg < numDevices ; reg++) {
    int slot = reg + nextDev;
    if (slot >= numDevices) slot-=numDevices; 
    if (speedTable[slot].cab > 0) {
      // have found the next loco to remind 
      // issueReminder will return true if this loco is completed (ie speed and functions)
      if (issueReminder(slot)) nextDev = slot + 1; 
      return;
    }
  }
}

bool DCC::issueReminder(int reg) {
  unsigned long functions = speedTable[reg].functions;
  int loco = speedTable[reg].cab;
  uint8_t flags = speedTable[reg].groupFlags;
  genericResponse response;
  
  switch (loopStatus) {
  case 0:
//   DIAG(F("\nReminder %d speed %d"),loco,speedTable[reg].speedCode);
    setThrottle(loco, speedTable[reg].speedCode, response);
    break;
  case 1: // remind function group 1 (F0-F4)
    if (flags & FN_GROUP_1) 
      setFunctionInternal(loco, 0, 128 | ((functions>>1) & 0x0F) | ((functions & 0x01)<<4), response);   
    break;     
  case 2: // remind function group 2 F5-F8
    if (flags & FN_GROUP_2) 
      setFunctionInternal(loco, 0, 176 + ((functions>>5) & 0x0F), response);   
    break;     
  case 3: // remind function group 3 F9-F12
    if (flags & FN_GROUP_3) 
      setFunctionInternal(loco, 0, 160 + ((functions>>9) & 0x0F), response);
    break;   
  case 4: // remind function group 4 F13-F20
    if (flags & FN_GROUP_4) 
      setFunctionInternal(loco, 222, ((functions>>13) & 0xFF), response); 
    flags &= ~FN_GROUP_4;  // don't send them again
    break;  
  case 5: // remind function group 5 F21-F28
    if (flags & FN_GROUP_5)
      setFunctionInternal(loco, 223, ((functions>>21) & 0xFF), response); 
    flags &= ~FN_GROUP_5;  // don't send them again
    break; 
  }
  loopStatus++;
  // if we reach status 6 then this loco is done so
  // reset status to 0 for next loco and return true so caller 
  // moves on to next loco. 
  if (loopStatus > 5) loopStatus=0;
  return loopStatus==0;
}

uint8_t DCC::setThrottle(uint16_t addr, uint8_t speedCode, genericResponse& response) {
  
  uint8_t b[5];     // Packet payload. Save space for checksum byte
  uint8_t nB = 0;   // Counter for number of bytes in the packet
  uint16_t railcomAddr = 0;  // For detecting the railcom instruction type

  if(addr > 127) {
    b[nB++] = highByte(addr) | 0xC0;    // convert address to packet format
    railcomAddr = (highByte(addr) | 0xC0) << 8;
  } 

  b[nB++]=lowByte(addr);
  railcomAddr |= lowByte(addr);
  b[nB++]=0x3F;   // 128-step speed control byte
  b[nB++]=speedCode;

  incrementCounterID();
  schedulePacket(b, nB, 0, counterID, kThrottleType, railcomAddr);

  updateSpeedTable(addr, speedCode);

  response.transactionID = counterID;

  return ERR_OK;
}

uint8_t DCC::setFunction(uint16_t addr, uint8_t functionNumber, bool on) {
  if (addr <= 0 || functionNumber > 28) return ERR_OK;
  int reg = lookupSpeedTable(addr);
  if (reg < 0) return ERR_OK;  

  // Take care of functions:
  // Set state of function
  unsigned long funcmask = (1UL<<functionNumber);
  if (on) {
    speedTable[reg].functions |= funcmask;
  } else {
    speedTable[reg].functions &= ~funcmask;
  }
  updateGroupFlags(speedTable[reg].groupFlags, functionNumber);
  
  return ERR_OK;
}

int DCC::changeFunction(uint16_t addr, uint8_t functionNumber, bool pressed) {
  int funcstate = -1;
  if (addr<=0 || functionNumber>28) return funcstate;
  int reg = lookupSpeedTable(addr);
  if (reg<0) return funcstate;  

  // Take care of functions:
  // Imitate how many command stations do it: Button press is
  // toggle but for F2 where it is momentary
  unsigned long funcmask = (1UL<<functionNumber);
  if (functionNumber == 2) {
    // turn on F2 on press and off again at release of button
    if (pressed) {
	    speedTable[reg].functions |= funcmask;
	    funcstate = 1;
    } else {
	    speedTable[reg].functions &= ~funcmask;
	    funcstate = 0;
    }
  } else {
    // toggle function on press, ignore release
    if (pressed) {
	    speedTable[reg].functions ^= funcmask;
    }
    funcstate = speedTable[reg].functions & funcmask;
  }
  updateGroupFlags(speedTable[reg].groupFlags, functionNumber);
  
  return funcstate;
}

uint8_t DCC::setFunctionInternal(uint16_t addr, uint8_t byte1, uint8_t byte2, 
  genericResponse& response) {
  
  uint8_t b[4];     // Packet payload. Save space for checksum byte
  uint8_t nB = 0;   // Counter for number of bytes in the packet
  uint16_t railcomAddr = 0;  // For detecting the railcom instruction type

  if(addr > 127) {
    b[nB++] = highByte(addr) | 0xC0;    // convert address to packet format
    railcomAddr = (highByte(addr) | 0xC0) << 8;
  }

  b[nB++] = lowByte(addr);
  railcomAddr |= lowByte(addr);

  // for safety this guarantees that first byte will either be 0xDE 
  // (for F13-F20) or 0xDF (for F21-F28)
  if (byte1!=0) b[nB++] = byte1;    
  b[nB++]=byte2;
  
  incrementCounterID();
  // Repeat the packet three times (one plus 2 repeats). NMRA spec is minimum 2 repeats.
  schedulePacket(b, nB, 2, counterID, kFunctionType, railcomAddr);  

  response.transactionID = counterID;

  return ERR_OK;
}

// Set the group flag to say we have touched the particular group.
// A group will be reminded only if it has been touched.  
void DCC::updateGroupFlags(uint8_t & flags, int functionNumber) {
  uint8_t groupMask;
  
  if (functionNumber<=4)       groupMask=FN_GROUP_1;
  else if (functionNumber<=8)  groupMask=FN_GROUP_2;
  else if (functionNumber<=12) groupMask=FN_GROUP_3;
  else if (functionNumber<=20) groupMask=FN_GROUP_4;
  else                         groupMask=FN_GROUP_5;

  flags |= groupMask; 
}

uint8_t DCC::setAccessory(uint16_t addr, uint8_t number, bool activate, 
  genericResponse& response) {
  
  uint8_t b[3];     // Packet payload. Save space for checksum byte
  uint16_t railcomAddr = 0;  // For detecting the railcom instruction type

  // first byte is of the form 10AAAAAA, where AAAAAA represent 6 least 
  // signifcant bits of accessory address
  b[0] = (addr % 64) + 128;           
  // second byte is of the form 1AAACDDD, where C should be 1, and the least 
  // significant D represent activate/deactivate                                
  b[1] = ((((addr / 64) % 8) << 4) + (number % 4 << 1) + activate % 2) ^ 0xF8;      
  railcomAddr = (b[0] << 8) | b[1];

  incrementCounterID();
  // Repeat the packet four times (one plus 3 repeats)
  schedulePacket(b, 2, 3, counterID, kAccessoryType, railcomAddr); 

  response.transactionID = counterID;

  return ERR_OK;
}

uint8_t DCC::writeCVByteMain(uint16_t addr, uint16_t cv, uint8_t bValue, 
  genericResponse& response, Print* stream, void (*POMCallback)(Print*, RailComPOMResponse)) {
  
  uint8_t b[6];     // Packet payload. Save space for checksum byte
  uint8_t nB = 0;   // Counter for number of bytes in the packet
  uint16_t railcomAddr = 0;  // For detecting the railcom instruction type

  cv--;   // actual CV addresses are cv-1 (0-1023)

  if(addr > 127) {
    b[nB++] = highByte(addr) | 0xC0;    // convert address to packet format
    railcomAddr = (highByte(addr) | 0xC0) << 8;
  } 

  b[nB++] = lowByte(addr);
  railcomAddr |= lowByte(addr);

  // any CV>1023 will become modulus(1024) due to bit-mask of 0x03
  b[nB++] = 0xEC + (highByte(cv) & 0x03);   
  b[nB++] = lowByte(cv);
  b[nB++] = bValue;

  setPOMResponseCallback(stream, POMCallback);

  incrementCounterID();
  // Repeat the packet four times (one plus 3 repeats)
  schedulePacket(b, nB, 3, counterID, kPOMByteWriteType, railcomAddr); 

  response.transactionID = counterID;

  return ERR_OK;
}

uint8_t DCC::writeCVBitMain(uint16_t addr, uint16_t cv, uint8_t bNum, 
  uint8_t bValue, genericResponse& response, Print *stream, 
  void (*POMCallback)(Print*, RailComPOMResponse)) {
  
  uint8_t b[6];     // Packet payload. Save space for checksum byte
  uint8_t nB = 0;   // Counter for number of bytes in the packet
  uint16_t railcomAddr = 0;  // For detecting the railcom instruction type

  cv--;   // actual CV addresses are cv-1 (0-1023)

  bValue = bValue % 2;
  bNum = bNum % 8;

  if(addr > 127) {
    b[nB++] = highByte(addr) | 0xC0;    // convert address to packet format
    railcomAddr = (highByte(addr) | 0xC0) << 8;
  } 

  b[nB++] = lowByte(addr);
  railcomAddr |= lowByte(addr);

  // any CV>1023 will become modulus(1024) due to bit-mask of 0x03
  b[nB++] = 0xE8 + (highByte(cv) & 0x03);   
  b[nB++] = lowByte(cv);
  b[nB++] = 0xF0 + (bValue * 8) + bNum;

  setPOMResponseCallback(stream, POMCallback);

  incrementCounterID();
  schedulePacket(b, nB, 4, counterID, kPOMBitWriteType, railcomAddr);

  response.transactionID = counterID;

  return ERR_OK;
}

uint8_t DCC::readCVByteMain(uint16_t addr, uint16_t cv, 
  genericResponse& response, Print *stream, void (*POMCallback)(Print*, RailComPOMResponse)) {

  uint8_t b[6];     // Packet payload. Save space for checksum byte
  uint8_t nB = 0;   // Counter for number of bytes in the packet
  uint16_t railcomAddr = 0;  // For detecting the railcom instruction type

  cv--;   // actual CV addresses are cv-1 (0-1023)

  if(addr > 127) {
    b[nB++] = highByte(addr) | 0xC0;    // convert address to packet format
    railcomAddr = (highByte(addr) | 0xC0) << 8;
  } 

  b[nB++] = lowByte(addr);
  railcomAddr |= lowByte(addr);

  // any CV>1023 will become modulus(1024) due to bit-mask of 0x03
  b[nB++] = 0xE4 + (highByte(cv) & 0x03);   
  b[nB++] = lowByte(cv);
  b[nB++] = 0;  // For some reason the railcom spec leaves an empty byte  

  setPOMResponseCallback(stream, POMCallback);

  incrementCounterID();
  // Repeat the packet four times (one plus 3 repeats)
  schedulePacket(b, nB, 3, counterID, kPOMReadType, railcomAddr); 

  response.transactionID = counterID;

  return ERR_OK;

}

uint8_t DCC::readCVBytesMain(uint16_t addr, uint16_t cv, 
  genericResponse& response, Print *stream, void (*POMCallback)(Print*, RailComPOMResponse)) {

  uint8_t b[5];     // Packet payload. Save space for checksum byte
  uint8_t nB = 0;   // Counter for number of bytes in the packet
  uint16_t railcomAddr = 0;  // For detecting the railcom instruction type

  cv--;   // actual CV addresses are cv-1 (0-1023)

  if(addr > 127) {
    b[nB++] = highByte(addr) | 0xC0;    // convert address to packet format
    railcomAddr = (highByte(addr) | 0xC0) << 8;
  } 

  b[nB++] = lowByte(addr);
  railcomAddr |= lowByte(addr);

  // any CV>1023 will become modulus(1024) due to bit-mask of 0x03
  b[nB++] = 0xE0 + (highByte(cv) & 0x03);   
  b[nB++] = lowByte(cv);  

  setPOMResponseCallback(stream, POMCallback);

  incrementCounterID();
  // Repeat the packet four times (one plus 3 repeats)
  schedulePacket(b, nB, 3, counterID, kPOMLongReadType, railcomAddr); 

  response.transactionID = counterID;

  return ERR_OK;

}

uint8_t DCC::getThrottleSpeed(uint8_t cab) {
  int reg=lookupSpeedTable(cab);
  if (reg < 0) {
    return -1;
  }
  return speedTable[reg].speedCode & 0x7F;
}

bool DCC::getThrottleDirection(uint8_t cab) {
  int reg=lookupSpeedTable(cab);
  if (reg < 0) {
    return false;
  }
  return (speedTable[reg].speedCode & 0x80) != 0;
}

// Turns 0 to 127 speed steps and a direction to a speed code
uint8_t DCC::speedAndDirToCode(uint8_t speed, bool dir) {
  return (speed & 0x7F) + (dir ? 128 : 0); 
} 

void DCC::updateSpeedTable(uint8_t cab, uint8_t speedCode) {
  if(cab == 0) {
    // broadcast to all locomotives
    for(int dev = 0; dev < numDevices; dev++) 
      speedTable[dev].speedCode = speedCode;
    return;
  }

  int reg = lookupSpeedTable(cab);
  if(reg >= 0) speedTable[reg].speedCode = speedCode;
}

int DCC::lookupSpeedTable(uint8_t cab) {
  int firstEmpty = numDevices;
  int reg;
  for(reg = 0; reg < numDevices; reg++) {
    if(speedTable[reg].cab == cab) break;   // We found the loco, break

    // We didn't find the loco, but this slot is empty so update firstEmpty
    if(speedTable[reg].cab == 0 && firstEmpty == numDevices) firstEmpty = reg;  
  }
  
  if(reg == numDevices) reg = firstEmpty;   // If we got all the way to the end of the search, it isn't in the system
  if(reg >= numDevices) return -1;    // Not enough locos

  if(reg==firstEmpty) {   // If this cab isn't already in the system, add it
    speedTable[reg].cab = cab;
    speedTable[reg].speedCode = 128;
  }

  return reg;
}

void DCC::forgetDevice(uint8_t cab) {  // removes any speed reminders for this loco  
  int reg = lookupSpeedTable(cab);

  if(reg >= 0) speedTable[reg].cab = 0;
}

void DCC::forgetAllDevices() {  // removes all speed reminders
  for(int i = 0; i < numDevices; i++) speedTable[i].cab = 0;  
}