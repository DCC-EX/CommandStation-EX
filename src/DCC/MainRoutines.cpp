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

void DCC::updateSpeed() {
  if (packetQueue.count() > 0) return;  // Only update speed if queue is empty

  for (; nextDev < numDevices; nextDev++) {
    if (speedTable[nextDev].cab > 0) {
      setThrottleResponse response;
      setThrottle(speedTable[nextDev].cab, speedTable[nextDev].speedCode, response);
      nextDev++;
      return;
    }
  }
  for (nextDev = 0; nextDev < numDevices; nextDev++) {
    if (speedTable[nextDev].cab > 0) {
      setThrottleResponse response;
      setThrottle(speedTable[nextDev].cab, speedTable[nextDev].speedCode, response);
      nextDev++;
      return;
    }
  }
}

uint8_t DCC::setThrottle(uint16_t addr, uint8_t speedCode, setThrottleResponse& response) {
  
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

  response.device = addr;
  response.speed = speedCode;
  response.transactionID = counterID;

  return ERR_OK;
}

uint8_t DCC::setFunction(uint16_t addr, uint8_t byte1, 
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

  b[nB++] = (byte1 | 0x80) & 0xBF;

  incrementCounterID();
  // Repeat the packet four times (one plus 3 repeats)
  schedulePacket(b, nB, 3, counterID, kFunctionType, railcomAddr);  


  response.transactionID = counterID;

  return ERR_OK;
}

uint8_t DCC::setFunction(uint16_t addr, uint8_t byte1, uint8_t byte2, 
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
  b[nB++]=(byte1 | 0xDE) & 0xDF;     
  b[nB++]=byte2;
  
  incrementCounterID();
  // Repeat the packet four times (one plus 3 repeats)
  schedulePacket(b, nB, 3, counterID, kFunctionType, railcomAddr);  

  response.transactionID = counterID;

  return ERR_OK;
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
  if (reg<0) return -1;
  return speedTable[reg].speedCode & 0x7F;
}

bool DCC::getThrottleDirection(uint8_t cab) {
  int reg=lookupSpeedTable(cab);
  if (reg<0) return false;
  return (speedTable[reg].speedCode & 0x80) != 0;
}

// Turns 0 to 127 speed steps and a direction to a speed code
uint8_t DCC::speedAndDirToCode(uint8_t speed, bool dir) {
  return (speed & 0x7F) + dir * 128; 
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