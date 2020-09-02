/*
 *  RailCom.cpp
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

#include <avr/pgmspace.h>

#ifndef ARDUINO_AVR_UNO

const uint8_t railcom_decode[256] PROGMEM =
{    INV,    INV,    INV,    INV,    INV,    INV,    INV,    INV,
     INV,    INV,    INV,    INV,    INV,    INV,    INV,   NACK,
     INV,    INV,    INV,    INV,    INV,    INV,    INV,   0x33,
     INV,    INV,    INV,   0x34,    INV,   0x35,   0x36,    INV,
     INV,    INV,    INV,    INV,    INV,    INV,    INV,   0x3A,
     INV,    INV,    INV,   0x3B,    INV,   0x3C,   0x37,    INV,
     INV,    INV,    INV,   0x3F,    INV,   0x3D,   0x38,    INV,
     INV,   0x3E,   0x39,    INV, RESVD3,    INV,    INV,    INV,
     INV,    INV,    INV,    INV,    INV,    INV,    INV,   0x24,
     INV,    INV,    INV,   0x23,    INV,   0x22,   0x21,    INV,
     INV,    INV,    INV,   0x1F,    INV,   0x1E,   0x20,    INV,
     INV,   0x1D,   0x1C,    INV,   0x1B,    INV,    INV,    INV,
     INV,    INV,    INV,   0x19,    INV,   0x18,   0x1A,    INV,
     INV,   0x17,   0x16,    INV,   0x15,    INV,    INV,    INV,
     INV,   0x25,   0x14,    INV,   0x13,    INV,    INV,    INV,
    0x32,    INV,    INV,    INV,    INV,    INV,    INV,    INV,
     INV,    INV,    INV,    INV,    INV,    INV,    INV, RESVD2,
     INV,    INV,    INV,   0x0E,    INV,   0x0D,   0x0C,    INV,
     INV,    INV,    INV,   0x0A,    INV,   0x09,   0x0B,    INV,
     INV,   0x08,   0x07,    INV,   0x06,    INV,    INV,    INV,
     INV,    INV,    INV,   0x04,    INV,   0x03,   0x05,    INV,
     INV,   0x02,   0x01,    INV,   0x00,    INV,    INV,    INV,
     INV,   0x0F,   0x10,    INV,   0x11,    INV,    INV,    INV,
    0x12,    INV,    INV,    INV,    INV,    INV,    INV,    INV,
     INV,    INV,    INV, RESVD1,    INV,   0x2B,   0x30,    INV,
     INV,   0x2A,   0x2F,    INV,   0x31,    INV,    INV,    INV,
     INV,   0x29,   0x2E,    INV,   0x2D,    INV,    INV,    INV,
    0x2C,    INV,    INV,    INV,    INV,    INV,    INV,    INV,
     INV,   BUSY,   0x28,    INV,   0x27,    INV,    INV,    INV,
    0x26,    INV,    INV,    INV,    INV,    INV,    INV,    INV,
     ACK,    INV,    INV,    INV,    INV,    INV,    INV,    INV,
     INV,    INV,    INV,    INV,    INV,    INV,    INV,    INV,
};

void DCC::rcomProcessData(uint8_t data[kRcomBufferSize], uint16_t id, PacketType txType, uint16_t addr) {
  // Check that the buffer is empty
  for (int i = 0; i < kRcomBufferSize; i++) {
    if(data[i] != 0) break;
    if(i == kRcomBufferSize - 1) return; 
  }
  
  // DIAG(F("Railcom RAW %d = %x %x %x %x %x %x %x %x\n\r"), 
  //     id,
  //     data[0], data[1], data[2], data[3], 
  //     data[4], data[5], data[6], data[7]
  //   );

  for (size_t i = 0; i < 8; i++)
  {
    data[i] = pgm_read_byte_near(&railcom_decode[data[i]]);
    // Only throw out the packet if channel 2 is corrupted - channel 1 may be 
    // corrupted by multiple decoders transmitting at once.
    if(i > 1) {  
      if(data[i] == INV || data[i] == RESVD1 || data[i] == RESVD2 || data[i] == RESVD3) goto CLEANUP;
    }
  }
  
  // Verbose print of decoded RailCom data to CommManager
  // CommManager::printf(F("Railcom DCD %d = %x %x %x %x %x %x %x %x\n\r"), 
  //   uniqueID,
  //   rawData[0], rawData[1], rawData[2], rawData[3], 
  //   rawData[4], rawData[5], rawData[6], rawData[7]
  // );
  
  RailComDatagram datagrams[4]; // One in ch1 plus up to three in ch2

  // First datagram is always the same format
  datagrams[0].channel = 1;
  datagrams[0].identifier = (data[0] >> 2) & 0x0F;
  datagrams[0].payload = (data[0] & 0x03) << 6 | (data[1] & 0x3F);

  switch (datagrams[0].identifier)
  {
  case kMOB_ADR_HIGH:
    // CommManager::printf(F("ADR HIGH %d (%d)\n\r"), datagrams[0].payload, 
    //   address);
    break;
  case kMOB_ADR_LOW:
    // CommManager::printf(F("ADR LOW %d (%d)\n\r"), datagrams[0].payload, 
    //   address);
    break;
  }

  // CommManager::printf(F("%d,%d\n\r"), highByte(address), lowByte(address));

  // For now, return if there's a special bit combination in the first byte of
  // channel two.
  if(data[2] == ACK || data[2] == NACK || data[2] == BUSY) goto CLEANUP;


  RailComInstructionType instructionType;
  if(highByte(addr) >= 1 && highByte(addr) <= 127) {
    instructionType = kMOBInstruction;
  }
  else if(highByte(addr) >= 128 && highByte(addr) <= 191) {
    instructionType = kSTATInstruction;
  }
  else if(highByte(addr) >= 192 && highByte(addr) <= 231) {
    instructionType = kMOBInstruction;
  }
  else {
    instructionType = kNoInstruction;
  }


  datagrams[1].channel = 2;
  datagrams[1].identifier = (data[2] >> 2) & 0x0F;

  if(instructionType == kMOBInstruction) {
    switch(datagrams[1].identifier) {
    case kMOB_POM: {
      switch(txType) {  // Decode based on what packet was just sent
      case kPOMBitWriteType:
      case kPOMByteWriteType:
      case kPOMReadType:
	datagrams[1].payload = (data[2] & 0x03);
	datagrams[1].payload <<= 6;
	datagrams[1].payload |= (data[3] & 0x3F);
        break;
      case kPOMLongReadType:
	datagrams[1].payload = (data[2] & 0x03);
	datagrams[1].payload <<= 6;
	datagrams[1].payload |= (data[3] & 0x3F);
	datagrams[1].payload <<= 6;
	datagrams[1].payload |= (data[4] & 0x3F);
	datagrams[1].payload <<= 6;
	datagrams[1].payload |= (data[5] & 0x3F);
	datagrams[1].payload <<= 6;
	datagrams[1].payload |= (data[6] & 0x3F);
	datagrams[1].payload <<= 6;
	datagrams[1].payload |= (data[7] & 0x3F);
         break;
      default:
        goto CLEANUP;
      }
      
      RailComPOMResponse response;

      response.data = datagrams[1].payload;
      response.transactionID = id;

      POMResponse(responseStreamPOM, response);
      
      break;
      }
    case kMOB_EXT:
    case kMOB_DYN:
    case kMOB_SUBID:
      break;  // We will handle these cases in a later revision
    }
  } 
  else if(instructionType == kSTATInstruction) {
    switch(datagrams[1].identifier) {
    case kSTAT_POM:
    case kSTAT_STAT1:
    case kSTAT_TIME:
    case kSTAT_ERROR:
    case kSTAT_DYN:
    case kSTAT_STAT2:
    case kSTAT_SUBID:
      break;  // We will handle these cases in a later revision
    }
  }   

CLEANUP:
  // Reset the buffer to all zeroes
  for (int i = 0; i < kRcomBufferSize; i++) {
    data[i] = 0x00;
  }
}

#endif