/*
 *  Railcom.cpp
 * 
 *  This file is part of CommandStation.
 *
 *  CommandStation is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  CommandStation is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with CommandStation.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "Railcom.h"

#include <avr/pgmspace.h>

#include "../CommInterface/CommManager.h"

#if defined(ARDUINO_ARCH_SAMD) || defined(ARDUINO_ARCH_SAMC)
  #include "wiring_private.h"
#endif

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

void Railcom::setup() {
  if(config.enable) {

  #if defined(ARDUINO_ARCH_SAMD) || defined(ARDUINO_ARCH_SAMC)
    if(config.serial == nullptr) {
      config.serial = new Uart(config.sercom, config.rx_pin, config.tx_pin, config.rx_pad, config.tx_pad);
    }
  #endif
    config.serial->begin(config.baud);
  }
}

// TODO(davidcutting42@gmail.com): test on AVR
void Railcom::enableRecieve(uint8_t on) {
  if(on) {
    while(config.serial->available()) {
      config.serial->read();   // Flush the buffer so we don't get a bunch of garbage
    }
  #if defined(ARDUINO_ARCH_SAMD)
    pinPeripheral(config.rx_pin, config.rx_mux);
  #else
    config.serial->begin(config.baud);
  #endif    
  }
  else {
  #if defined(ARDUINO_ARCH_SAMD)
    pinPeripheral(config.rx_pin, PIO_INPUT);
  #else
    config.serial->end();
  #endif    
  }
}

// This is called from an interrupt routine, so it's gotta be quick. DON'T try
// to write to the serial port here. You'll destroy the waveform.
void Railcom::readData(uint16_t _uniqueID, PacketType _packetType, 
  uint16_t _address) {

  if(dataReady) return;
  
  uint8_t bytes = config.serial->available();
  if(bytes > 8) bytes = 8;
  config.serial->readBytes(rawData, bytes);

  uniqueID = _uniqueID;
  address = _address;
  type = _packetType;
  
  if(bytes > 0)
    dataReady = true;
}

void Railcom::processData() {
  if(dataReady) {
    // CommManager::printf(F("Railcom RAW %d = %x %x %x %x %x %x %x %x\n\r"), 
    //   uniqueID,
    //   rawData[0], rawData[1], rawData[2], rawData[3], 
    //   rawData[4], rawData[5], rawData[6], rawData[7]
    // );

    for (size_t i = 0; i < 8; i++)
    {
      rawData[i] = pgm_read_byte_near(&railcom_decode[rawData[i]]);
      // Only throw out the packet if channel 2 is corrupted - channel 1 may be 
      // corrupted by multiple decoders transmitting at once.
      if(i > 1) {  
        if(rawData[i] == INV || rawData[i] == RESVD1 || rawData[i] == RESVD2 
            || rawData[i] == RESVD3) {  
          dataReady = false;
          return;
        }
      }
    }
    
    // Verbose print of decoded railcom data to CommManager
    // CommManager::printf(F("Railcom DCD %d = %x %x %x %x %x %x %x %x\n\r"), 
    //   uniqueID,
    //   rawData[0], rawData[1], rawData[2], rawData[3], 
    //   rawData[4], rawData[5], rawData[6], rawData[7]
    // );
    
    RailcomDatagram datagrams[4]; // One in ch1 plus up to three in ch2

    // First datagram is always the same format
    datagrams[0].channel = 1;
    datagrams[0].identifier = (rawData[0] >> 2) & 0x0F;
    datagrams[0].payload = (rawData[0] & 0x03) << 6 | (rawData[1] & 0x3F);

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
    if(rawData[2] == ACK || rawData[2] == NACK || rawData[2] == BUSY) {
      dataReady = false;
      return;
    }

    RailcomInstructionType instructionType;
    if(highByte(address) >= 1 && highByte(address) <= 127) {
      instructionType = kMOBInstruction;
    }
    else if(highByte(address) >= 128 && highByte(address) <= 191) {
      instructionType = kSTATInstruction;
    }
    else if(highByte(address) >= 192 && highByte(address) <= 231) {
      instructionType = kMOBInstruction;
    }
    else {
      instructionType = kNoInstruction;
    }


    datagrams[1].channel = 2;
    datagrams[1].identifier = (rawData[2] >> 2) & 0x0F;

    if(instructionType == kMOBInstruction) {
      switch(datagrams[1].identifier) {
      case kMOB_POM: {
        switch(type) {  // Decode based on what packet was just sent
        case kPOMBitWriteType:
        case kPOMByteWriteType:
        case kPOMReadType:
          datagrams[1].payload = 
            ((rawData[2] & 0x03) << 6) | 
            (rawData[3] & 0x3F);
          break;
        case kPOMLongReadType:
          datagrams[1].payload = 
            ((rawData[2] & 0x03) << 30) | 
            ((rawData[3] & 0x3F) << 24) |
            ((rawData[4] & 0x3F) << 18) |
            ((rawData[5] & 0x3F) << 12) |
            ((rawData[6] & 0x3F) << 6) |
            (rawData[7] & 0x3F);
          break;
        default:
          dataReady = false;
          return;
        }
        
        RailcomPOMResponse response;

        response.data = datagrams[1].payload;
        response.transactionID = uniqueID;

        POMResponse(responseStream, response);
        
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
    
    dataReady = false;
    return;
  }
}