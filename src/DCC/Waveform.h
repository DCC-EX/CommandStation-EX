/*
 *  Waveform.h
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

#ifndef COMMANDSTATION_DCC_WAVEFORM_H_
#define COMMANDSTATION_DCC_WAVEFORM_H_

#include <Arduino.h>

#include "../Boards/Board.h"

const uint8_t kIdlePacket[] = {0xFF,0x00,0xFF};
const uint8_t kResetPacket[] = {0x00,0x00,0x00};
const uint8_t kBitMask[] = {0x00,0x80,0x40,0x20,0x10,0x08,0x04,0x02,0x01};

const uint8_t kPacketMaxSize = 6; 

enum : uint8_t {
  ERR_OK = 1,
  ERR_BUSY = 2,
};

class Waveform {
public:
  virtual bool interrupt1() = 0;
  virtual void interrupt2() = 0;

  void loop() {
    board->checkOverload();
  }

  Board* board;
protected:
  // Data that controls the packet currently being sent out.
  uint8_t bits_sent;  // Bits sent from byte
  uint8_t bytes_sent; // Bytes sent from packet
  uint8_t currentBit = false;
  uint8_t transmitRepeats = 0;  // Repeats (does not include initial transmit)
  uint8_t remainingPreambles = 0; 
  uint8_t generateStartBit = false;  // Send a start bit for the current byte?
  uint8_t transmitPacket[kPacketMaxSize];
  uint8_t transmitLength;
  uint16_t transmitID = 0;

  // Interrupt segments, called in interrupt_handler
  
  uint8_t interruptState = 0; // Waveform generator state

  uint16_t counterID = 1; // Maintains the last assigned packet ID
  bool counterWrap = false;
  inline void incrementCounterID() { 
    counterID++;
    if(counterID == 0) {
      counterID = 1;
      counterWrap = true;
    }
  }
};

#endif