/*
 *  DCCMain.h
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

#ifndef COMMANDSTATION_DCC_DCCMAIN_H_
#define COMMANDSTATION_DCC_DCCMAIN_H_

#include <Arduino.h>

#include "Waveform.h"
#include "Railcom.h"
#include "Queue.h"

struct setThrottleResponse {
  uint8_t device;
  uint8_t speed;
  uint8_t direction;
  uint16_t transactionID;
};

struct genericResponse {
  uint16_t transactionID;
};

class DCCMain : public Waveform {
public:
  DCCMain(uint8_t numDevices, Board* board, Railcom* railcom);

  void setup() {
    // board.setup must be called from the main file
    railcom->setup();
  }

  void loop() {
    Waveform::loop();
    updateSpeed();
    railcom->processData();
  }

  bool interrupt1();
  void interrupt2();

  uint8_t setThrottle(uint16_t addr, uint8_t speedCode, setThrottleResponse& response);
  uint8_t setFunction(uint16_t addr, uint8_t byte1, genericResponse& response);
  uint8_t setFunction(uint16_t addr, uint8_t byte1, uint8_t byte2, genericResponse& response);
  uint8_t setAccessory(uint16_t addr, uint8_t number, bool activate, genericResponse& response);
  // Writes a CV to a decoder on the main track and calls a callback function
  // if there is any railcom response to the request.
  uint8_t writeCVByteMain(uint16_t addr, uint16_t cv, uint8_t bValue, 
    genericResponse& response, Print *stream, void (*POMCallback)(Print*, RailcomPOMResponse));
  // Writes a single bit to the decoder on the main track and calls a callback 
  // function if there is any railcom response to the request.
  uint8_t writeCVBitMain(uint16_t addr, uint16_t cv, uint8_t bNum, 
    uint8_t bValue, genericResponse& response, Print *stream, 
    void (*POMCallback)(Print*, RailcomPOMResponse));
  // Reads one byte from the decoder over railcom and calls a callback function 
  // with the value
  uint8_t readCVByteMain(uint16_t addr, uint16_t cv, 
    genericResponse& response, Print *stream, void (*POMCallback)(Print*, RailcomPOMResponse));
  // Reads four bytes from the decoder over railcom. CV corresponds to the
  // first byte, the rest are CV+1, CV+2, and CV+3. Calls a callback function
  // with the four values.
  uint8_t readCVBytesMain(uint16_t addr, uint16_t cv, 
    genericResponse& response, Print *stream, void (*POMCallback)(Print*, RailcomPOMResponse));

  uint8_t numDevices;

  // Holds info about a device's speed and direction. 
  // TODO(davidcutting42@gmail.com): Make this private
  struct Speed {
    uint16_t cab;
    uint8_t speedCode;
  };
  // Speed table holds speed of all devices on the bus that have been set since
  // startup. 
  Speed* speedTable;

  // Railcom object, complements hdw object inherited from Waveform
  Railcom* railcom;

  void forgetDevice(uint8_t cab);
  void forgetAllDevices();

private:
  // Queues a packet for the next device in line reminding it of its speed.
  void updateSpeed();
  // Holds state for updateSpeed function.
  uint8_t nextDev = 0;

  struct Packet {
    uint8_t payload[kPacketMaxSize];
    uint8_t length;
    uint8_t repeats;
    uint16_t transmitID;  // Identifier for railcom, etc.
    PacketType type;
    uint16_t address;
  };

  PacketType transmitType = kIdleType;
  uint16_t transmitAddress = 0;

  // Queue of packets, FIFO, that controls what gets sent out next. Size 5.
  Queue<Packet, 5> packetQueue;

  void schedulePacket(const uint8_t buffer[], uint8_t byteCount, 
    uint8_t repeats, uint16_t identifier, PacketType type, uint16_t address);

  // Railcom cutout variables
  // TODO(davidcutting42@gmail.com): Move these to the railcom class
  bool generateRailcomCutout = false; // Should we do a railcom cutout?
  bool inRailcomCutout = false;    // Are we in a cutout?
  bool railcomData = false;    // Is there railcom data available? 

  void updateSpeedTable(uint8_t cab, uint8_t speedCode);
  int lookupSpeedTable(uint8_t cab);
};

#endif