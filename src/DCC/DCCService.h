/*
 *  DCCService.h
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

#ifndef COMMANDSTATION_DCC_DCCSERVICE_H_
#define COMMANDSTATION_DCC_DCCSERVICE_H_

#include <Arduino.h>

#include "Waveform.h"
#include "Queue.h"

// Threshold (mA) that a sample must cross to ACK
const uint8_t kACKThreshold = 20; 

enum cv_edit_type : uint8_t {
  READCV,
  WRITECV,
  WRITECVBIT,
};

struct serviceModeResponse {
  cv_edit_type type;
  uint16_t callback;
  uint16_t callbackSub;
  uint16_t cv;
  uint8_t cvBitNum;
  int cvValue;  // Might be -1, so int works
};

enum ackOpCodes {
  BASELINE,   // ensure enough resets sent before starting and obtain baseline 
              // current
  W0,W1,      // issue write bit (0..1) packet 
  WB,         // issue write byte packet 
  VB,         // Issue validate Byte packet
  V0, V1,     // issue validate bit (0/1) packet
  WACK,       // wait for ack (or absence of ack)
  ITC1,       // If True Callback(1)  (if prevous WACK got an ACK)
  ITC0,       // If True callback(0);
  ITCB,       // If True callback(byte)
  NACKFAIL,   // if false callback(-1)
  FAIL,       // callback(-1)
  STARTMERGE, // Clear bit and byte settings ready for merge pass 
  MERGE,      // Merge previous wack response with byte value and decrement bit 
              // number (use for reading CV bytes)
};

typedef void (*ACK_CALLBACK)(Print* stream, serviceModeResponse result);

class DCCService : public Waveform {
public:
  DCCService(Board* board);

  void setup() {
    // board.setup must be called from the main file
  }

  void loop() {
    Waveform::loop(); // Checks for overcurrent and manages power
    ackManagerLoop();
  }

  bool interrupt1();
  void interrupt2();

  uint8_t writeCVByte(uint16_t cv, uint8_t bValue, uint16_t callback, 
    uint16_t callbackSub, Print* stream, ACK_CALLBACK);
  uint8_t writeCVBit(uint16_t cv, uint8_t bNum, uint8_t bValue, 
    uint16_t callback, uint16_t callbackSub, Print* stream, 
    ACK_CALLBACK);
  uint8_t readCV(uint16_t cv, uint16_t callback, uint16_t callbackSub, Print* stream, 
    ACK_CALLBACK);

private:
  struct Packet {
    uint8_t payload[kPacketMaxSize];
    uint8_t length;
    uint8_t repeats;
    uint16_t transmitID;  // Identifier for CV programming
  };

  // Queue of packets, FIFO, that controls what gets sent out next.
  Queue<Packet, 5> packetQueue;

  void schedulePacket(const uint8_t buffer[], uint8_t byteCount, 
    uint8_t repeats, uint16_t identifier);  

  // ACK MANAGER
  void ackManagerSetup(uint16_t cv, uint8_t value, ackOpCodes const program[],
    cv_edit_type type, uint16_t callbackNum, uint16_t callbackSub, 
    Print* stream, ACK_CALLBACK callback);
  void ackManagerLoop();
  void callback(Print* stream, serviceModeResponse value);
  ackOpCodes const * ackManagerProg = NULL;
  uint8_t ackManagerByte = 0;
  uint8_t ackManagerBitNum = 0;
  uint16_t ackManagerCV = 0;
  bool ackReceived = false;
  const uint8_t kProgRepeats = 8;
  const uint8_t kResetRepeats = 8;
  ACK_CALLBACK ackManagerCallback = NULL;
  uint16_t ackManagerCallbackNum = 0;
  uint16_t ackManagerCallbackSub = 0;
  cv_edit_type ackManagerType = READCV;
  Print* responseStream;

  uint8_t cv1(uint8_t opcode, uint16_t cv)  {
    cv--;
    return (highByte(cv) & (uint8_t)0x03) | opcode;
  }
  uint8_t cv2(uint16_t cv)  {
    cv--;
    return lowByte(cv);
  }

  uint8_t transmitResetCount = 0;   // Tracks resets sent since last payload packet

  void setAckPending();
  uint8_t didAck();
  void checkAck();
  uint16_t lastCurrent = 0;
  bool ackPending = false;
  bool ackDetected = false; 

  // NMRA codes #
  const uint8_t SET_SPEED = 0x3f;
  const uint8_t WRITE_BYTE_MAIN = 0xEC;
  const uint8_t WRITE_BIT_MAIN = 0xE8;
  const uint8_t WRITE_BYTE = 0x7C;
  const uint8_t VERIFY_BYTE = 0x74;
  const uint8_t BIT_MANIPULATE = 0x78;
  const uint8_t WRITE_BIT = 0xF0;
  const uint8_t VERIFY_BIT = 0xE0;
  const uint8_t BIT_ON = 0x08;
  const uint8_t BIT_OFF = 0x00;
};

#endif