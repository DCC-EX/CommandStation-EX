/*
 *  DCC.h
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

#ifndef COMMANDSTATION_DCC_DCC_H_
#define COMMANDSTATION_DCC_DCC_H_

#include <Arduino.h>

#include "Queue.h"
#include "Waveform.h"

// Timing constraints for an ack pulse (millis)
const int kMinAckPulseDuration = 3000;
const int kMaxAckPulseDuration = 9000;

enum cv_edit_type : uint8_t {
  READCV,
  WRITECV,
  WRITECVBIT,
};

struct setThrottleResponse {
  uint8_t device;
  uint8_t speed;
  uint8_t direction;
  uint16_t transactionID;
};

struct genericResponse {
  uint16_t transactionID;
};

struct serviceModeResponse {
  cv_edit_type type;
  uint16_t callback;
  uint16_t callbackSub;
  uint16_t cv;
  uint8_t cvBitNum;
  int cvValue;  // Might be -1, so int works
};

struct RailComPOMResponse {
  uint32_t data;
  uint16_t transactionID;
};

enum PacketType : uint8_t {
  kResetType,
  kIdleType,
  kThrottleType,
  kFunctionType,
  kAccessoryType,
  kPOMByteWriteType,  // Railcom is same as standard command for write byte
  kPOMBitWriteType,   // Railcom is same as standard command for write bit
  kPOMReadType,
  kPOMLongReadType,
  kSrvcByteWriteType,
  kSrvcBitWriteType,
  kSrvcReadType
};

extern const uint8_t railcom_decode[256];

// invalid value (not conforming to the 4bit weighting requirement)
const uint8_t INV = 0xFF;
// Railcom ACK; the decoder received the message ok. NOTE: some early
// software versions may have ACK and NACK exchanged.
const uint8_t ACK = 0xFE;
// The decoder rejected the packet.
const uint8_t NACK = 0xFD;
// The decoder is busy; send the packet again. This is typically returned
// when a POM CV write is still pending; the caller must re-try sending the
// packet later.
const uint8_t BUSY = 0xFC;
// Reserved for future expansion.
const uint8_t RESVD1 = 0xFB;
const uint8_t RESVD2 = 0xFA;
const uint8_t RESVD3 = 0xF8;

enum RailComInstructionType : uint8_t {
  kMOBInstruction,
  kSTATInstruction,
  kNoInstruction
};

enum RailComMOBID : uint8_t {
  kMOB_POM = 0,
  kMOB_ADR_HIGH = 1,
  kMOB_ADR_LOW = 2,
  kMOB_EXT = 3,
  kMOB_DYN = 7,
  kMOB_SUBID = 12
};

enum RailComSTATID : uint8_t {
  kSTAT_POM = 0,
  kSTAT_STAT1 = 4,
  kSTAT_TIME = 5,
  kSTAT_ERROR = 6,
  kSTAT_DYN = 7,
  kSTAT_STAT2 = 8,
  kSTAT_SUBID = 12
};

struct RailComDatagram {
  uint8_t identifier; // 4-bit ID, LSB justified
  uint8_t channel;  // Railcom channel the data came in on, either 1 or 2
  uint32_t payload; // LSB justified payload of the datagram, excluding the ID
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

const ackOpCodes PROGMEM WRITE_BIT0_PROG[] = {
  BASELINE,
  W0, WACK,
  V0, WACK,  // validate bit is 0 
  ITC1,      // if acked, callback(1)
  FAIL  // callback (-1)
};

const ackOpCodes PROGMEM WRITE_BIT1_PROG[] = {
  BASELINE,
  W1,WACK,
  V1, WACK,  // validate bit is 1 
  ITC1,      // if acked, callback(1)
  FAIL  // callback (-1)
};


const ackOpCodes PROGMEM READ_BIT_PROG[] = {
  BASELINE,
  V1, WACK,  // validate bit is 1 
  ITC1,      // if acked, callback(1)
  V0, WACK,  // validate bit is zero
  ITC0,      // if acked callback 0
  FAIL       // bit not readable 
};
     
const ackOpCodes PROGMEM WRITE_BYTE_PROG[] = {
  BASELINE,
  WB,WACK,    // Write 
  VB,WACK,     // validate byte 
  ITC1,       // if ok callback (1)
  FAIL        // callback (-1)
};
      
      
const ackOpCodes PROGMEM READ_CV_PROG[] = {
  BASELINE,
  STARTMERGE,    //clear bit and byte values ready for merge pass
  // each bit is validated against 0 and the result inverted in MERGE
  // this is because there tend to be more zeros in cv values than ones.  
  // There is no need for one validation as entire byte is validated at the end
  V0, WACK, MERGE,  // read and merge bit 0
  V0, WACK, MERGE,  // read and merge bit 1 etc
  V0, WACK, MERGE,
  V0, WACK, MERGE,
  V0, WACK, MERGE,
  V0, WACK, MERGE,
  V0, WACK, MERGE,
  V0, WACK, MERGE,
  VB, WACK, ITCB,   // verify merged byte and return it if acked ok 
  FAIL              // verification failed
};          

typedef void (*ACK_CALLBACK)(Print* stream, serviceModeResponse result);

// NMRA DCC codes
// TODO: Implement more of these in the main track routines
const uint8_t kSetSpeed = 0x3f;
const uint8_t kWriteByteMain = 0xEC;
const uint8_t kWwiteBitMain = 0xE8;
const uint8_t kWriteByte = 0x7C;
const uint8_t kWriteBit = 0xF0;
const uint8_t kVerifyByte = 0x74;
const uint8_t kVerifyBit = 0xE0;
const uint8_t kBitManipulate = 0x78;
const uint8_t kBitOn = 0x08;
const uint8_t kBitOff = 0x00;

class DCC : public Waveform {
public:
  bool interrupt1();
  void interrupt2();

  uint8_t setThrottle(uint16_t addr, uint8_t speedCode, setThrottleResponse& response);
  uint8_t setFunction(uint16_t addr, uint8_t byte1, genericResponse& response);
  uint8_t setFunction(uint16_t addr, uint8_t byte1, uint8_t byte2, genericResponse& response);
  uint8_t setAccessory(uint16_t addr, uint8_t number, bool activate, genericResponse& response);
  
  
  // Writes a CV to a decoder on the main track and calls a callback function
  // if there is any railcom response to the request.
  uint8_t writeCVByteMain(uint16_t addr, uint16_t cv, uint8_t bValue, 
    genericResponse& response, Print *stream, void (*POMCallback)(Print*, RailComPOMResponse));
  // Writes a single bit to the decoder on the main track and calls a callback 
  // function if there is any railcom response to the request.
  uint8_t writeCVBitMain(uint16_t addr, uint16_t cv, uint8_t bNum, 
    uint8_t bValue, genericResponse& response, Print *stream, 
    void (*POMCallback)(Print*, RailComPOMResponse));
  // Reads one byte from the decoder over railcom and calls a callback function 
  // with the value
  uint8_t readCVByteMain(uint16_t addr, uint16_t cv, 
    genericResponse& response, Print *stream, void (*POMCallback)(Print*, RailComPOMResponse));
  // Reads four bytes from the decoder over railcom. CV corresponds to the
  // first byte, the rest are CV+1, CV+2, and CV+3. Calls a callback function
  // with the four values.
  uint8_t readCVBytesMain(uint16_t addr, uint16_t cv, 
    genericResponse& response, Print *stream, void (*POMCallback)(Print*, RailComPOMResponse));

  uint8_t writeCVByte(uint16_t cv, uint8_t bValue, uint16_t callback, 
    uint16_t callbackSub, Print* stream, ACK_CALLBACK);
  uint8_t writeCVBit(uint16_t cv, uint8_t bNum, uint8_t bValue, 
    uint16_t callback, uint16_t callbackSub, Print* stream, 
    ACK_CALLBACK);
  uint8_t readCV(uint16_t cv, uint16_t callback, uint16_t callbackSub, Print* stream, 
    ACK_CALLBACK);


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

  void forgetDevice(uint8_t cab);
  void forgetAllDevices();

  void rcomProcessData(uint8_t data[kRcomBufferSize], uint16_t id, PacketType txType, uint16_t addr);

  DCC(uint8_t numDevices, Board* board);

  void setup() {
    // Set up board from main file
  }

  void loop() {
    Waveform::loop();
    updateSpeed();
    ackManagerLoop();
    if(!board->getProgMode())
      rcomProcessData(board->rcomBuffer, rcomID, rcomTxType, rcomAddr);
  }

  void setPOMResponseCallback(Print* _stream, void (*_POMResponse)(Print*, RailComPOMResponse)) {
    POMResponse = _POMResponse;
    responseStream = _stream;
  }
private:
  // Queues a packet for the next device in line reminding it of its speed.
  void updateSpeed();
  // Holds state for updateSpeed function.
  uint8_t nextDev = 0;

  struct Packet {
    uint8_t payload[kPacketMaxSize];
    uint8_t length;
    uint8_t repeats;
    uint16_t transmitID;  // Identifier for RailCom and CV programming
    PacketType type;      // Type of packet sent out on tracks
    uint16_t address;     // Address packet sent to
  };

  PacketType transmitType = kIdleType;
  uint16_t transmitAddress = 0;

  // Queue of packets, FIFO, that controls what gets sent out next. Size 5.
  Queue<Packet, 5> packetQueue;

  void schedulePacket(
    const uint8_t buffer[], uint8_t byteCount, uint8_t repeats, 
    uint16_t identifier, PacketType type, uint16_t address);

  void updateSpeedTable(uint8_t cab, uint8_t speedCode);
  int lookupSpeedTable(uint8_t cab);

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
  bool ackPending;
  bool ackDetected; 
  unsigned long ackCheckStart; // millis
  unsigned int ackCheckDuration; // millis       
  unsigned long ackPulseStart; // micros
  unsigned int ackPulseDuration;  // micros
  uint16_t ackMaxCurrent;

  // Railcom cutout variables
  // TODO(davidcutting42@gmail.com): Move these to the railcom class
  bool rcomCutout = false; // Should we do a railcom cutout?
  bool inRcomCutout = false;    // Are we in a cutout?

  void (*POMResponse)(Print*, RailComPOMResponse);
  Print* responseStreamPOM = nullptr;

  uint16_t rcomID;
  PacketType rcomTxType;
  uint16_t rcomAddr;
};

#endif