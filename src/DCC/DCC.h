/*
 *  DCC.h
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

#ifndef DCC_DCC_H_
#define DCC_DCC_H_

#include <Arduino.h>

#include "../Utils/Queue.h"
#include "../Utils/DIAG.h"
#include "../Boards/Board.h"
#include "../../Config.h"

// Timing constraints for an ack pulse (millis)
const int kMinAckPulseDuration = 3000;
const int kMaxAckPulseDuration = 9000;

enum cv_edit_type : uint8_t {
  READCV,
  WRITECV,
  WRITECVBIT,
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

#ifndef ARDUINO_AVR_UNO
extern const uint8_t railcom_decode[256];
#endif 

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

const uint8_t kIdlePacket[] = {0xFF,0x00,0xFF};
const uint8_t kResetPacket[] = {0x00,0x00,0x00};
const uint8_t kBitMask[] = {0x00,0x80,0x40,0x20,0x10,0x08,0x04,0x02,0x01};

const uint8_t kPacketMaxSize = 6; 

enum : uint8_t {
  ERR_OK = 1,
  ERR_BUSY = 2,
};

const uint8_t FN_GROUP_1=0x01;         
const uint8_t FN_GROUP_2=0x02;         
const uint8_t FN_GROUP_3=0x04;         
const uint8_t FN_GROUP_4=0x08;         
const uint8_t FN_GROUP_5=0x10;   

class DCC {
public:
  DCC(uint8_t numDevices, Board* board) {
    this->numDevices = numDevices;
    this->board = board;
    
    // Purge the queue memory
    packetQueue.clear();

    // Allocate memory for the speed table and clear it
    speedTable = (Speed *)calloc(numDevices, sizeof(Speed));
    for (int i = 0; i < numDevices; i++)
    {
      speedTable[i].cab = 0;
      speedTable[i].speedCode = 128;
    }
  };

  uint8_t numLocos() { return numDevices; };

  void loop() {
    board->checkOverload();
    issueReminders();
    
    if(board->getProgMode()) // If we're in programming mode
      ackManagerLoop();
#ifndef ARDUINO_AVR_UNO      
    else
      rcomProcessData(board->rcomBuffer, rcomID, rcomTxType, rcomAddr);
      #endif
  }

  Board* board;

  bool interrupt1();
  void interrupt2();

  uint8_t setThrottle(uint16_t addr, uint8_t speedCode, genericResponse& response);
  uint8_t setFunction(uint16_t addr, uint8_t functionNumber, bool on);
  int changeFunction(uint16_t addr, uint8_t functionNumber, bool pressed);
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
  struct Speed {
    uint16_t cab;
    uint8_t speedCode;
    uint8_t groupFlags;
    unsigned long functions;
  };
  // Speed table holds speed of all devices on the bus that have been set since
  // startup. 
  Speed* speedTable;

  void forgetDevice(uint8_t cab);
  void forgetAllDevices();
  uint8_t getThrottleSpeed(uint8_t cab);
  bool getThrottleDirection(uint8_t cab);
  uint8_t speedAndDirToCode(uint8_t speed, bool dir);

  void rcomProcessData(uint8_t data[kRcomBufferSize], uint16_t id, PacketType txType, uint16_t addr);

  void setPOMResponseCallback(Print* _stream, void (*_POMResponse)(Print*, RailComPOMResponse)) {
    POMResponse = _POMResponse;
    responseStreamProg = _stream;
  }
private:
  uint8_t setFunctionInternal(uint16_t addr, uint8_t byte1, uint8_t byte2, genericResponse& response);
  void updateGroupFlags(uint8_t & flags, int functionNumber);

  // Queues a packet for the next device in line reminding it of its speed/functions.
  void issueReminders();
  bool issueReminder(int reg);
  uint8_t nextDev = 0;  // Holds state for issueReminder function.
  uint8_t loopStatus = 0;

  // Functions for auto management of the speed table.
  void updateSpeedTable(uint8_t cab, uint8_t speedCode);
  int lookupSpeedTable(uint8_t cab);

  bool rcomCutout = false; // Should we do a railcom cutout?

  void (*POMResponse)(Print*, RailComPOMResponse);
  Print* responseStreamPOM = nullptr;

  uint16_t rcomID;
  PacketType rcomTxType;
  uint16_t rcomAddr;

  const uint8_t kProgRepeats = 8;
  const uint8_t kResetRepeats = 8;  

  void ackManagerSetup(uint16_t cv, uint8_t value, ackOpCodes const program[],
    cv_edit_type type, uint16_t callbackNum, uint16_t callbackSub, 
    Print* stream, ACK_CALLBACK callback);
  void ackManagerLoop();
  ackOpCodes const * ackManagerProg = NULL;
  cv_edit_type ackManagerType;
  
  uint16_t ackManagerCallbackNum;
  uint16_t ackManagerCallbackSub;
  uint8_t ackManagerByte;
  uint8_t ackManagerBitNum;
  uint16_t ackManagerCV;
  
  void setAckPending();
  uint8_t didAck();
  void checkAck();
  bool ackReceived = false;
  bool ackPending;
  bool ackDetected; 
  unsigned long ackCheckStart; // millis
  unsigned int ackCheckDuration; // millis       
  unsigned long ackPulseStart; // micros
  unsigned int ackPulseDuration;  // micros
  uint16_t ackMaxCurrent;

  Print* responseStreamProg;
  ACK_CALLBACK ackManagerCallback = NULL;

  uint8_t cv1(uint8_t opcode, uint16_t cv)  {
    cv--;
    return (highByte(cv) & (uint8_t)0x03) | opcode;
  }
  uint8_t cv2(uint16_t cv)  {
    cv--;
    return lowByte(cv);
  }

  void schedulePacket(
    const uint8_t buffer[], uint8_t byteCount, uint8_t repeats, 
    uint16_t identifier, PacketType type, uint16_t address) {
    
    Packet newPacket;

    uint8_t checksum=0;
    for (int b=0; b<byteCount; b++) {
      checksum ^= buffer[b];
      newPacket.payload[b] = buffer[b];
    }
    newPacket.payload[byteCount] = checksum;
    newPacket.length = byteCount+1;
    newPacket.repeats = repeats;
    newPacket.transmitID = identifier;
    newPacket.type = type;
    newPacket.address = address;

    const Packet pushPacket = newPacket;
    noInterrupts();
    packetQueue.push(pushPacket); // Push the packet into the queue for processing
    interrupts();  
  }

  struct Packet {
    uint8_t payload[kPacketMaxSize];
    uint8_t length;
    uint8_t repeats;
    uint16_t transmitID;  // Identifier for RailCom and CV programming
    PacketType type;      // Type of packet sent out on tracks
    uint16_t address;     // Address packet sent to
  };
  
  // Queue of packets, FIFO, that controls what gets sent out next. Size 5.
  Queue<Packet, 5> packetQueue;

  // Data that controls the packet currently being sent out.
  uint8_t interruptState = 0; // Waveform generator state
  uint8_t bits_sent;  // Bits sent from byte...
  uint8_t bytes_sent; // ...and in turn, bytes sent from packet
  uint8_t currentBit = false; 
  
  uint8_t remainingPreambles = 0; 
  
  // Information about the worked packet
  uint8_t transmitPacket[kPacketMaxSize];
  uint8_t transmitLength;
  uint8_t transmitRepeats = 0;  // does not include initial transmit
  uint16_t transmitID = 0;
  PacketType transmitType = kIdleType;
  uint16_t transmitAddress = 0;
  
  // Tracks resets sent since last payload packet
  uint8_t transmitResetCount = 0;   

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

#endif  // DCC_DCC_H_
