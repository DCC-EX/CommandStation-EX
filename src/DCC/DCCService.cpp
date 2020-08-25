/*
 *  DCCService.cpp
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

#include "DCCService.h"

DCCService::DCCService(Board* settings) {
  this->board = settings; 
}

void DCCService::schedulePacket(const uint8_t buffer[], uint8_t byteCount, 
  uint8_t repeats, uint16_t identifier) {
  if(byteCount >= kPacketMaxSize) return; // allow for checksum
  
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

  const Packet pushPacket = newPacket;
  
  packetQueue.push(pushPacket);   

  transmitResetCount = 0;
}

const int  MIN_ACK_PULSE_DURATION = 3000;
const int  MAX_ACK_PULSE_DURATION = 9000;



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

uint8_t DCCService::writeCVByte(uint16_t cv, uint8_t bValue, uint16_t callback, 
  uint16_t callbackSub, Print* stream, ACK_CALLBACK callbackFunc) {
  
  ackManagerSetup(cv, bValue, WRITE_BYTE_PROG, WRITECV, callback, callbackSub, stream, callbackFunc);

  return ERR_OK;
}


uint8_t DCCService::writeCVBit(uint16_t cv, uint8_t bNum, uint8_t bValue, 
  uint16_t callback, uint16_t callbackSub, Print* stream, ACK_CALLBACK callbackFunc) {

  ackManagerSetup(cv, bNum, (bValue==0 ? WRITE_BIT0_PROG : WRITE_BIT1_PROG), 
    WRITECVBIT, callback, callbackSub, stream, callbackFunc);

  return ERR_OK;
}


uint8_t DCCService::readCV(uint16_t cv, uint16_t callback, uint16_t callbackSub, 
  Print* stream, ACK_CALLBACK callbackFunc) {
  
  ackManagerSetup(cv, 0, READ_CV_PROG, READCV, callback, callbackSub, stream, callbackFunc);

  return ERR_OK;
}

void DCCService::ackManagerSetup(uint16_t cv, uint8_t value, 
  ackOpCodes const program[], cv_edit_type type, uint16_t callbackNum, 
  uint16_t callbackSub, Print* stream, ACK_CALLBACK callback) {
  
  ackManagerCV = cv;
  ackManagerProg = program;
  ackManagerByte = value;
  ackManagerBitNum = value;
  ackManagerCallback = callback;
  ackManagerCallbackNum = callbackNum;
  ackManagerCallbackSub = callbackSub;
  ackManagerType = type;
  responseStream = stream;
}

void DCCService::setAckPending() {
  //ackPulseStart = 0;
  //ackPulseDuration = 0;
  ackDetected = false;
  //ackCheckStart = millis();
  ackPending = true;
}

uint8_t DCCService::didAck() {
  if(ackPending) return (2);
  if(ackDetected) return (1);
  return(0);
}

void DCCService::checkAck() {
  if(transmitResetCount > 6) {
    // ackCheckDuration = millis() - ackCheckStart;
    ackPending = false;
    return;
  }

  lastCurrent = board->getCurrentMilliamps();

  // Detect the leading edge of a pulse
  if(lastCurrent-board->getCurrentBase() > kACKThreshold) {
    ackDetected = true;
    ackPending = false;
    transmitRepeats = 0;  // stop sending repeats

    return; 
  }
}

void DCCService::ackManagerLoop() {
  if(ackPending) checkAck();

  while (ackManagerProg) {

    // breaks from this switch will step to next prog entry
    // returns from this switch will stay on same entry (typically WACK waiting 
    // and when all finished.)
    uint8_t opcode = pgm_read_byte_near(ackManagerProg);
    uint8_t resets = transmitResetCount;
     
    switch (opcode) {
    case BASELINE:
      if (resets<kResetRepeats) return; // try later 
      board->setCurrentBase();
      break;   
    case W0:    // write 0 bit 
    case W1:    // write 1 bit 
      {
        if (resets<kResetRepeats) return; // try later 
        uint8_t instruction = WRITE_BIT | (opcode==W1 ? BIT_ON : BIT_OFF) | ackManagerBitNum;
        uint8_t message[] = {cv1(BIT_MANIPULATE, ackManagerCV), cv2(ackManagerCV), instruction };
        incrementCounterID();
        schedulePacket(message, sizeof(message), kProgRepeats, counterID);
        setAckPending(); 
      }
      break; 
    
    case WB:   // write byte 
      {
        if (resets<kResetRepeats) return; // try later 
        uint8_t message[] = {cv1(WRITE_BYTE, ackManagerCV), cv2(ackManagerCV), ackManagerByte };
        incrementCounterID();
        schedulePacket(message, sizeof(message), kProgRepeats, counterID);
        setAckPending(); 
      }
      break;
    
    case VB:     // Issue validate Byte packet
      {
        if (resets<kResetRepeats) return; // try later 
        uint8_t message[] = { cv1(VERIFY_BYTE, ackManagerCV), cv2(ackManagerCV), ackManagerByte };
        incrementCounterID();
        schedulePacket(message, sizeof(message), kProgRepeats, counterID);
        setAckPending(); 
      }
      break;
    
    case V0:
    case V1:      // Issue validate bit=0 or bit=1  packet
      {
        if (resets<kResetRepeats) return; // try later 
        uint8_t instruction = VERIFY_BIT | (opcode==V0?BIT_OFF:BIT_ON) | ackManagerBitNum;
        uint8_t message[] = {cv1(BIT_MANIPULATE, ackManagerCV), cv2(ackManagerCV), instruction };
        incrementCounterID();
        schedulePacket(message, sizeof(message), kProgRepeats, counterID);
        setAckPending(); 
      }
      break;
    
    case WACK:   // wait for ack (or absence of ack)
      {
        uint8_t ackState = didAck();
        if (ackState==2) return; // keep polling
        ackReceived = (ackState==1);
      }
      break;  // we have an ACK result (good or bad)
    case ITC0:
    case ITC1:   // If True Callback(0 or 1)  (if prevous WACK got an ACK)
      {
        if (ackReceived) {
          ackManagerProg = NULL; // all done now
          serviceModeResponse response;
          response.cv = ackManagerCV;
          response.cvBitNum = ackManagerBitNum;
          response.cvValue = opcode==ITC0?0:1;
          response.callback = ackManagerCallbackNum;
          response.callbackSub = ackManagerCallbackSub; 
          response.type = ackManagerType;
          callback(responseStream, response);
          return;
        }
      }
      break;
      
    case ITCB:   // If True callback(byte)
      {
        if (ackReceived) {
          ackManagerProg = NULL; // all done now
          serviceModeResponse response;
          response.cv = ackManagerCV;
          response.cvBitNum = 0;
          response.cvValue = ackManagerByte;
          response.callback = ackManagerCallbackNum;
          response.callbackSub = ackManagerCallbackSub; 
          response.type = ackManagerType;
          callback(responseStream, response);
          return;
        }
      }
      break;
      
    case NACKFAIL:   // If nack callback(-1)
      {
        if (!ackReceived) {
          ackManagerProg = NULL; // all done now
          serviceModeResponse response;
          response.cv = ackManagerCV;
          response.cvValue = -1;
          response.callback = ackManagerCallbackNum;
          response.callbackSub = ackManagerCallbackSub; 
          response.type = ackManagerType;
          callback(responseStream, response);
          return;
        }
      }
      break;
      
    case FAIL:  // callback(-1)
      {
        ackManagerProg = NULL;
        serviceModeResponse response;
        response.cv = ackManagerCV;
        response.cvValue = -1;
        response.callback = ackManagerCallbackNum;
        response.callbackSub = ackManagerCallbackSub; 
        response.type = ackManagerType;
        callback(responseStream, response);
      }
      return;
          
    case STARTMERGE:
      ackManagerBitNum=7;
      ackManagerByte=0;     
      break;
        
    case MERGE:  // Merge previous Validate zero wack response with byte value and update bit number (use for reading CV bytes)
      ackManagerByte <<= 1;
      // ackReceived means bit is zero. 
      if (!ackReceived) ackManagerByte |= 1;
      ackManagerBitNum--;
      break;
    default: 
      {
        ackManagerProg=NULL;
        serviceModeResponse response;
        response.cv = ackManagerCV;
        response.cvValue = -1;
        response.callback = ackManagerCallbackNum;
        response.callbackSub = ackManagerCallbackSub; 
        response.type = ackManagerType;
        callback(responseStream, response);
      }
      return;        
    }  // end of switch
    ackManagerProg++;
  }
}
void DCCService::callback(Print* stream, serviceModeResponse response) {
  (ackManagerCallback)(stream, response);
}