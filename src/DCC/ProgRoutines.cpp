/*
 *  DCCProgRoutines.cpp
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

uint8_t DCC::writeCVByte(uint16_t cv, uint8_t bValue, uint16_t callback, 
  uint16_t callbackSub, Print* stream, ACK_CALLBACK callbackFunc) {
  
  ackManagerSetup(cv, bValue, WRITE_BYTE_PROG, WRITECV, callback, callbackSub, stream, callbackFunc);

  return ERR_OK;
}


uint8_t DCC::writeCVBit(uint16_t cv, uint8_t bNum, uint8_t bValue, 
  uint16_t callback, uint16_t callbackSub, Print* stream, ACK_CALLBACK callbackFunc) {

  ackManagerSetup(cv, bNum, (bValue==0 ? WRITE_BIT0_PROG : WRITE_BIT1_PROG), 
    WRITECVBIT, callback, callbackSub, stream, callbackFunc);

  return ERR_OK;
}


uint8_t DCC::readCV(uint16_t cv, uint16_t callback, uint16_t callbackSub, 
  Print* stream, ACK_CALLBACK callbackFunc) {
  
  ackManagerSetup(cv, 0, READ_CV_PROG, READCV, callback, callbackSub, stream, callbackFunc);

  return ERR_OK;
}

void DCC::ackManagerSetup(uint16_t cv, uint8_t value, 
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
  responseStreamProg = stream;
}

void DCC::setAckPending() {
  ackMaxCurrent = 0;
  ackPulseStart = 0;
  ackPulseDuration = 0;
  ackDetected = false;
  ackCheckStart = millis();
  ackPending = true;
}

uint8_t DCC::didAck() {
  if(ackPending) return (2);
  if(ackDetected) return (1);
  return(0);
}

void DCC::checkAck() {
  if(transmitResetCount > 6) {
    ackCheckDuration = millis() - ackCheckStart;
    ackPending = false;
    return;
  }

  uint16_t lastCurrent = board->getAckMilliamps();
  if (lastCurrent > ackMaxCurrent) ackMaxCurrent=lastCurrent;

  // Detect the leading edge of a pulse
  if(lastCurrent > board->getThreshold()) {
    if (ackPulseStart==0) ackPulseStart=micros();    // leading edge of pulse detected
    return;
  }

  if (ackPulseStart==0) return; // keep waiting for leading edge 

  ackPulseDuration=micros()-ackPulseStart;

  if (ackPulseDuration>=kMinAckPulseDuration && ackPulseDuration<=kMaxAckPulseDuration) {
    ackCheckDuration=millis()-ackCheckStart;
    ackDetected=true;
    ackPending=false;
    transmitRepeats=0;  // shortcut remaining repeat packets
    DIAG(F("\n\rGood ACK: micros=%d max=%d"), ackPulseDuration, ackMaxCurrent);
    return;  // we have a genuine ACK result
  }      

  DIAG(F("\n\rBad  ACK: micros=%d max=%d"), ackPulseDuration, ackMaxCurrent);
  ackPulseStart=0;  // We have detected a too-short or too-long pulse so ignore and wait for next leading edge 
}

void DCC::ackManagerLoop() {
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
        DIAG(F("\n\rW%d cv=%d bit=%d"), opcode==W1, ackManagerCV, ackManagerBitNum);
        uint8_t instruction = kWriteBit | (opcode==W1 ? kBitOn : kBitOff) | ackManagerBitNum;
        uint8_t message[] = {cv1(kBitManipulate, ackManagerCV), cv2(ackManagerCV), instruction };
        incrementCounterID();
        schedulePacket(message, sizeof(message), kProgRepeats, counterID, kSrvcBitWriteType, 0);
        transmitResetCount = 0;
        setAckPending(); 

      }
      break; 
    
    case WB:   // write byte 
      {
        if (resets<kResetRepeats) return; // try later 
        DIAG(F("\n\rWB cv=%d value=%d"), ackManagerCV, ackManagerByte);
        uint8_t message[] = {cv1(kWriteByte, ackManagerCV), cv2(ackManagerCV), ackManagerByte };
        incrementCounterID();
        schedulePacket(message, sizeof(message), kProgRepeats, counterID, kSrvcByteWriteType, 0);
        transmitResetCount = 0;
        setAckPending(); 
      }
      break;
    
    case VB:     // Issue validate Byte packet
      {
        if (resets<kResetRepeats) return; // try later 
        DIAG(F("\n\rVB cv=%d value=%d"),ackManagerCV,ackManagerByte);
        uint8_t message[] = { cv1(kVerifyByte, ackManagerCV), cv2(ackManagerCV), ackManagerByte };
        incrementCounterID();
        schedulePacket(message, sizeof(message), kProgRepeats, counterID, kSrvcReadType, 0);
        transmitResetCount = 0;
        setAckPending(); 
      }
      break;
    
    case V0:
    case V1:      // Issue validate bit=0 or bit=1  packet
      {
        if (resets<kResetRepeats) return; // try later 
        DIAG(F("\n\rV%d cv=%d bit=%d"), opcode==V1, ackManagerCV, ackManagerBitNum);
        uint8_t instruction = kVerifyBit | (opcode == V0 ? kBitOff : kBitOn) | ackManagerBitNum;
        uint8_t message[] = {cv1(kBitManipulate, ackManagerCV), cv2(ackManagerCV), instruction };
        incrementCounterID();
        schedulePacket(message, sizeof(message), kProgRepeats, counterID, kSrvcReadType, 0);
        transmitResetCount = 0;
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
          ackManagerCallback(responseStreamProg, response);
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
          ackManagerCallback(responseStreamProg, response);
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
          ackManagerCallback(responseStreamProg, response);
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
        ackManagerCallback(responseStreamProg, response);
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
      DIAG(F(" %d"), !ackReceived);
      ackManagerBitNum--;
      break;
    default: 
      {
        DIAG(F("\n\r!! ackOp %d FAULT!!"), opcode);
        ackManagerProg=NULL;
        serviceModeResponse response;
        response.cv = ackManagerCV;
        response.cvValue = -1;
        response.callback = ackManagerCallbackNum;
        response.callbackSub = ackManagerCallbackSub; 
        response.type = ackManagerType;
        ackManagerCallback(responseStreamProg, response);
      }
      return;        
    }  // end of switch
    ackManagerProg++;
  }
}

