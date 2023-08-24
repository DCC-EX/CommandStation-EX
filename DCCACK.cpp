/*
 *  © 2021 M Steve Todd
 *  © 2021 Mike S
 *  © 2021 Fred Decker
 *  © 2020-2021 Harald Barth
 *  © 2020-2022 Chris Harlow
 *  All rights reserved.
 *  
 *  This file is part of CommandStation-EX
 *
 *  This is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  It is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with CommandStation.  If not, see <https://www.gnu.org/licenses/>.
 */
#include "DCCACK.h"
#include "DIAG.h"
#include "DCC.h"
#include "DCCWaveform.h"
#include "TrackManager.h"

unsigned int DCCACK::minAckPulseDuration = 2000; // micros
unsigned int DCCACK::maxAckPulseDuration = 20000; // micros
  
MotorDriver *  DCCACK::progDriver=NULL;
ackOp  const *  DCCACK::ackManagerProg;
ackOp  const *  DCCACK::ackManagerProgStart;
byte   DCCACK::ackManagerByte;
byte   DCCACK::ackManagerByteVerify;
byte   DCCACK::ackManagerStash;
int    DCCACK::ackManagerWord;
byte   DCCACK::ackManagerRetry;
byte   DCCACK::ackRetry = 2;
int16_t  DCCACK::ackRetrySum;
int16_t  DCCACK::ackRetryPSum;
int    DCCACK::ackManagerCv;
byte   DCCACK::ackManagerBitNum;
bool   DCCACK::ackReceived;
bool   DCCACK::ackManagerRejoin;
volatile uint8_t DCCACK::numAckGaps=0;
volatile uint8_t DCCACK::numAckSamples=0;
uint8_t DCCACK::trailingEdgeCounter=0;


 unsigned int DCCACK::ackPulseDuration;  // micros
 unsigned long DCCACK::ackPulseStart; // micros
 volatile bool DCCACK::ackDetected;
 unsigned long DCCACK::ackCheckStart; // millis
 volatile bool DCCACK::ackPending;
  bool   DCCACK::autoPowerOff;
   int  DCCACK::ackThreshold; 
   int  DCCACK::ackLimitmA = 50;
     int DCCACK::ackMaxCurrent;
      unsigned int DCCACK::ackCheckDuration; // millis       
    
    
CALLBACK_STATE DCCACK::callbackState=READY;

ACK_CALLBACK DCCACK::ackManagerCallback;

void  DCCACK::Setup(int cv, byte byteValueOrBitnum, ackOp const program[], ACK_CALLBACK callback) {
  ackManagerRejoin=TrackManager::isJoined();
  if (ackManagerRejoin) {
    // Change from JOIN must zero resets packet.
    TrackManager::setJoin(false);
    DCCWaveform::progTrack.clearResets();
  }

  progDriver=TrackManager::getProgDriver();
  if (progDriver==NULL) {
    TrackManager::setJoin(ackManagerRejoin);
    callback(-3); // we dont have a prog track!
    return;
  }
  if (!progDriver->canMeasureCurrent()) {
    TrackManager::setJoin(ackManagerRejoin);
    callback(-2); // our prog track cant measure current
    return;
  }

   autoPowerOff=false;
   if (progDriver->getPower() == POWERMODE::OFF) {
        autoPowerOff=true;  // power off afterwards
        if (Diag::ACK) DIAG(F("Auto Prog power on"));
        progDriver->setPower(POWERMODE::ON);
	
    /* TODO !!! in MotorDriver surely!
    if (MotorDriver::commonFaultPin)
	  DCCWaveform::mainTrack.setPowerMode(POWERMODE::ON);
        DCCWaveform::progTrack.clearResets();
   **/
      }


  ackManagerCv = cv;
  ackManagerProg = program;
  ackManagerProgStart = program;
  ackManagerRetry = ackRetry;
  ackManagerByte = byteValueOrBitnum;
  ackManagerByteVerify = byteValueOrBitnum;
  ackManagerBitNum=byteValueOrBitnum;
  ackManagerCallback = callback;
}

void  DCCACK::Setup(int wordval, ackOp const program[], ACK_CALLBACK callback) {
  ackManagerWord=wordval;
  Setup(0, 0, program, callback);
  }

const byte RESET_MIN=8;  // tuning of reset counter before sending message

// checkRessets return true if the caller should yield back to loop and try later.
bool DCCACK::checkResets(uint8_t numResets) {
  return DCCWaveform::progTrack.getResets() < numResets;
}
// Operations applicable to PROG track ONLY.
// (yes I know I could have subclassed the main track but...) 

void DCCACK::setAckBaseline() {
      int baseline=progDriver->getCurrentRaw();
      ackThreshold= baseline + progDriver->mA2raw(ackLimitmA);
      if (Diag::ACK) DIAG(F("ACK baseline=%d/%dmA Threshold=%d/%dmA Duration between %uus and %uus"),
			  baseline,progDriver->raw2mA(baseline),
			  ackThreshold,progDriver->raw2mA(ackThreshold),
                          minAckPulseDuration, maxAckPulseDuration);
}

void DCCACK::setAckPending() {
      ackMaxCurrent=0;
      ackPulseStart=0;
      ackPulseDuration=0;
      ackDetected=false;
      ackCheckStart=millis();
      numAckSamples=0;
      numAckGaps=0;
      ackPending=true;  // interrupt routines will now take note
}

byte DCCACK::getAck() {
      if (ackPending) return (2);  // still waiting
      if (Diag::ACK) DIAG(F("%S after %dmS max=%d/%dmA pulse=%uuS samples=%d gaps=%d"),ackDetected?F("ACK"):F("NO-ACK"), ackCheckDuration,
			  ackMaxCurrent,progDriver->raw2mA(ackMaxCurrent), ackPulseDuration, numAckSamples, numAckGaps);
      if (ackDetected) return (1); // Yes we had an ack
      return(0);  // pending set off but not detected means no ACK.   
}

#ifndef DISABLE_PROG
void DCCACK::loop() {
  while (ackManagerProg) {
    byte opcode=GETFLASH(ackManagerProg);

    // breaks from this switch will step to next prog entry
    // returns from this switch will stay on same entry
    // (typically waiting for a reset counter or ACK waiting, or when all finished.)
    switch (opcode) {
      case BASELINE:
          if (progDriver->getPower()==POWERMODE::OVERLOAD) return;
      	  if (checkResets(autoPowerOff || ackManagerRejoin  ? 20 : 3)) return;
          setAckBaseline();
          callbackState=AFTER_READ;
          break;
      case W0:    // write 0 bit
      case W1:    // write 1 bit
            {
	      if (checkResets(RESET_MIN)) return;
              if (Diag::ACK) DIAG(F("W%d cv=%d bit=%d"),opcode==W1, ackManagerCv,ackManagerBitNum);
              byte instruction = WRITE_BIT | (opcode==W1 ? BIT_ON : BIT_OFF) | ackManagerBitNum;
              byte message[] = {DCC::cv1(BIT_MANIPULATE, ackManagerCv), DCC::cv2(ackManagerCv), instruction };
              DCCWaveform::progTrack.schedulePacket(message, sizeof(message), PROG_REPEATS);
              setAckPending();
             callbackState=AFTER_WRITE;
         }
            break;

      case WB:   // write byte
            {
	      if (checkResets( RESET_MIN)) return;
              if (Diag::ACK) DIAG(F("WB cv=%d value=%d"),ackManagerCv,ackManagerByte);
              byte message[] = {DCC::cv1(WRITE_BYTE, ackManagerCv), DCC::cv2(ackManagerCv), ackManagerByte};
              DCCWaveform::progTrack.schedulePacket(message, sizeof(message), PROG_REPEATS);
              setAckPending();
              callbackState=AFTER_WRITE;
            }
            break;

      case   VB:     // Issue validate Byte packet
        {
	  if (checkResets( RESET_MIN)) return;
          if (Diag::ACK) DIAG(F("VB cv=%d value=%d"),ackManagerCv,ackManagerByte);
          byte message[] = { DCC::cv1(VERIFY_BYTE, ackManagerCv), DCC::cv2(ackManagerCv), ackManagerByte};
          DCCWaveform::progTrack.schedulePacket(message, sizeof(message), PROG_REPEATS);
          setAckPending();
        }
        break;

      case V0:
      case V1:      // Issue validate bit=0 or bit=1  packet
        {
	  if (checkResets(RESET_MIN)) return;
          if (Diag::ACK) DIAG(F("V%d cv=%d bit=%d"),opcode==V1, ackManagerCv,ackManagerBitNum);
          byte instruction = VERIFY_BIT | (opcode==V0?BIT_OFF:BIT_ON) | ackManagerBitNum;
          byte message[] = {DCC::cv1(BIT_MANIPULATE, ackManagerCv), DCC::cv2(ackManagerCv), instruction };
          DCCWaveform::progTrack.schedulePacket(message, sizeof(message), PROG_REPEATS);
          setAckPending();
        }
        break;

      case WACK:   // wait for ack (or absence of ack)
         {
          byte ackState=2; // keep polling

          ackState=getAck();
          if (ackState==2) return; // keep polling
          ackReceived=ackState==1;
          break;  // we have a genuine ACK result
         }
     case ITC0:
     case ITC1:   // If True Callback(0 or 1)  (if prevous WACK got an ACK)
        if (ackReceived) {
            callback(opcode==ITC0?0:1);
            return;
          }
        break;

      case ITCB:   // If True callback(byte)
          if (ackReceived) {
            callback(ackManagerByte);
            return;
          }
        break;
		
      case ITCBV:   // If True callback(byte) - Verify
          if (ackReceived) {
            if (ackManagerByte == ackManagerByteVerify) {
               ackRetrySum ++;
               LCD(1, F("v %d %d Sum=%d"), ackManagerCv, ackManagerByte, ackRetrySum);
            }
            callback(ackManagerByte);
            return;
          }
        break;
		
      case ITCB7:   // If True callback(byte & 0x7F)
          if (ackReceived) {
            callback(ackManagerByte & 0x7F);
            return;
          }
        break;

      case NAKFAIL:   // If nack callback(-1)
          if (!ackReceived) {
            callback(-1);
            return;
          }
        break;

      case CALLFAIL:  // callback(-1)
           callback(-1);
           return;

      case BIV:     // ackManagerByte initial value
           ackManagerByte = ackManagerByteVerify;
           break;

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

      case SETBIT:
          ackManagerProg++;
          ackManagerBitNum=GETFLASH(ackManagerProg);
          break;

     case SETCV:
          ackManagerProg++;
          ackManagerCv=GETFLASH(ackManagerProg);
          break;

     case SETBYTE:
          ackManagerProg++;
          ackManagerByte=GETFLASH(ackManagerProg);
          break;

    case SETBYTEH:
          ackManagerByte=highByte(ackManagerWord);
          break;

    case SETBYTEL:
          ackManagerByte=lowByte(ackManagerWord);
          break;

     case STASHLOCOID:
          ackManagerStash=ackManagerByte;  // stash value from CV17
          break;

     case COMBINELOCOID:
          // ackManagerStash is  cv17, ackManagerByte is CV 18
          callback( LONG_ADDR_MARKER | ( ackManagerByte + ((ackManagerStash - 192) << 8)));
          return;

     case ITSKIP:
          if (!ackReceived) break;
          // SKIP opcodes until SKIPTARGET found
          while (opcode!=SKIPTARGET) {
            ackManagerProg++;
            opcode=GETFLASH(ackManagerProg);
          }
          break;
     case SKIPTARGET:
          break;
     default:
          DIAG(F("!! ackOp %d FAULT!!"),opcode);
          callback( -1);
          return;

      }  // end of switch
    ackManagerProg++;
  }
}

void DCCACK::callback(int value) {
    // check for automatic retry
    if (value == -1 && ackManagerRetry > 0) {
      ackRetrySum ++;
      LCD(0, F("Retry %d %d Sum=%d"), ackManagerCv, ackManagerRetry, ackRetrySum);
      ackManagerRetry --;
      ackManagerProg = ackManagerProgStart;
      return;
    }

    static unsigned long callbackStart;
    // We are about to leave programming mode
    // Rule 1: If we have written to a decoder we must maintain power for 100mS
    // Rule 2: If we are re-joining the main track we must power off for 30mS

    switch (callbackState) {
      case AFTER_READ:
         if (ackManagerRejoin && !autoPowerOff) {
                progDriver->setPower(POWERMODE::OFF);
                callbackStart=millis();
                callbackState=WAITING_30;
                if (Diag::ACK) DIAG(F("OFF 30mS"));
        } else {
               callbackState=READY;
        }
        break;

      case AFTER_WRITE:  // first attempt to callback after a write operation
	    if (!ackManagerRejoin && !autoPowerOff) {
               callbackState=READY;
               break;
            }                              // lines 906-910 added. avoid wait after write. use 1 PROG
            callbackStart=millis();
            callbackState=WAITING_100;
            if (Diag::ACK) DIAG(F("Stable 100mS"));
            break;

       case WAITING_100:  // waiting for 100mS
            if (millis()-callbackStart < 100) break;
            // stable after power maintained for 100mS

            // If we are going to power off anyway, it doesnt matter
            // but if we will keep the power on, we must off it for 30mS
            if (autoPowerOff) callbackState=READY;
            else { // Need to cycle power off and on
                progDriver->setPower(POWERMODE::OFF);
                callbackStart=millis();
                callbackState=WAITING_30;
                if (Diag::ACK) DIAG(F("OFF 30mS"));
            }
            break;

        case WAITING_30:  // waiting for 30mS with power off
            if (millis()-callbackStart < 30) break;
            //power has been off for 30mS
            progDriver->setPower(POWERMODE::ON);
            callbackState=READY;
            break;

       case READY:  // ready after read, or write after power delay and off period.
            // power off if we powered it on
           if (autoPowerOff) {
              if (Diag::ACK) DIAG(F("Auto Prog power off"));
              progDriver->setPower(POWERMODE::OFF);
              /* TODO 
	      if (MotorDriver::commonFaultPin)
		DCCWaveform::mainTrack.setPowerMode(POWERMODE::OFF);
        **/
           }
          // Restore <1 JOIN> to state before BASELINE
          if (ackManagerRejoin) {
              TrackManager::setJoin(true);
              if (Diag::ACK) DIAG(F("Auto JOIN"));
          }

          ackManagerProg=NULL;  // no more steps to execute
          if (Diag::ACK) DIAG(F("Callback(%d)"),value);
          (ackManagerCallback)( value);
    }
}
#endif

void DCCACK::checkAck(byte sentResetsSincePacket) {
    if (!ackPending) return; 
    // This function operates in interrupt() time so must be fast and can't DIAG 
    if (sentResetsSincePacket > 6) {  //ACK timeout
        ackCheckDuration=millis()-ackCheckStart;
        ackPending = false;
        return; 
    }
      
    int current=progDriver->getCurrentRaw(true); // true means "from interrupt"
    numAckSamples++;
    if (current > ackMaxCurrent) ackMaxCurrent=current;
    // An ACK is a pulse lasting between minAckPulseDuration and maxAckPulseDuration uSecs (refer @haba)
        
    if (current>ackThreshold) {
       if (trailingEdgeCounter > 0) {
	 numAckGaps++;
	 trailingEdgeCounter = 0;
       }
       if (ackPulseStart==0) ackPulseStart=micros();    // leading edge of pulse detected
       return;
    }
    
    // not in pulse
    if (ackPulseStart==0) return; // keep waiting for leading edge 
    
    // if we reach to this point, we have
    // detected trailing edge of pulse
    if (trailingEdgeCounter == 0) {
      ackPulseDuration=micros()-ackPulseStart;
    }

    // but we do not trust it yet and return (which will force another
    // measurement) and first the third time around with low current
    // the ack detection will be finalized. 
    if (trailingEdgeCounter < 2) {
      trailingEdgeCounter++;
      return;
    }
    trailingEdgeCounter = 0;

    if (ackPulseDuration>=minAckPulseDuration && ackPulseDuration<=maxAckPulseDuration) {
        ackCheckDuration=millis()-ackCheckStart;
        ackDetected=true;
        ackPending=false;
        DCCWaveform::progTrack.clearRepeats();  // shortcut remaining repeat packets 
        return;  // we have a genuine ACK result
    }      
    ackPulseStart=0;  // We have detected a too-short or too-long pulse so ignore and wait for next leading edge 
}

