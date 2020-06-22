#include "DCC.h"
#include "DCCWaveform.h"
#include "DIAG.h"
#include "Hardware.h"

// This module is responsible for converting API calls into
// messages to be sent to the waveform generator.
// It has no visibility of the hardware, timers, interrupts
// nor of the waveform issues such as preambles, start bits checksums or cutouts.
//
// Nor should it have to deal with JMRI responsess other than the OK/FAIL
// or cv value returned. I will move that back to the JMRI interface later
//
// The interface to the waveform generator is narrowed down to merely:
//   Scheduling a message on the prog or main track using a function
//   Obtaining ACKs from the prog track using a function
//   There are no volatiles here.

const byte FN_GROUP_1=0x01;         
const byte FN_GROUP_2=0x02;         
const byte FN_GROUP_3=0x04;         
const byte FN_GROUP_4=0x08;         
const byte FN_GROUP_5=0x10;         


void DCC::begin() {
  DCCWaveform::begin();
}

void DCC::setThrottle( uint16_t cab, uint8_t tSpeed, bool tDirection)  {
  byte speedCode = tSpeed + (tSpeed > 0) + tDirection * 128; // max speed is 126, but speed codes range from 2-127 (0=stop, 1=emergency stop)
  setThrottle2(cab, speedCode);
  // retain speed for loco reminders
  updateLocoReminder(cab, speedCode );
}

void DCC::setThrottle2( uint16_t cab, byte speedCode)  {

  uint8_t b[4];
  uint8_t nB = 0;

  if (cab > 127)
    b[nB++] = highByte(cab) | 0xC0;    // convert train number into a two-byte address
  b[nB++] = lowByte(cab);
  b[nB++] = SET_SPEED;                      // 128-step speed control byte
  b[nB++] = speedCode; // for encoding see setThrottle

  DCCWaveform::mainTrack.schedulePacket(b, nB, 0);
}

void DCC::setFunctionInternal(int cab, byte byte1, byte byte2) {
  //DIAG(F("\nsetFunctionInternal %d %x %x"),cab,byte1,byte2);
  byte b[4];
  byte nB = 0;

  if (cab > 127)
    b[nB++] = highByte(cab) | 0xC0;    // convert train number into a two-byte address
  b[nB++] = lowByte(cab);
  if (byte1!=0) b[nB++] = byte1;
  b[nB++] = byte2;

  DCCWaveform::mainTrack.schedulePacket(b, nB, 3);     // send packet 3 times
}

static void DCC::setFn( int cab, byte functionNumber, bool on) {
  if (cab<=0 || functionNumber<0 || functionNumber>28) return;
  int reg = lookupSpeedTable(cab);
  if (reg<0) return;  
    // set the function on/off in the functions and set the group flag to
    // say we have touched the particular group.
    // A group will be reminded only if it has been touched.  
      if (on) speedTable[reg].functions |= (1L<<functionNumber);
         else speedTable[reg].functions &= ~(1L<<functionNumber);
      byte groupMask;
      if (functionNumber<=4)       groupMask=FN_GROUP_1;
      else if (functionNumber<=8)  groupMask=FN_GROUP_2;
      else if (functionNumber<=12) groupMask=FN_GROUP_3;
      else if (functionNumber<=20) groupMask=FN_GROUP_4;
      else                         groupMask=FN_GROUP_5;
      speedTable[reg].groupFlags |= groupMask; 
}

void DCC::setAccessory(int address, byte number, bool activate) {
  byte b[2];

  b[0] = address % 64 + 128;                                     // first byte is of the form 10AAAAAA, where AAAAAA represent 6 least signifcant bits of accessory address
  b[1] = ((((address / 64) % 8) << 4) + (number % 4 << 1) + activate % 2) ^ 0xF8; // second byte is of the form 1AAACDDD, where C should be 1, and the least significant D represent activate/deactivate

  DCCWaveform::mainTrack.schedulePacket(b, 2, 4);      // Repeat the packet four times
}

void DCC::writeCVByteMain(int cab, int cv, byte bValue)  {
  byte b[5];
  byte nB = 0;
  if (cab > 127)
    b[nB++] = highByte(cab) | 0xC0;    // convert train number into a two-byte address

  b[nB++] = lowByte(cab);
  b[nB++] = cv1(WRITE_BYTE_MAIN, cv); // any CV>1023 will become modulus(1024) due to bit-mask of 0x03
  b[nB++] = cv2(cv);
  b[nB++] = bValue;

  DCCWaveform::mainTrack.schedulePacket(b, nB, 4);
}

void DCC::writeCVBitMain(int cab, int cv, byte bNum, bool bValue)  {
  byte b[5];
  byte nB = 0;
  bValue = bValue % 2;
  bNum = bNum % 8;

  if (cab > 127)
    b[nB++] = highByte(cab) | 0xC0;    // convert train number into a two-byte address

  b[nB++] = lowByte(cab);
  b[nB++] = cv1(WRITE_BIT_MAIN, cv); // any CV>1023 will become modulus(1024) due to bit-mask of 0x03
  b[nB++] = cv2(cv);
  b[nB++] = WRITE_BIT | (bValue ? BIT_ON : BIT_OFF) | bNum;

  DCCWaveform::mainTrack.schedulePacket(b, nB, 4);
}



const ackOp PROGMEM WRITE_BIT0_PROG[] = {
     BASELINE,
     W0,WACK,
     V0, WACK,  // validate bit is 0 
     ITC1,      // if acked, callback(1)
     FAIL  // callback (-1)
};
const ackOp PROGMEM WRITE_BIT1_PROG[] = {
     BASELINE,
     W1,WACK,
     V1, WACK,  // validate bit is 1 
     ITC1,      // if acked, callback(1)
     FAIL  // callback (-1)
};


const ackOp PROGMEM READ_BIT_PROG[] = {
     BASELINE,
     V1, WACK,  // validate bit is 1 
     ITC1,      // if acked, callback(1)
     V0, WACK,  // validate bit is zero
     ITC0,      // if acked callback 0
     FAIL       // bit not readable 
     };
     
const ackOp PROGMEM WRITE_BYTE_PROG[] = {
      BASELINE,
      WB,WACK,    // Write 
      VB,WACK,     // validate byte 
      ITC1,       // if ok callback (1)
      FAIL        // callback (-1)
      };
      
      
const ackOp PROGMEM READ_CV_PROG[] = {
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
      VB, WACK, ITCB,  // verify merged byte and return it if acked ok 
      FAIL };          // verification failed


const ackOp PROGMEM LOCO_ID_PROG[] = {
      BASELINE,
      SETCV,(ackOp)29,
      SETBIT,(ackOp)5,
      V0, WACK, ITSKIP,  // Skip to SKIPTARGET if bit 5 of CV29 is zero
      // Long locoid  
      SETCV, (ackOp)17,       // CV 17 is part of locoid
      STARTMERGE,
      V0, WACK, MERGE,  // read and merge bit 1 etc
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      VB, WACK, NAKFAIL,  // verify merged byte and return -1 it if not acked ok
      STASHLOCOID,         // keep stashed cv 17 for later 
      // Read 2nd part from CV 18 
      SETCV, (ackOp)18,
      STARTMERGE,
      V0, WACK, MERGE,  // read and merge bit 1 etc
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      VB, WACK, NAKFAIL,  // verify merged byte and return -1 it if not acked ok
      COMBINELOCOID,        // Combile byte with stash to make long locoid and callback
      
      // ITSKIP Skips to here if CV 29 bit 5 was zero. so read CV 1 and return that  
      SKIPTARGET,
      SETCV, (ackOp)1,
      STARTMERGE,
      V0, WACK, MERGE,  // read and merge bit 1 etc
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      VB, WACK, ITCB,  // verify merged byte and callback
      FAIL
      };    



void  DCC::writeCVByte(int cv, byte byteValue, ACK_CALLBACK callback)  {
  ackManagerSetup(cv, byteValue,  WRITE_BYTE_PROG, callback);
}


void DCC::writeCVBit(int cv, byte bitNum, bool bitValue, ACK_CALLBACK callback)  {
  if (bitNum >= 8) callback(-1);
  else ackManagerSetup(cv, bitNum, bitValue?WRITE_BIT1_PROG:WRITE_BIT0_PROG, callback);
}


void DCC::readCVBit(int cv, byte bitNum, ACK_CALLBACK callback)  {
  if (bitNum >= 8) callback(-1);
  else ackManagerSetup(cv, bitNum,READ_BIT_PROG, callback);
}

void DCC::readCV(int cv, ACK_CALLBACK callback)  {
  ackManagerSetup(cv, 0,READ_CV_PROG, callback);
}

void DCC::getLocoId(ACK_CALLBACK callback) {
  ackManagerSetup(0,0, LOCO_ID_PROG, callback);
}

void DCC::forgetLoco(int cab) {  // removes any speed reminders for this loco  
  int reg=lookupSpeedTable(cab);
  if (reg>=0) speedTable[reg].loco=0;
}
void DCC::forgetAllLocos() {  // removes all speed reminders
  for (int i=0;i<MAX_LOCOS;i++) speedTable[i].loco=0;  
}

byte DCC::loopStatus=0;  

void DCC::loop()  {
  DCCWaveform::loop(); // power overload checks
  ackManagerLoop();    // maintain prog track ack manager
  issueReminders();
}

void DCC::issueReminders() {
  // if the main track transmitter still has a pending packet, skip this time around.
  if ( DCCWaveform::mainTrack.packetPending) return;

  // This loop searches for a loco in the speed table starting at nextLoco and cycling back around
  for (int reg=0;reg<MAX_LOCOS;reg++) {
       int slot=reg+nextLoco;
       if (slot>=MAX_LOCOS) slot-=MAX_LOCOS; 
       if (speedTable[slot].loco > 0) {
          // have found the next loco to remind 
          // issueReminder will return true if this loco is completed (ie speed and functions)
          if (issueReminder(slot)) nextLoco=slot+1; 
          return;
        }
  }
}
 
bool DCC::issueReminder(int reg) {
  long functions=speedTable[reg].functions;
  int loco=speedTable[reg].loco;
  byte flags=speedTable[reg].groupFlags;
  
  switch (loopStatus) {
        case 0:
      //   DIAG(F("\nReminder %d speed %d"),loco,speedTable[reg].speedCode);
         setThrottle2(loco, speedTable[reg].speedCode);
         break;
       case 1: // remind function group 1 (F0-F4)
          if (flags & FN_GROUP_1) 
              setFunctionInternal(loco,0, 128 + ((functions>>1)& 0x0F) | (functions & 0x01)<<4);   
          break;     
       case 2: // remind function group 2 F5-F8
          if (flags & FN_GROUP_2) 
              setFunctionInternal(loco,0, 176 + ((functions>>5)& 0x0F));   
          break;     
       case 3: // remind function group 3 F9-F12
          if (flags & FN_GROUP_3) 
              setFunctionInternal(loco,0, 160 + ((functions>>9)& 0x0F));
          break;   
       case 4: // remind function group 4 F13-F20
          if (flags & FN_GROUP_4) 
              setFunctionInternal(loco,222, ((functions>>13)& 0xFF)); 
          break;  
       case 5: // remind function group 5 F21-F28
          if (flags & FN_GROUP_5)
              setFunctionInternal(loco,223, ((functions>>21)& 0xFF)); 
          break; 
      }
      loopStatus++;
      // if we reach status 6 then this loco is done so
      // reset status to 0 for next loco and return true so caller 
      // moves on to next loco. 
      if (loopStatus>5) loopStatus=0;
      return loopStatus==0;
    }
 
 


///// Private helper functions below here /////////////////////

byte DCC::cv1(byte opcode, int cv)  {
  cv--;
  return (highByte(cv) & (byte)0x03) | opcode;
}
byte DCC::cv2(int cv)  {
  cv--;
  return lowByte(cv);
}

int DCC::lookupSpeedTable(int locoId) {
  // determine speed reg for this loco
  int firstEmpty = MAX_LOCOS;
  int reg;
  for (reg = 0; reg < MAX_LOCOS; reg++) {
    if (speedTable[reg].loco == locoId) break;
    if (speedTable[reg].loco == 0 && firstEmpty == MAX_LOCOS) firstEmpty = reg;
  }
  if (reg == MAX_LOCOS) reg = firstEmpty;
  if (reg >= MAX_LOCOS) {
    DIAG(F("\nToo many locos\n"));
    return -1;
  }
  if (reg==firstEmpty){
        speedTable[reg].loco = locoId;
        speedTable[reg].speedCode=0;
        speedTable[reg].groupFlags=0;
        speedTable[reg].functions=0;
  }
  return reg;
}
  
void  DCC::updateLocoReminder(int loco, byte speedCode) {
 
  if (loco==0) {
     // broadcast message
     for (int reg = 0; reg < MAX_LOCOS; reg++) speedTable[reg].speedCode = speedCode;
     return; 
  }
  
  // determine speed reg for this loco
  int reg=lookupSpeedTable(loco);       
  if (reg>=0) speedTable[reg].speedCode = speedCode;
}

DCC::LOCO DCC::speedTable[MAX_LOCOS];
int DCC::nextLoco = 0;

//ACK MANAGER
ackOp  const *  DCC::ackManagerProg;
byte   DCC::ackManagerByte;
byte   DCC::ackManagerStash;
int   DCC::ackManagerCv;
byte   DCC::ackManagerBitNum;
bool   DCC::ackReceived;
int   DCC::ackTriggerMilliamps;
unsigned long   DCC::ackPulseStart;

ACK_CALLBACK DCC::ackManagerCallback;

void  DCC::ackManagerSetup(int cv, byte byteValueOrBitnum, ackOp const program[], ACK_CALLBACK callback) {
  ackManagerCv = cv;
  ackManagerProg = program;
  ackManagerByte = byteValueOrBitnum;
  ackManagerBitNum=byteValueOrBitnum;
  ackManagerCallback = callback;

}

const byte RESET_MIN=8;  // tuning of reset counter before sending message

void DCC::ackManagerLoop() {
  while (ackManagerProg) {

    // breaks from this switch will step to next prog entry
    // returns from this switch will stay on same entry (typically WACK waiting and when all finished.)
    byte opcode=pgm_read_byte_near(ackManagerProg);
    // DIAG(F("apAck %d\n"),opcode);
    int resets=DCCWaveform::progTrack.sentResetsSincePacket;
    int current; 
     
    switch (opcode) {
      case BASELINE:
          if (resets<RESET_MIN) return; // try later 
          ackTriggerMilliamps=Hardware::getCurrentMilliamps(false) + ACK_MIN_PULSE;
          // DIAG(F("\nBASELINE trigger mA=%d\n"),ackTriggerMilliamps);
          break;

      case W0:    // write 0 bit 
      case W1:    // write 1 bit 
            {
              if (resets<RESET_MIN) return; // try later
              byte instruction = WRITE_BIT | (opcode==W1 ? BIT_ON : BIT_OFF) | ackManagerBitNum;
              byte message[] = {cv1(BIT_MANIPULATE, ackManagerCv), cv2(ackManagerCv), instruction };
              DCCWaveform::progTrack.schedulePacket(message, sizeof(message), 6);
              ackPulseStart=0; 
             }
            break; 
      
      case WB:   // write byte 
            {
              if (resets<RESET_MIN) return; // try later 
              byte message[] = {cv1(WRITE_BYTE, ackManagerCv), cv2(ackManagerCv), ackManagerByte};
              DCCWaveform::progTrack.schedulePacket(message, sizeof(message), 6);
              ackPulseStart=0; 
            }
            break;
      
      case   VB:     // Issue validate Byte packet
        {
          if (resets<RESET_MIN) return; // try later 
          // DIAG(F("\nVB %d %d"),ackManagerCv,ackManagerByte);
          byte message[] = { cv1(VERIFY_BYTE, ackManagerCv), cv2(ackManagerCv), ackManagerByte};
          DCCWaveform::progTrack.schedulePacket(message, sizeof(message), 5);
          ackPulseStart=0; 
        }
        break;
      
      case V0:
      case V1:      // Issue validate bit=0 or bit=1  packet
        {
          if (resets<RESET_MIN) return; // try later 
          // DIAG(F("V%d cv=%d bit=%d"),opcode==V1, ackManagerCv,ackManagerBitNum); 
          byte instruction = VERIFY_BIT | (opcode==V0?BIT_OFF:BIT_ON) | ackManagerBitNum;
          byte message[] = {cv1(BIT_MANIPULATE, ackManagerCv), cv2(ackManagerCv), instruction };
          DCCWaveform::progTrack.schedulePacket(message, sizeof(message), 5);
          ackPulseStart=0; 
        }
        break;
      
      case WACK:   // wait for ack (or absence of ack)
          
          if (resets > 6) {  //ACK timeout
            // DIAG(F("\nWACK fail %d\n"), resets);
            ackReceived = false;
            break; // move on to next prog step
            }
      
        current=Hardware::getCurrentMilliamps(false);
        // An ACK is a pulse lasting between 4.5 and 8.5 mSecs (refer @haba)
        
        if (current>ackTriggerMilliamps) {
          if (ackPulseStart==0)ackPulseStart=micros();    // leading edge of pulse detected
          return;
        }
      
        // not in pulse
        if (ackPulseStart==0) return; // keep waiting for leading edge 
        { // detected trailing edge of pulse
          long pulseDuration=micros()-ackPulseStart;       
          if (pulseDuration>4500 && pulseDuration<8000) {
            ackReceived=true;
            DCCWaveform::progTrack.killRemainingRepeats(); // probably no need after 8.5ms!!
            break;  // we have a genuine ACK result
          }
        }
          ackPulseStart=0;  // We have detected a too-short or too-long pulse so ignore and wait for next leading edge 
          return; // keep waiting 
        
     case ITC0:
     case ITC1:   // If True Callback(0 or 1)  (if prevous WACK got an ACK)
        if (ackReceived) {
            ackManagerProg = NULL; // all done now
            (ackManagerCallback)(opcode==ITC0?0:1);
            return;
          }
        break;
        
      case ITCB:   // If True callback(byte)
          if (ackReceived) {
            ackManagerProg = NULL; // all done now
            (ackManagerCallback)(ackManagerByte);
            return;
          }
        break;
        
      case NAKFAIL:   // If nack callback(-1)
          if (!ackReceived) {
            ackManagerProg = NULL; // all done now
            (ackManagerCallback)(-1);
            return;
          }
        break;
        
      case FAIL:  // callback(-1)
           ackManagerProg = NULL;
           (ackManagerCallback)(-1);
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

      case SETBIT:
          ackManagerProg++; 
          ackManagerBitNum=pgm_read_byte_near(ackManagerProg);
          break;

     case SETCV:
          ackManagerProg++; 
          ackManagerCv=pgm_read_byte_near(ackManagerProg);
          break;

     case STASHLOCOID:
          ackManagerStash=ackManagerByte;  // stash value from CV17 
          break;
          
     case COMBINELOCOID: 
          // ackManagerStash is  cv17, ackManagerByte is CV 18
          ackManagerProg=NULL;
          (ackManagerCallback)( ackManagerByte + ((ackManagerStash - 192) << 8));
          return;            

     case ITSKIP:
          if (!ackReceived) break; 
          // SKIP opcodes until SKIPTARGET found
          while (opcode!=SKIPTARGET) {
            ackManagerProg++; 
            opcode=pgm_read_byte_near(ackManagerProg);
          }
          // DIAG(F("\nSKIPTARGET located\n"));
          break;
     case SKIPTARGET: 
          break;     
     default: 
          // DIAG(F("!! ackOp %d FAULT!!"),opcode);
          ackManagerProg=NULL;
          (ackManagerCallback)( -1);
          return;        
    
      }  // end of switch
    ackManagerProg++;
  }
}
