#include "DCCWaveform.h"
#include "DIAG.h"

/* this code is here to test the waveforwe generator and reveal the issues involved in programming track operations.
 *  
 *  It tests the Waveform genartor and demonstrates how a DCC  API function can be simply written  
 *  to transmit and receive DCC data on the programming track.
 *  
 *  Important... DCCWaveform.h contains hardware specific confioguration settings 
 *  that you will need to check.  
 *  
 *  Notes: the waveform generator sends reset packets on progTrack when it has nothing better to do, this means that
 *  the decoder does not step out of service mode. (The main track sends idles).
 *  It also means that the API functions dont have to mess with reset packets. 
 *  
 */


bool verifyCV(int cv, byte bValue);
int readCv(int cv);
const int cvnums[]={1,2,3,4,5,17,18,19,21,22,29};

void setup() {
  Serial.begin(115200);
  DCCWaveform::begin();
  
  DIAG(F("\n===== CVReader begin ==============================\n"));
  
  for (byte x=0;x<sizeof(cvnums)/sizeof(cvnums[0]);x++) {
    int value=readCV(cvnums[x]);
    DIAG(F("\nCV %d = %d  0x%x  %s"),cvnums[x],value,value, value>=0?" VERIFIED OK":"FAILED VERIFICATION"); 
  }
  
 DIAG(F("\nProgram complete, press reset to retry"));
}

void loop() {
  DCCWaveform::loop();
  }

// Two helpers to make API functions look simpler

byte cv1(byte opcode, int cv) {
  cv--;
  return (highByte(cv) & (byte)0x03) | opcode;
}
byte cv2(int cv) {
  cv--;
  return lowByte(cv);
}

// The functions below are lifted from the DCCApi and then fixed up 
// Once reliable, tha DCCApi should be updated to match 
bool verifyCV(int cv, byte value) {
   byte message[] = {
    cv1(0x74,cv)  ,   // set-up to re-verify entire byte
    cv2(cv),
    value
    };
  
  DCCWaveform::progTrack.schedulePacket(message, sizeof(message), 5);  
  return DCCWaveform::progTrack.getAck();
}

int readCV(int cv) 
{
  
  byte message[]={  cv1(0x78,cv)  ,   // any CV>1023 will become modulus(1024) due to bit-mask of 0x03
                   cv2(cv),
                   0};    // trailing zero will be updated in loop below

  byte value = 0;

  for (int i = 0; i<8; i++) {
    message[2] = 0xE8 + i;
    DCCWaveform::progTrack.schedulePacket(message,sizeof(message), 4);                // NMRA recommends 5 read packets
    value+= (DCCWaveform::progTrack.getAck()<<i);
  }
  
  //  DIAG(F("\n*** readCV(%d) = %d ******\n"),cv,value);
  return verifyCV(cv,value)?value:-1;
}
