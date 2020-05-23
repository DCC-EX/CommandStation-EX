#include "DCCWaveform.h"
#include "DIAG.h"
bool verifyCV(int cv, byte bValue);
int readCv(int cv);
const int cvnums[]={1,2,3,4,5,17,18,19,21,22,29};

void setup() {
  Serial.begin(115200);
  DCCWaveform::begin();
  Serial.println(F("CVReader"));

  DIAG(F("\n===================================\n"));
  
  for (byte x=0;x<sizeof(cvnums)/sizeof(cvnums[0]);x++) {
    DIAG(F("\n\nCV VERIFICATION %d = %s\n"),cvnums[x],readCV(cvnums[x])>=0?"OK":"FAIL"); 
  }
  
 DIAG(F("\nProgram complete, press reset to retry"));
}

void loop() {
  
}

byte cv1(byte opcode, int cv) {
  cv--;
  return (highByte(cv) & (byte)0x03) | opcode;
}
byte cv2(int cv) {
  cv--;
  return lowByte(cv);
}

//// The functions below are lifted from the DCCApi for easy testing and experimentation.
// Once reliable, tha DCCApi should be updated to match 
bool verifyCV(int cv, byte value) {
   
   delay(2);  // allow for decoder to quiesce latest pulse
   
   
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
    DCCWaveform::progTrack.schedulePacket(message,sizeof(message), 4);                // NMRA recommends 5 rerad packets
    value+= (DCCWaveform::progTrack.getAck()<<i);
  }
  DIAG(F("\n*** READ CV %d = %d ******\n"),cv,value);
  return verifyCV(cv,value)?value:-1;
}
