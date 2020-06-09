#ifndef DCC_h
#define DCC_h
#include <Arduino.h>
#include "Config.h"

typedef void (*ACK_CALLBACK)(int result);

enum ackOp {  // Program opcodes for the ack Manager
BASELINE,  // ensure enough resets sent before starting and obtain baseline current
W0,W1,   // issue write bit (0..1) packet 
WB,     // issue write byte packet 
VB,     // Issue validate Byte packet
V0,     // Issue validate bit=0 packet
V1,     // issue validate bit=1 packlet
WACK,   // wait for ack (or absence of ack)
ITC1,   // If True Callback(1)  (if prevous WACK got an ACK)
ITC0,   // If True callback(0);
ITCB,   // If True callback(byte)
NAKFAIL, // if false callback(-1)
FAIL,   // callback(-1)
STARTMERGE,    // Clear bit and byte settings ready for merge pass 
MERGE,  // Merge previous wack response with byte value and decrement bit number (use for readimng CV bytes)
SETBIT, // sets bit number to next prog byte
SETCV, // sets cv number to next prog byte
STASHLOCOID, // keeps current byte value for later
COMBINELOCOID, // combines current value with stashed value and returns it
ITSKIP,        // skip to SKIPTARGET if ack true
SKIPTARGET=0xFF  // jump to target
};

class DCC {
  public:

  static void begin();
  static void loop();

  // Public DCC API functions
  static void setThrottle( uint16_t cab, uint8_t tSpeed, bool tDirection);
  static void writeCVByteMain(int cab, int cv, byte bValue);
  static void writeCVBitMain(int cab, int cv, byte bNum, bool bValue);
  static void setFunction( int cab, byte fByte, byte eByte);
  static void setFunction( int cab, byte fByte);
  static void setAccessory(int aAdd, byte aNum, bool activate) ;
  static bool writeTextPacket( byte *b, int nBytes);
  
  // ACKable progtrack calls  bitresults callback 0,0 or -1, cv returns value or -1 
  static void  readCV(int cv, ACK_CALLBACK callback);
  static void  readCVBit(int cv, byte bitNum, ACK_CALLBACK callback);  // -1 for error
  static void writeCVByte(int cv, byte byteValue, ACK_CALLBACK callback) ;
  static void writeCVBit(int cv, byte bitNum, bool bitValue, ACK_CALLBACK callback);
  
  static void getLocoId(ACK_CALLBACK callback);

private: 
  struct LOCO {
     int loco;
     byte speedCode;
  };
  static void setThrottle2( uint16_t cab, uint8_t speedCode);
  static void updateLocoReminder(int loco, byte speedCode);
  static int nextLoco;
  static LOCO speedTable[MAX_LOCOS];
  static byte cv1(byte opcode, int cv);
  static byte cv2(int cv);


// ACK MANAGER
  static ackOp  const * ackManagerProg;
  static byte   ackManagerByte;
  static byte   ackManagerBitNum;
  static int    ackManagerCv;
  static byte    ackManagerStash;
  static bool ackReceived;
  static int ackTriggerMilliamps;
  static long ackPulseStart;
  static ACK_CALLBACK  ackManagerCallback;
  static void ackManagerSetup(int cv, byte bitNumOrbyteValue, ackOp const program[], ACK_CALLBACK callback);
  static void ackManagerLoop();

  

  // NMRA codes #
  static const byte SET_SPEED=0x3f;
  static const byte WRITE_BYTE_MAIN = 0xEC;
  static const byte WRITE_BIT_MAIN = 0xE8;
  static const byte WRITE_BYTE = 0x7C;
  static const byte VERIFY_BYTE= 0x74;
  static const byte BIT_MANIPULATE=0x78;
  static const byte WRITE_BIT=0xF0;
  static const byte VERIFY_BIT=0xE0;
  static const byte BIT_ON=0x08;
  static const byte BIT_OFF=0x00;
};

#endif
