#ifndef DCC_h
#define DCC_h
#include <Arduino.h>
#include "Config.h"


class DCC {
  public:

  static void begin();
  static void loop();

  // Public DCC API functions
  static void setThrottle( uint16_t cab, uint8_t tSpeed, bool tDirection);
  static int  readCV(int cv);
  static bool writeCVByte(int cv, byte bValue) ;
  static bool writeCVBit(int cv, byte bNum, bool bValue);
  static void writeCVByteMain(int cab, int cv, byte bValue);
  static void writeCVBitMain(int cab, int cv, byte bNum, bool bValue);
  static void setFunction( int cab, byte fByte, byte eByte);
  static void setFunction( int cab, byte fByte);
  static void setAccessory(int aAdd, byte aNum, bool activate) ;
  static bool writeTextPacket( byte *b, int nBytes);

private: 
  struct LOCO {
     int loco;
       byte speed;
       bool forward;
  };
  static bool verifyCV(int cv,byte bValue);
  static void setThrottle2( uint16_t cab, uint8_t tSpeed, bool tDirection);
  static void updateLocoReminder(int loco, byte tSpeed, bool forward);
  static int nextLoco;
  static LOCO speedTable[MAX_LOCOS];
  static byte cv1(byte opcode, int cv);
  static byte cv2(int cv);
  
};
#endif
