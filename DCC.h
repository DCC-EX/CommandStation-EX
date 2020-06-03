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
  static int  readCVBit(int cv, byte bNum);  // -1 for error
  static bool writeCVByte(int cv, byte bValue) ;
  static bool verifyCVByte(int cv,byte bValue);
  static bool writeCVBit(int cv, byte bNum, bool bValue);
  static bool verifyCVBit(int cv, byte bNum, bool bValue);
  static void writeCVByteMain(int cab, int cv, byte bValue);
  static void writeCVBitMain(int cab, int cv, byte bNum, bool bValue);
  static void setFunction( int cab, byte fByte, byte eByte);
  static void setFunction( int cab, byte fByte);
  static void setAccessory(int aAdd, byte aNum, bool activate) ;
  static bool writeTextPacket( byte *b, int nBytes);
  static int getLocoId();

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
