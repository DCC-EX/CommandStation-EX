#ifndef FSH_h
#define FSH_h
#include <Arduino.h>
#if defined(ARDUINO_ARCH_MEGAAVR)
#ifdef F
#undef F
#define F(str) (str)
#endif
typedef char FSH; 
#define GETFLASH(addr) (*(const unsigned char *)(addr))
#define FLASH
#else 
typedef __FlashStringHelper FSH;
#define GETFLASH(addr) pgm_read_byte_near(addr)
#define FLASH PROGMEM
#endif
#endif
