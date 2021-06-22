#ifndef FSH_h
#define FSH_h

/* This is an architecture support file to manage the differences 
 *  between the nano/uno.mega and the later nanoEvery, unoWifiRev2 etc
 *  
 *  IMPORTANT:
 *  To maintain portability the main code should NOT contain ANY references 
 *  to the following: 
 *  
 *  __FlashStringHelper     Use FSH instead.
 *  PROGMEM                 use FLASH instead
 *  pgm_read_byte_near      use GETFLASH instead.
 *  pgm_read_word_near      use GETFLASHW instead.
 *  
 */
#include <Arduino.h>
#if defined(ARDUINO_ARCH_MEGAAVR)
#ifdef F
  #undef F
#endif
#define F(str) (str)
typedef char FSH; 
#define GETFLASH(addr) (*(const unsigned char *)(addr))
#define GETFLASHW(addr) (*(const unsigned short *)(addr))
#define FLASH
#define strlen_P strlen
#define strcpy_P strcpy
#else 
typedef __FlashStringHelper FSH;
#define GETFLASH(addr) pgm_read_byte_near(addr)
#define GETFLASHW(addr) pgm_read_word_near(addr)
#define FLASH PROGMEM
#endif
#endif
