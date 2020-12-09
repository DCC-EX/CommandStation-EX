#ifndef FSH_h
#define FSH_h
#include <Arduino.h>
#if defined(ARDUINO_ARCH_MEGAAVR)
typedef char FSH; 
#else 
typedef __FlashStringHelper FSH;
#endif
#endif
