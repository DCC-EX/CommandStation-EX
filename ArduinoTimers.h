// This file is copied from https://github.com/davidcutting42/ArduinoTimers
// All Credit and copyright David Cutting
// The files included below come from the same source.
// This library had been included with the DCC code to avoid issues with
// library management for inexperienced users. "It just works (TM)"
 
#ifndef ArduinoTimers_h
#define ArduinoTimers_h

#if defined(ARDUINO_AVR_MEGA) || defined(ARDUINO_AVR_MEGA2560)
    #include "ATMEGA2560/Timer.h"
#elif defined(ARDUINO_AVR_UNO)
    #include "ATMEGA328/Timer.h"
#else
    #error "Cannot compile - ArduinoTimers library does not support your board, or you are missing compatible build flags."
#endif

#endif
