/*
 *  © 2021 Mike S
 *  © 2021 Harald Barth
 *  © 2021 Fred Decker
 *  All rights reserved.
 *  
 *  This file is part of CommandStation-EX
 *
 *  This is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  It is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with CommandStation.  If not, see <https://www.gnu.org/licenses/>.
 */

/* There are several different implementations of this class which the compiler will select 
   according to the hardware.
   */

/* This timer class is used to manage the single timer required to handle the DCC waveform.
 *  All timer access comes through this class so that it can be compiled for 
 *  various hardware CPU types. 
 *  
 *  DCCEX works on a single timer interrupt at a regular 58uS interval.
 *  The DCCWaveform class generates the signals to the motor shield  
 *  based on this timer. 
 *  
 *  If the motor drivers are BOTH configured to use the correct 2 pins for the architecture,
 *  (see isPWMPin() function. )
 *  then this allows us to use a hardware driven pin switching arrangement which is
 *  achieved by setting the duty cycle of the NEXT clock interrupt to 0% or 100% depending on 
 *  the required pin state. (see setPWM())  
 *  This is more accurate than the software interrupt but at the expense of 
 *  limiting the choice of available pins. 
 *  Fortunately, a standard motor shield on a Mega uses pins that qualify for PWM... 
 *  Other shields may be jumpered to PWM pins or run directly using the software interrupt.
 *  
 *  Because the PWM-based waveform is effectively set half a cycle after the software version,
 *  it is not acceptable to drive the two tracks on different methiods or it would cause
 *  problems for <1 JOIN> etc.
 *  
 */

#ifndef DCCTimer_h
#define DCCTimer_h
#include "Arduino.h"

typedef void (*INTERRUPT_CALLBACK)();

class DCCTimer {
  public:
  static void begin(INTERRUPT_CALLBACK interrupt);
  static void getSimulatedMacAddress(byte mac[6]);
  static bool isPWMPin(byte pin);
  static void setPWM(byte pin, bool high);
  static void clearPWM();
// Update low ram level.  Allow for extra bytes to be specified
// by estimation or inspection, that may be used by other 
// called subroutines.  Must be called with interrupts disabled.
// 
// Although __brkval may go up and down as heap memory is allocated
// and freed, this function records only the worst case encountered.
// So even if all of the heap is freed, the reported minimum free 
// memory will not increase.
//
static void inline updateMinimumFreeMemoryISR(unsigned char extraBytes=0) {
  int spare = freeMemory()-extraBytes;
  if (spare < 0) spare = 0;
  if (spare < minimum_free_memory) minimum_free_memory = spare;
}

  static int  getMinimumFreeMemory();

private:
  static int freeMemory();
  static volatile int minimum_free_memory;
  static const int DCC_SIGNAL_TIME=58;  // this is the 58uS DCC 1-bit waveform half-cycle 
  static const long CLOCK_CYCLES=(F_CPU / 1000000 * DCC_SIGNAL_TIME) >>1;

};

////////////////////////////////////////////////////////////////////////////////
// Create a cpu type we can share and 
// gigure out if we have enough memory for advanced features
// so define HAS_ENOUGH_MEMORY until proved otherwise.
#define HAS_ENOUGH_MEMORY
#define HAS_AVR_WDT

#if defined(ARDUINO_AVR_UNO)
#define ARDUINO_TYPE "UNO"
#undef HAS_ENOUGH_MEMORY
#elif defined(ARDUINO_AVR_NANO)
#define ARDUINO_TYPE "NANO"
#undef HAS_ENOUGH_MEMORY
#elif defined(ARDUINO_AVR_MEGA)
#define ARDUINO_TYPE "MEGA"
#elif defined(ARDUINO_AVR_MEGA2560)
#define ARDUINO_TYPE "MEGA"
#elif defined(ARDUINO_ARCH_MEGAAVR)
#define ARDUINO_TYPE "MEGAAVR"
#elif defined(ARDUINO_TEENSY32)
#define ARDUINO_TYPE "TEENSY32"
#elif defined(ARDUINO_TEENSY35)
#define ARDUINO_TYPE "TEENSY35"
#elif defined(ARDUINO_TEENSY36)
#define ARDUINO_TYPE "TEENSY36"
#elif defined(ARDUINO_TEENSY40)
#define ARDUINO_TYPE "TEENSY40"
#elif defined(ARDUINO_TEENSY41)
#define ARDUINO_TYPE "TEENSY41"
#elif defined(ARDUINO_ARCH_ESP8266)
#define ARDUINO_TYPE "ESP8266"
#undef HAS_AVR_WDT
#elif defined(ARDUINO_ARCH_ESP32)
#define ARDUINO_TYPE "ESP32"
#undef HAS_AVR_WDT
#else
#error CANNOT COMPILE - DCC++ EX ONLY WORKS WITH THE ARCHITECTURES LISTED IN DCCTimer.h
#endif
#endif
