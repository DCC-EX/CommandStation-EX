/*
 *  © 2022-2023 Paul M. Antoine
 *  © 2021 Mike S
 *  © 2021-2023 Harald Barth
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
  static void startRailcomTimer(byte brakePin);
  static void ackRailcomTimer();
  static void DCCEXanalogWriteFrequency(uint8_t pin, uint32_t frequency);
  static void DCCEXanalogWrite(uint8_t pin, int value);

// Update low ram level.  Allow for extra bytes to be specified
// by estimation or inspection, that may be used by other 
// called subroutines.  Must be called with interrupts disabled.
// 
// Although __brkval may go up and down as heap memory is allocated
// and freed, this function records only the worst case encountered.
// So even if all of the heap is freed, the reported minimum free 
// memory will not increase.
//
  static void inline updateMinimumFreeMemoryISR(unsigned char extraBytes=0)
    __attribute__((always_inline)) {
    int spare = freeMemory()-extraBytes;
    if (spare < 0) spare = 0;
    if (spare < minimum_free_memory) minimum_free_memory = spare;
  };

  static int  getMinimumFreeMemory();
  static void reset();
  
private:
  static int freeMemory();
  static volatile int minimum_free_memory;
  static const int DCC_SIGNAL_TIME=58;  // this is the 58uS DCC 1-bit waveform half-cycle 
#if defined(ARDUINO_ARCH_STM32)  // TODO: PMA temporary hack - assumes 100Mhz F_CPU as STM32 can change frequency
  static const long CLOCK_CYCLES=(100000000L / 1000000 * DCC_SIGNAL_TIME) >>1;
#else
  static const long CLOCK_CYCLES=(F_CPU / 1000000 * DCC_SIGNAL_TIME) >>1;
#endif

};

// Class ADCee implements caching of the ADC value for platforms which
// have a too slow ADC read to wait for. On these platforms the ADC is
// scanned continiously in the background from an ISR. On such
// architectures that use the analog read during DCC waveform with
// specially configured ADC, for example AVR, init must be called
// PRIOR to the start of the waveform. It returns the current value so
// that an offset can be initialized.
class ADCee {
public:
  // begin is called for any setup that must be done before
  // **init** can be called. On some architectures this involves ADC
  // initialisation and clock routing, sampling times etc.
  static void begin();
  // init adds the pin to the list of scanned pins (if this
  // platform's implementation scans pins) and returns the first
  // read value (which is why it required begin to have been called first!)
  // It must be called before the regular scan is started.
  static int init(uint8_t pin);
  // read does read the pin value from the scanned cache or directly
  // if this is a platform that does not scan. fromISR is a hint if
  // it was called from ISR because for some implementations that
  // makes a difference.
  static int read(uint8_t pin, bool fromISR=false);
  // returns possible max value that the ADC can return
  static int16_t ADCmax();
private:
  // On platforms that scan, it is called from waveform ISR
  // only on a regular basis.
  static void scan();
  #if defined (ARDUINO_ARCH_STM32)
  // bit array of used pins (max 32)
  static uint32_t usedpins;
#else
  // bit array of used pins (max 16)
  static uint16_t usedpins;
#endif
  static uint8_t highestPin;
  // cached analog values (malloc:ed to actual number of ADC channels)
  static int *analogvals;
  // friend so that we can call scan() and begin()
  friend class DCCWaveform;
  };
#endif
