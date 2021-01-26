/*
 *  © 2021, Chris Harlow & David Cutting. All rights reserved.
 *  
 *  This file is part of Asbelos DCC API
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


/* This timer class is used to manage the single timer required to handle the DCC waveform.
 *  All timer access comes through this class so that it can be compiled for 
 *  various hardware CPU types. 
 *  
 *  DCCEX works on a single timer interrupt at a regular 58uS interval.
 *  The DCCWaveform class generates the signals to the motor shield  
 *  based on this timer. 
 */

#include "DCCTimer.h"
#include "DIAG.h" 
const int DCC_SIGNAL_TIME=58;  // this is the 58uS DCC 1-bit waveform half-cycle 
const int DCC_SLOW_TIME=58*512;  // for <D DCC SLOW> command diagnostics 

INTERRUPT_CALLBACK interruptHandler=0;


void DCCTimer::begin(INTERRUPT_CALLBACK callback, bool slow) {
  interruptHandler=callback;
  // Initialise timer1 to trigger every 58us (DCC_SIGNAL_TIME)
  noInterrupts();
  
#ifdef ARDUINO_ARCH_MEGAAVR
      // Arduino unoWifi Rev2 and nanoEvery architectire 
      long clockCycles=slow? (14*512) : 14;  // guesswork!!!!
      DIAG(F("\nTimer unoWifi/nanoEvery F_CPU=%l c=%d"),F_CPU,clockCycles); 
      TCB0.CCMP = clockCycles;
      TCB0.INTFLAGS = TCB_CAPT_bm; // clear interrupt request flag
      TCB0.INTCTRL = TCB_CAPT_bm;  // Enable the interrupt
      TCB0.CNT = 0;
      TCB0.CTRLA |= TCB_ENABLE_bm;  // start
      #define ISR_NAME TCB0_INT_vect

#else 

      // Arduino nano, uno, mega
      long clockCycles=((F_CPU / 1000000) * (slow? DCC_SLOW_TIME : DCC_SIGNAL_TIME)) >>1;
      DIAG(F("\nTimer nano/uno/mega F_CPU=%l c=%d"),F_CPU,clockCycles); 
      TCCR1A = 0;
      ICR1 = clockCycles;
      TCNT1 = 0;   
      TCCR1B = _BV(WGM13) | _BV(CS10);     // Mode 8, clock select 1
      TIMSK1 = _BV(TOIE1); // Enable Software interrupt
      #define ISR_NAME TIMER1_OVF_vect
#endif
  
  interrupts();
}

// ISR called by timer interrupt every 58uS
ISR(ISR_NAME)
{
#ifdef ARDUINO_ARCH_MEGAAVR
    TCB0.INTFLAGS = TCB_CAPT_bm;
#endif
    if (interruptHandler) interruptHandler();
}
