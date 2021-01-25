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

const int DCC_SIGNAL_TIME=58;  // this is the 58uS DCC 1-bit waveform half-cycle 
const int DCC_SLOW_TIME=58*512;  // for <D DCC SLOW> command diagnostics 

INTERRUPT_CALLBACK interruptHandler=0;


void DCCTimer::begin(INTERRUPT_CALLBACK callback, bool slow) {
  interruptHandler=callback;
  // Initialise timer1 to trigger every 58us (DCC_SIGNAL_TIME)
  noInterrupts();
  TCCR1A = 0;
  ICR1 = ((F_CPU / 1000000) * (slow? DCC_SLOW_TIME : DCC_SIGNAL_TIME)) >>1;
  TCNT1 = 0;   
  TCCR1B = _BV(WGM13) | _BV(CS10);     // Mode 8, clock select 1
  TIMSK1 = _BV(TOIE1); // Enable Software interrupt
  interrupts();
}

// Timer interrupt every 58uS
ISR(TIMER1_OVF_vect)
{
    if (interruptHandler) interruptHandler();
}
