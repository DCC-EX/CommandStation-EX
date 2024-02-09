/*
 *  © 2022 Paul M. Antoine
 *  © 2021 Mike S
 *  © 2021 Harald Barth
 *  © 2021 Fred Decker
 *  © 2021 Chris Harlow
 *  © 2021 David Cutting
 *  All rights reserved.
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

// ATTENTION: this file only compiles on a UnoWifiRev3 or NanoEvery
// Please refer to DCCTimer.h for general comments about how this class works
// This is to avoid repetition and duplication.
#ifdef ARDUINO_ARCH_MEGAAVR

#include "DCCTimer.h"

INTERRUPT_CALLBACK interruptHandler=0;
extern char *__brkval;
extern char *__malloc_heap_start;

  
  void DCCTimer::begin(INTERRUPT_CALLBACK callback) {
    interruptHandler=callback;
    noInterrupts(); 
    ADC0.CTRLC = (ADC0.CTRLC & 0b00110000) | 0b01000011;  // speed up analogRead sample time   
    TCB0.CTRLB = TCB_CNTMODE_INT_gc & ~TCB_CCMPEN_bm; // timer compare mode with output disabled
    TCB0.CTRLA = TCB_CLKSEL_CLKDIV2_gc; //   8 MHz ~ 0.125 us      
    TCB0.CCMP =  CLOCK_CYCLES -1;  // 1 tick less for timer reset
    TCB0.INTFLAGS = TCB_CAPT_bm; // clear interrupt request flag
    TCB0.INTCTRL = TCB_CAPT_bm;  // Enable the interrupt
    TCB0.CNT = 0;
    TCB0.CTRLA |= TCB_ENABLE_bm;  // start
    interrupts();
  }

  // ISR called by timer interrupt every 58uS
  ISR(TCB0_INT_vect){
    TCB0.INTFLAGS = TCB_CAPT_bm; // Clear interrupt request flag
    interruptHandler();
  }

void DCCTimer::startRailcomTimer(byte brakePin) {
  // TODO: for intended operation see DCCTimerAVR.cpp
  (void) brakePin; 
}

void DCCTimer::ackRailcomTimer() {
  // TODO: for intended operation see DCCTimerAVR.cpp
}

  bool DCCTimer::isPWMPin(byte pin) {
       (void) pin; 
       return false;  // TODO what are the relevant pins? 
  }

 void DCCTimer::setPWM(byte pin, bool high) {
    (void) pin;
    (void) high;
    // TODO what are the relevant pins?
 }

void DCCTimer::clearPWM() {
    // Do nothing unless we implent HA
}

  void   DCCTimer::getSimulatedMacAddress(byte mac[6]) {
    memcpy(mac,(void *) &SIGROW.SERNUM0,6);  // serial number
    mac[0] &= 0xFE;
    mac[0] |= 0x02;
  }

volatile int DCCTimer::minimum_free_memory=__INT_MAX__;

// Return low memory value... 
int DCCTimer::getMinimumFreeMemory() {
  noInterrupts(); // Disable interrupts to get volatile value 
  int retval = minimum_free_memory;
  interrupts();
  return retval;
}

extern char *__brkval;
extern char *__malloc_heap_start;

int DCCTimer::freeMemory() {
  char top;
  return __brkval ? &top - __brkval : &top - __malloc_heap_start;
}

void DCCTimer::reset() {
  CPU_CCP=0xD8;
  WDT.CTRLA=0x4;
  while(true){}
}

int16_t ADCee::ADCmax() {
  return 4095;
}

int ADCee::init(uint8_t pin) {
  return analogRead(pin);
}
/*
 * Read function ADCee::read(pin) to get value instead of analogRead(pin)
 */
int ADCee::read(uint8_t pin, bool fromISR) {
  int current;
  if (!fromISR) noInterrupts();
  current = analogRead(pin);
  if (!fromISR) interrupts();
  return current;
}
/*
 * Scan function that is called from interrupt
 */
void ADCee::scan() {
}

void ADCee::begin() {
  noInterrupts();
  interrupts();
}
#endif
