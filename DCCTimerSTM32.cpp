/*
 *  © 2022 Paul M Antoine
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

// ATTENTION: this file only compiles on a STM32 based boards
// Please refer to DCCTimer.h for general comments about how this class works
// This is to avoid repetition and duplication.
#ifdef ARDUINO_ARCH_STM32

#include "FSH.h" //PMA temp debug
#include "DIAG.h" //PMA temp debug
#include "DCCTimer.h"

// STM32 doesn't have Serial1 defined by default
HardwareSerial Serial1(PA10, PA15);  // Rx=PA10, Tx=PA15

INTERRUPT_CALLBACK interruptHandler=0;
// Let's use STM32's timer #1 until disabused of this notion
HardwareTimer timer(TIM11);

// Timer IRQ handler
void Timer11_Handler() {
  interruptHandler();
}



void DCCTimer::begin(INTERRUPT_CALLBACK callback) {
  interruptHandler=callback;
  noInterrupts();

  // adc_set_sample_rate(ADC_SAMPLETIME_480CYCLES);
  timer.pause();
  timer.setPrescaleFactor(1);
//  timer.setOverflow(CLOCK_CYCLES * 2);
  timer.setOverflow(DCC_SIGNAL_TIME, MICROSEC_FORMAT);
  timer.attachInterrupt(Timer11_Handler);
  timer.refresh();
  timer.resume();

  interrupts();
}

bool DCCTimer::isPWMPin(byte pin) {
  //TODO: SAMD whilst this call to digitalPinHasPWM will reveal which pins can do PWM,
  //      there's no support yet for High Accuracy, so for now return false
  //  return digitalPinHasPWM(pin);
  return false;
}

void DCCTimer::setPWM(byte pin, bool high) {
    // TODO: High Accuracy mode is not supported as yet, and may never need to be
    (void) pin;
    (void) high;
}

void DCCTimer::clearPWM() {
  return;
}

void   DCCTimer::getSimulatedMacAddress(byte mac[6]) {
  volatile uint32_t *serno1 = (volatile uint32_t *)0x0080A00C;
  volatile uint32_t *serno2 = (volatile uint32_t *)0x0080A040;
//  volatile uint32_t *serno3 = (volatile uint32_t *)0x0080A044;
//  volatile uint32_t *serno4 = (volatile uint32_t *)0x0080A048;

  volatile uint32_t m1 = *serno1;
  volatile uint32_t m2 = *serno2;
  mac[0] = m1 >> 8;
  mac[1] = m1 >> 0;
  mac[2] = m2 >> 24;
  mac[3] = m2 >> 16;
  mac[4] = m2 >> 8;
  mac[5] = m2 >> 0;
}

volatile int DCCTimer::minimum_free_memory=__INT_MAX__;

// Return low memory value... 
int DCCTimer::getMinimumFreeMemory() {
  noInterrupts(); // Disable interrupts to get volatile value 
  int retval = freeMemory();
  interrupts();
  return retval;
}

extern "C" char* sbrk(int incr);

int DCCTimer::freeMemory() {
  char top;
  return (int)(&top - reinterpret_cast<char *>(sbrk(0)));
}

void DCCTimer::reset() {
   __disable_irq();
    NVIC_SystemReset();
    while(true) {};
}

#endif