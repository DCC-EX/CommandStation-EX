/*
 *  © 2023 Travis Farmer
 *  © 2023 Neil McKechnie
 *  © 2022-2023 Paul M. Antoine
 *  © 2021 Mike S
 *  © 2021, 2023 Harald Barth
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
#if defined(ARDUINO_GIGA)

#include "DCCTimer.h"
#include "DIAG.h"
#include "GigaHardwareTimer.h"
#include <Arduino_AdvancedAnalog.h>
//#include "config.h"
///////////////////////////////////////////////////////////////////////////////////////////////
// Experimental code for High Accuracy (HA) DCC Signal mode
// Warning - use of TIM2 and TIM3 can affect the use of analogWrite() function on certain pins,
// which is used by the DC motor types.
///////////////////////////////////////////////////////////////////////////////////////////////

INTERRUPT_CALLBACK interruptHandler=0;

#ifndef DCC_EX_TIMER
#if defined(TIM6)
#define DCC_EX_TIMER TIM6
#elif defined(TIM7)
#define DCC_EX_TIMER TIM7
#elif defined(TIM12)
#define DCC_EX_TIMER TIM12
#else
#warning This Giga variant does not have Timers 1,8 or 11!!
#endif
#endif // ifndef DCC_EX_TIMER

HardwareTimer dcctimer(TIM8);
void DCCTimer_Handler() __attribute__((interrupt));

void DCCTimer_Handler() {
  interruptHandler();
}

void DCCTimer::begin(INTERRUPT_CALLBACK callback) {
  interruptHandler=callback;
  noInterrupts();

  dcctimer.pause();
  dcctimer.setPrescaleFactor(1);
//  timer.setOverflow(CLOCK_CYCLES * 2);
  dcctimer.setOverflow(DCC_SIGNAL_TIME, MICROSEC_FORMAT);
  // dcctimer.attachInterrupt(Timer11_Handler);
  dcctimer.attachInterrupt(DCCTimer_Handler);
  dcctimer.setInterruptPriority(0, 0); // Set highest preemptive priority!
  dcctimer.refresh();
  dcctimer.resume();

  interrupts();
}

bool DCCTimer::isPWMPin(byte pin) {
  //TODO: STM32 whilst this call to digitalPinHasPWM will reveal which pins can do PWM,
  //      there's no support yet for High Accuracy, so for now return false
  //  return digitalPinHasPWM(pin);
  (void) pin;
  return false;
}

void DCCTimer::setPWM(byte pin, bool high) {
    // TODO: High Accuracy mode is not supported as yet, and may never need to be
    (void) pin;
    (void) high;
    return;
 }

void DCCTimer::clearPWM() {
  return;
}

void   DCCTimer::getSimulatedMacAddress(byte mac[6]) {
  volatile uint32_t *serno1 = (volatile uint32_t *)UID_BASE;
  volatile uint32_t *serno2 = (volatile uint32_t *)UID_BASE+4;
  volatile uint32_t *serno3 = (volatile uint32_t *)UID_BASE+8;
  volatile uint32_t m1 = *serno1;
  volatile uint32_t m2 = *serno2;
  volatile uint32_t m3 = *serno3;
  mac[0] = 0xBE;
  mac[1] = 0xEF;
  mac[2] = m1 ^ m3 >> 24;
  mac[3] = m1 ^ m3 >> 16;
  mac[4] = m1 ^ m3 >> 8;
  mac[5] = m1 ^ m3 >> 0;
  //DIAG(F("MAC: %P:%P:%P:%P:%P:%P"),mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);

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
  unsigned int tmp = (unsigned int)(&top - reinterpret_cast<char*>(sbrk(0)));
  return (int)(tmp / 1000);
}

void DCCTimer::reset() {
  //Watchdog &watchdog = Watchdog::get_instance();
  //Watchdog::stop();
  //Watchdog::start(500);
  
  //while(true) {};
  return;
}

int * ADCee::analogvals = NULL;

int16_t ADCee::ADCmax()
{
    return 1023;
}

AdvancedADC adc(A0, A1, A2, A3);
int ADCee::init(uint8_t pin) {
  adc.begin(AN_RESOLUTION_10, 16000, 1, 512);
  return 123;
}

/*
 * Read function ADCee::read(pin) to get value instead of analogRead(pin)
 */
int ADCee::read(uint8_t pin, bool fromISR) {
  static SampleBuffer buf = adc.read();
  int retVal = -123;
  if (adc.available()) {
    buf.release();
    buf = adc.read();
  }
  return (buf[pin - A0]);
}

/*
 * Scan function that is called from interrupt
 */
#pragma GCC push_options
#pragma GCC optimize ("-O3")
void ADCee::scan() {
}
#pragma GCC pop_options

void ADCee::begin() {
  noInterrupts();
  
  interrupts();
}
#endif
