/*
 *  Timer.cpp
 * 
 *  This file is part of CommandStation-EX.
 *
 *  CommandStation-EX is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  CommandStation-EX is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with CommandStation-EX.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <Arduino.h>

#if defined(ARDUINO_ARCH_SAMD) || defined(ARDUINO_ARCH_SAMC)

#if defined(ARDUINO_ARCH_SAMC)
#include "ATSAMC21/Timer.h"
#else
#include "ATSAMD21/Timer.h"
#endif

Timer TimerA(TCC0);
Timer TimerB(TCC1);
Timer TimerC(TCC2);

void TCC0_Handler() {
  if(TCC0->INTFLAG.bit.OVF) {
    TCC0->INTFLAG.bit.OVF = 1;
    TimerA.isrCallback();
  }
}

void TCC1_Handler() {
  if(TCC1->INTFLAG.bit.OVF) {
    TCC1->INTFLAG.bit.OVF = 1;
    TimerB.isrCallback();
  }
}

void TCC2_Handler() {
  if(TCC2->INTFLAG.bit.OVF) {
    TCC2->INTFLAG.bit.OVF = 1;
    TimerC.isrCallback();
  }
}
#elif defined(ARDUINO_AVR_MEGA) || defined(ARDUINO_AVR_MEGA2560)

#include "ATMEGA2560/Timer.h"

Timer TimerA(1);
Timer TimerB(3);
Timer TimerC(4);
Timer TimerD(5);

ISR(TIMER1_OVF_vect)
{
  TimerA.isrCallback();
}

ISR(TIMER3_OVF_vect)
{
  TimerB.isrCallback();
}

ISR(TIMER4_OVF_vect)
{
  TimerC.isrCallback();
}

ISR(TIMER5_OVF_vect)
{
  TimerD.isrCallback();
}

#elif defined(ARDUINO_AVR_UNO)      // Todo: add other 328 boards for compatibility

#include "ATMEGA328/Timer.h"

Timer TimerA(1);
Timer TimerB(2);

ISR(TIMER1_OVF_vect)
{
  TimerA.isrCallback();
}

ISR(TIMER2_OVF_vect)
{
  TimerB.isrCallback();
}

#elif defined(ARDUINO_ARCH_MEGAAVR)

#include "ATMEGA4809/Timer.h"

Timer TimerA(0);

ISR(TCA0_OVF_vect) {
  TimerA.isrCallback();
}

#endif