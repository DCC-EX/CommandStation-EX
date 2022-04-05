/*
 *  © 2021 Mike S
 *  © 2021 Harald Barth
 *  © 2021 Fred Decker
 *  © 2021 Chris Harlow
 *  © 2021 David Cutting
 *  © 2022 Paul M. Antoine
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

// ATTENTION: this file only compiles on a TEENSY
// Please refer to DCCTimer.h for general comments about how this class works
// This is to avoid repetition and duplication.
#ifdef ARDUINO_ARCH_SAMD

#include "FSH.h" //PMA temp debug
#include "DIAG.h" //PMA temp debug
#include "DCCTimer.h"

INTERRUPT_CALLBACK interruptHandler=0;

void DCCTimer::begin(INTERRUPT_CALLBACK callback) {
  interruptHandler=callback;




  }

bool DCCTimer::isPWMPin(byte pin) {
       //SAMD: digitalPinHasPWM, todo
      (void) pin;
       return false;  // TODO what are the relevant pins? 
  }

void DCCTimer::setPWM(byte pin, bool high) {
    // TODO what are the relevant pins?
    (void) pin;
    (void) high;
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

#endif