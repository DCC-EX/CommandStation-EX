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

// ATTENTION: this file only compiles on a TEENSY
// Please refer to DCCTimer.h for general comments about how this class works
// This is to avoid repetition and duplication.
#ifdef TEENSYDUINO

#include "DCCTimer.h"

INTERRUPT_CALLBACK interruptHandler=0;

IntervalTimer myDCCTimer;

void DCCTimer::begin(INTERRUPT_CALLBACK callback) {
  interruptHandler=callback;
  myDCCTimer.begin(interruptHandler, DCC_SIGNAL_TIME);
  }

void DCCTimer::startRailcomTimer(byte brakePin) {
  // TODO: for intended operation see DCCTimerAVR.cpp
  (void) brakePin; 
}

void DCCTimer::ackRailcomTimer() {
  // TODO: for intended operation see DCCTimerAVR.cpp
}

bool DCCTimer::isPWMPin(byte pin) {
       //Teensy: digitalPinHasPWM, todo
      (void) pin;
       return false;  // TODO what are the relevant pins? 
  }

void DCCTimer::setPWM(byte pin, bool high) {
    // TODO what are the relevant pins?
    (void) pin;
    (void) high;
}

void DCCTimer::clearPWM() {
    // Do nothing unless we implent HA
}

#if defined(__IMXRT1062__)  //Teensy 4.0 and Teensy 4.1
void   DCCTimer::getSimulatedMacAddress(byte mac[6]) {
    uint32_t m1 = HW_OCOTP_MAC1;
    uint32_t m2 = HW_OCOTP_MAC0;
    mac[0] = m1 >> 8;
    mac[1] = m1 >> 0;
    mac[2] = m2 >> 24;
    mac[3] = m2 >> 16;
    mac[4] = m2 >> 8;
    mac[5] = m2 >> 0;
  }

#else

// http://forum.pjrc.com/threads/91-teensy-3-MAC-address
void teensyRead(uint8_t word, uint8_t *mac, uint8_t offset) {
  FTFL_FCCOB0 = 0x41;             // Selects the READONCE command
  FTFL_FCCOB1 = word;             // read the given word of read once area

  // launch command and wait until complete
  FTFL_FSTAT = FTFL_FSTAT_CCIF;
  while(!(FTFL_FSTAT & FTFL_FSTAT_CCIF));

  *(mac+offset) =   FTFL_FCCOB5;       // collect only the top three bytes,
  *(mac+offset+1) = FTFL_FCCOB6;       // in the right orientation (big endian).
  *(mac+offset+2) = FTFL_FCCOB7;       // Skip FTFL_FCCOB4 as it's always 0.
}

void   DCCTimer::getSimulatedMacAddress(byte mac[6]) {
    teensyRead(0xe,mac,0);
    teensyRead(0xf,mac,3);
  }
#endif 

volatile int DCCTimer::minimum_free_memory=__INT_MAX__;

// Return low memory value... 
int DCCTimer::getMinimumFreeMemory() {
  noInterrupts(); // Disable interrupts to get volatile value 
  int retval = freeMemory();
  interrupts();
  return retval;
}

extern "C" char* sbrk(int incr);

#if !defined(__IMXRT1062__)
int DCCTimer::freeMemory() {
  char top;
  return &top - reinterpret_cast<char*>(sbrk(0));
}

#else
#if defined(ARDUINO_TEENSY40)
  static const unsigned DTCM_START = 0x20000000UL;
  static const unsigned OCRAM_START = 0x20200000UL;
  static const unsigned OCRAM_SIZE = 512;
  static const unsigned FLASH_SIZE = 1984;
#elif defined(ARDUINO_TEENSY41)
  static const unsigned DTCM_START = 0x20000000UL;
  static const unsigned OCRAM_START = 0x20200000UL;
  static const unsigned OCRAM_SIZE = 512;
  static const unsigned FLASH_SIZE = 7936;
#if TEENSYDUINO>151
  extern "C" uint8_t external_psram_size;
#endif
#endif

int DCCTimer::freeMemory() {
  extern unsigned long _ebss;
  extern unsigned long _sdata;
  extern unsigned long _estack;
  const unsigned DTCM_START = 0x20000000UL;
  unsigned dtcm = (unsigned)&_estack - DTCM_START;
  unsigned stackinuse = (unsigned) &_estack -  (unsigned) __builtin_frame_address(0);
  unsigned varsinuse = (unsigned)&_ebss - (unsigned)&_sdata;
  unsigned freemem = dtcm - (stackinuse + varsinuse);
  return freemem;
}

#endif
void DCCTimer::reset() {
  // found at https://forum.pjrc.com/threads/59935-Reboot-Teensy-programmatically
  SCB_AIRCR = 0x05FA0004;
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
