/*
 *  © 2021 Neil McKechnie
 *  © 2021 Mike S
 *  © 2020 Harald Barth
 *  
 *  This file is part of Asbelos DCC-EX
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

#include <Arduino.h>
#include "freeMemory.h"

// thanks go to  https://github.com/mpflaga/Arduino-MemoryFree
#if defined(__arm__)
extern "C" char* sbrk(int);
#elif defined(__AVR__)
extern char *__brkval;
extern char *__malloc_heap_start;
#else
#error Unsupported board type
#endif


static volatile int minimum_free_memory = __INT_MAX__;

#if !defined(__IMXRT1062__)
static inline int freeMemory() {
  char top;
#if defined(__arm__)
  return &top - reinterpret_cast<char*>(sbrk(0));
#elif defined(__AVR__)
  return __brkval ? &top - __brkval : &top - __malloc_heap_start;
#else
#error bailed out already above
#endif
}

// Return low memory value.
int minimumFreeMemory() {
  byte sreg_save = SREG;
  noInterrupts(); // Disable interrupts
  int retval = minimum_free_memory;
  SREG = sreg_save; // Restore interrupt state
  return retval;
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

static inline int freeMemory() {
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

// Return low memory value.
int minimumFreeMemory() {
  //byte sreg_save = SREG;
  //noInterrupts(); // Disable interrupts
  int retval = minimum_free_memory;
  //SREG = sreg_save; // Restore interrupt state
  return retval;
}
#endif


// Update low ram level.  Allow for extra bytes to be specified
// by estimation or inspection, that may be used by other 
// called subroutines.  Must be called with interrupts disabled.
// 
// Although __brkval may go up and down as heap memory is allocated
// and freed, this function records only the worst case encountered.
// So even if all of the heap is freed, the reported minimum free 
// memory will not increase.
//
void updateMinimumFreeMemory(unsigned char extraBytes) {
  int spare = freeMemory()-extraBytes;
  if (spare < 0) spare = 0;
  if (spare < minimum_free_memory) minimum_free_memory = spare;
}

