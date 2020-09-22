/*
 *  © 2020, Harald Barth
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


int freeMemory() {
  char top;
#if defined(__arm__)
  return &top - reinterpret_cast<char*>(sbrk(0));
#elif defined(__AVR__)
  return __brkval ? &top - __brkval : &top - __malloc_heap_start;
#else
#error bailed out alredy above
#endif
}
