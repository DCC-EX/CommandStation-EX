/*
 *  © 2020, Chris Harlow. All rights reserved.
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
#ifndef StringFormatter_h
#define StringFormatter_h
#include <Arduino.h>

#if defined(ARDUINO_ARCH_SAMD)
   // Some processors use a gcc compiler that renames va_list!!!
  #include <cstdarg>
  #define DIAGSERIAL SerialUSB
#elif defined(ARDUINO_ARCH_AVR)
  #define DIAGSERIAL Serial
#elif defined(ARDUINO_ARCH_MEGAAVR)
  #define DIAGSERIAL Serial
  #define __FlashStringHelper char
#endif

class StringFormatter
{
  public:
    static void print( const __FlashStringHelper* input...);
    static void send(Print & serial, const __FlashStringHelper* input...);
    static void send(Print * serial, const __FlashStringHelper* input...);
    static void printEscapes(Print * stream, char * input);
    static void printEscapes(Print * stream, const __FlashStringHelper* input);
    static void printEscape(Print * stream, char c);
    static void send(Print * serial, const __FlashStringHelper* input,va_list args);
   
};
#endif
