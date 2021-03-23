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
#include "FSH.h"
#if defined(ARDUINO_ARCH_SAMD)
   // Some processors use a gcc compiler that renames va_list!!!
  #include <cstdarg>  
#endif

#include "LCDDisplay.h"
class Diag {
  public:
  static bool ACK;
  static bool CMD;
  static bool WIFI;
  static bool WITHROTTLE;
  static bool ETHERNET;
  static bool LCN;
  
};

class StringFormatter
{
  public:
    static void send(Print * serial, const FSH* input...);
    static void send(Print & serial, const FSH* input...);
    
    static void printEscapes(Print * serial,char * input);
    static void printEscapes(Print * serial,const FSH* input);
    static void printEscape(Print * serial, char c);

    // DIAG support
    static Print * diagSerial;
    static void diag( const FSH* input...);
    static void lcd(byte row, const FSH* input...);
    static void printEscapes(char * input);
    static void printEscape( char c);

    private: 
    static void send2(Print * serial, const FSH* input,va_list args);
    static void printPadded(Print* stream, long value, byte width, bool formatLeft);

};
#endif
