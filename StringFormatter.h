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
#include "TransportProcessor.h"

#if defined(ARDUINO_ARCH_SAMD)
   // Some processors use a gcc compiler that renames va_list!!!
  #include <cstdarg>  
#elif defined(ARDUINO_ARCH_MEGAAVR)
  #define __FlashStringHelper char
#endif

class Diag {
  public:
  static bool ACK;
  static bool CMD;
  static bool WIFI;
  static bool WITHROTTLE;
};

class StringFormatter
{
  public:
    static void send(Print * serial, const __FlashStringHelper* input...);
    static void send(Print & serial, const __FlashStringHelper* input...);
    
    static void printEscapes(Print * serial,char * input);
    static void printEscape(Print * serial, char c);

    // DIAG support
    static Print * diagSerial;
    static void diag( const __FlashStringHelper* input...);
    static void printEscapes(char * input);
    static void printEscape( char c);


    static void setDiagOut(Connection *c) {
      if ( c->client->connected() ) {
        diagSerial = c->client;
      }
    }
    static void resetDiagOut() {
      diagSerial = &Serial;
    }

    private: 
    static void send2(Print * serial, const __FlashStringHelper* input,va_list args);

};
#endif
