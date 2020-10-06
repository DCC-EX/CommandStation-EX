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
#include "StringFormatter.h"
#include <stdarg.h>

#if defined(ARDUINO_ARCH_SAMD)
   // Some processors use a gcc compiler that renames va_list!!!
  #include <cstdarg>
  Print * StringFormatter::diagSerial= &SerialUSB;
  
#elif defined(ARDUINO_ARCH_AVR)
  Print * StringFormatter::diagSerial= &Serial;
#elif defined(ARDUINO_ARCH_MEGAAVR)
  Print * StringFormatter::diagSerial=&Serial;
  #define __FlashStringHelper char
#endif

#include "LCDDisplay.h"

bool Diag::ACK=false;
bool Diag::CMD=false;
bool Diag::WIFI=false;
bool Diag::WITHROTTLE=false;

 
void StringFormatter::diag( const __FlashStringHelper* input...) {
  if (!diagSerial) return;    
  va_list args;
  va_start(args, input);
  send2(diagSerial,input,args);
}

void StringFormatter::lcd(byte row, const __FlashStringHelper* input...) {
  if (!LCDDisplay::lcdDisplay) return;
  LCDDisplay::lcdDisplay->setRow(row);    
  va_list args;
  va_start(args, input);
  send2(LCDDisplay::lcdDisplay,input,args);
  LCDDisplay::lcdDisplay->display();
}

void StringFormatter::send(Print * stream, const __FlashStringHelper* input...) {
  va_list args;
  va_start(args, input);
  send2(stream,input,args);
}

void StringFormatter::send(Print & stream, const __FlashStringHelper* input...) {
  va_list args;
  va_start(args, input);
  send2(&stream,input,args);
}


void StringFormatter::send2(Print * stream,const __FlashStringHelper* format, va_list args) {
    
  // thanks to Jan Turoň  https://arduino.stackexchange.com/questions/56517/formatting-strings-in-arduino-for-output

  char* flash=(char*)format;
  for(int i=0; ; ++i) {
    char c=pgm_read_byte_near(flash+i);
    if (c=='\0') return;
    if(c!='%') { stream->print(c); continue; }
    i++;
    c=pgm_read_byte_near(flash+i);
    switch(c) {
      case '%': stream->print('%'); break;
      case 'c': stream->print((char) va_arg(args, int)); break;
      case 's': stream->print(va_arg(args, char*)); break;
      case 'e': printEscapes(stream,va_arg(args, char*)); break;
      case 'S': stream->print((const __FlashStringHelper*)va_arg(args, char*)); break;
      case 'd': stream->print(va_arg(args, int), DEC); break;
      case 'l': stream->print(va_arg(args, long), DEC); break;
      case 'b': stream->print(va_arg(args, int), BIN); break;
      case 'o': stream->print(va_arg(args, int), OCT); break;
      case 'x': stream->print(va_arg(args, int), HEX); break;
      case 'f': stream->print(va_arg(args, double), 2); break;
    }
  }
  va_end(args);
}

void StringFormatter::printEscapes(Print * stream,char * input) {
 if (!stream) return;
 for(int i=0; ; ++i) {
  char c=input[i];
  printEscape(stream,c);
  if (c=='\0') return;
 }
}

void StringFormatter::printEscape( char c) {
  printEscape(diagSerial,c);
}

void StringFormatter::printEscape(Print * stream, char c) {
  if (!stream) return;
  switch(c) {
     case '\n': stream->print(F("\\n")); break; 
     case '\r': stream->print(F("\\r")); break; 
     case '\0': stream->print(F("\\0")); return; 
     case '\t': stream->print(F("\\t")); break;
     case '\\': stream->print(F("\\")); break;
     default: stream->print(c);
  }
 }
 
