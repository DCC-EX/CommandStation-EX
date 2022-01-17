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
#else
  Print * StringFormatter::diagSerial=&Serial;
#endif

#include "LCDDisplay.h"

bool Diag::ACK=false;
bool Diag::CMD=false;
bool Diag::WIFI=false;
bool Diag::WITHROTTLE=false;
bool Diag::ETHERNET=false;
bool Diag::LCN=false;

 
void StringFormatter::diag( const FSH* input...) {
  if (!diagSerial) return; 
  diagSerial->print(F("<* "));   
  va_list args;
  va_start(args, input);
  send2(diagSerial,input,args);
  diagSerial->print(F(" *>\n"));
}

void StringFormatter::lcd(byte row, const FSH* input...) {
  va_list args;

  // Issue the LCD as a diag first
  send(diagSerial,F("<* LCD%d:"),row);
  va_start(args, input);
  send2(diagSerial,input,args);
  send(diagSerial,F(" *>\n"));
  
  if (!LCDDisplay::lcdDisplay) return;
  LCDDisplay::lcdDisplay->setRow(row);    
  va_start(args, input);
  send2(LCDDisplay::lcdDisplay,input,args);
}

void StringFormatter::send(Print * stream, const FSH* input...) {
  va_list args;
  va_start(args, input);
  send2(stream,input,args);
}

void StringFormatter::send(Print & stream, const FSH* input...) {
  va_list args;
  va_start(args, input);
  send2(&stream,input,args);
}

void StringFormatter::send2(Print * stream,const FSH* format, va_list args) {
    
  // thanks to Jan Turoň  https://arduino.stackexchange.com/questions/56517/formatting-strings-in-arduino-for-output

  char* flash=(char*)format;
  for(int i=0; ; ++i) {
    char c=GETFLASH(flash+i);
    if (c=='\0') return;
    if(c!='%') { stream->print(c); continue; }

    bool formatContinues=false;
    byte formatWidth=0;
    bool formatLeft=false; 
  do {
    
    formatContinues=false;
    i++;
    c=GETFLASH(flash+i);
    switch(c) {
      case '%': stream->print('%'); break;
      case 'c': stream->print((char) va_arg(args, int)); break;
      case 's': stream->print(va_arg(args, char*)); break;
      case 'e': printEscapes(stream,va_arg(args, char*)); break;
      case 'E': printEscapes(stream,(const FSH*)va_arg(args, char*)); break;
      case 'S': stream->print((const FSH*)va_arg(args, char*)); break;
      case 'd': printPadded(stream,va_arg(args, int), formatWidth, formatLeft); break;
      case 'u': printPadded(stream,va_arg(args, unsigned int), formatWidth, formatLeft); break;
      case 'l': printPadded(stream,va_arg(args, long), formatWidth, formatLeft); break;
      case 'b': stream->print(va_arg(args, int), BIN); break;
      case 'o': stream->print(va_arg(args, int), OCT); break;
      case 'x': stream->print(va_arg(args, int), HEX); break;
      case 'f': stream->print(va_arg(args, double), 2); break;
      //format width prefix
      case '-': 
            formatLeft=true;
            formatContinues=true;
            break; 
      case '0': 
      case '1': 
      case '2': 
      case '3': 
      case '4': 
      case '5': 
      case '6': 
      case '7': 
      case '8': 
      case '9': 
            formatWidth=formatWidth * 10 + (c-'0');
            formatContinues=true;
            break;
    }
  } while(formatContinues);
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

void StringFormatter::printEscapes(Print * stream, const FSH * input) {
 
 if (!stream) return;
 char* flash=(char*)input;
 for(int i=0; ; ++i) {
  char c=GETFLASH(flash+i);
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

 
void StringFormatter::printPadded(Print* stream, long value, byte width, bool formatLeft) {
  if (width==0) {
    stream->print(value, DEC);
    return;
  }
  
    int digits=(value <= 0)? 1: 0;  // zero and negative need extra digot
    long v=value;
    while (v) {
        v /= 10;
        digits++;
    }
    
    if (formatLeft) stream->print(value, DEC);
    while(digits<width) {
      stream->print(' ');
      digits++;
    }
    if (!formatLeft) stream->print(value, DEC);    
  }

 
