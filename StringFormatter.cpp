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
#include "DisplayInterface.h"
#include "CommandDistributor.h"

bool Diag::ACK=false;
bool Diag::CMD=false;
bool Diag::WIFI=false;
bool Diag::WITHROTTLE=false;
bool Diag::ETHERNET=false;
bool Diag::LCN=false;

 
void StringFormatter::diag( const FSH* input...) {
 USB_SERIAL.print(F("<* "));   
  va_list args;
  va_start(args, input);
  send2(&USB_SERIAL,input,args);
  USB_SERIAL.print(F(" *>\n"));
}

void StringFormatter::lcd(byte row, const FSH* input...) {
  va_list args;
#ifndef DISABLE_VDPY
  Print * virtualLCD=CommandDistributor::getVirtualLCDSerial(0,row);
#else
  Print * virtualLCD=NULL;
#endif
  // Issue the LCD as a diag first
  // Unless the same serial is asking for the virtual @ respomnse
  if (virtualLCD!=&USB_SERIAL) {
    send(&USB_SERIAL,F("<* LCD%d:"),row);
    va_start(args, input);
    send2(&USB_SERIAL,input,args);
    send(&USB_SERIAL,F(" *>\n"));
  }
  
#ifndef DISABLE_VDPY
  // send to virtual LCD collector (if any) 
  if (virtualLCD) {
    va_start(args, input);
    send2(virtualLCD,input,args);
    CommandDistributor::commitVirtualLCDSerial();
  }
#endif
  DisplayInterface::setRow(row);    
  va_start(args, input);
  send2(DisplayInterface::getDisplayHandler(),input,args);
}

void StringFormatter::lcd2(uint8_t display, byte row, const FSH* input...) {
  va_list args;
  
   // send to virtual LCD collector (if any) 
#ifndef DISABLE_VDPY
  Print * virtualLCD=CommandDistributor::getVirtualLCDSerial(display,row);
  if (virtualLCD) {
    va_start(args, input);
    send2(virtualLCD,input,args);
    CommandDistributor::commitVirtualLCDSerial();
  }
#endif

  DisplayInterface::setRow(display, row);    
  va_start(args, input);
  send2(DisplayInterface::getDisplayHandler(),input,args);
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
    if (c=='\0') break; // to va_end()
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
      case 'S':
      { 
        const FSH*  flash= (const FSH*)va_arg(args, char*);

#if WIFI_ON | ETHERNET_ON
        // RingStream has special logic to handle flash strings
        // but is not implemented unless wifi or ethernet are enabled.
        // The define prevents RingStream code being added unnecessariliy.        
        if (stream->availableForWrite()==RingStream::THIS_IS_A_RINGSTREAM)
              ((RingStream *)stream)->printFlash(flash);
              else 
#endif
        stream->print(flash);
        break;
             }
      case 'P': stream->print((uint32_t)va_arg(args, void*), HEX); break;
      case 'd': printPadded(stream,va_arg(args, int), formatWidth, formatLeft); break;
      case 'u': printPadded(stream,va_arg(args, unsigned int), formatWidth, formatLeft); break;
      case 'l': printPadded(stream,va_arg(args, long), formatWidth, formatLeft); break;
      case 'b': stream->print(va_arg(args, int), BIN); break;
      case 'o': stream->print(va_arg(args, int), OCT); break;
      case 'x': stream->print((unsigned int)va_arg(args, unsigned int), HEX); break;
      case 'X': stream->print((unsigned long)va_arg(args, unsigned long), HEX); break;
      case 'h': printHex(stream,(unsigned int)va_arg(args, unsigned int)); break;
      case 'M':
      { // this prints a unsigned long microseconds time in readable format
	unsigned long time = va_arg(args, long);
	if (time >= 2000) {
	  time = time / 1000;
	  if (time >= 2000) {
	    printPadded(stream, time/1000, formatWidth, formatLeft);
	    stream->print(F("sec"));
	  } else {
	    printPadded(stream,time, formatWidth, formatLeft);
	    stream->print(F("msec"));
	  }
	} else {
	  printPadded(stream,time, formatWidth, formatLeft);
	  stream->print(F("usec"));
	}
      }
      break;
      //case 'f': stream->print(va_arg(args, double), 2); break;
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
  printEscape(&USB_SERIAL,c);
}

void StringFormatter::printEscape(Print * stream, char c) {
  if (!stream) return;
  switch(c) {
     case '\n': stream->print(F("\\n")); break; 
     case '\r': stream->print(F("\\r")); break; 
     case '\0': stream->print(F("\\0")); return; 
     case '\t': stream->print(F("\\t")); break;
     case '\\': stream->print(F("\\\\")); break;
     default: stream->write(c);
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

// printHex prints the full 2 byte hex with leading zeros, unlike print(value,HEX)
const char FLASH hexchars[]="0123456789ABCDEF";
void StringFormatter::printHex(Print * stream,uint16_t value) {
    char result[5];
    for (int i=3;i>=0;i--) {
      result[i]=GETFLASH(hexchars+(value & 0x0F));
      value>>=4;
    }
    result[4]='\0';
     stream->print(result);
}
