/*
 *  CommManager.cpp
 * 
 *  This file is part of CommandStation-EX.
 *
 *  CommandStation-EX is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  CommandStation-EX is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with CommandStation-EX.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "CommManager.h"

#include <Arduino.h>

CommInterface *CommManager::interfaces[5] = {NULL, NULL, NULL, NULL, NULL};
int CommManager::nextInterface = 0;

void CommManager::update() {
	for(int i = 0; i < nextInterface; i++) {
		if(interfaces[i] != NULL) {
			interfaces[i]->process();
		}
	}
}

void CommManager::registerInterface(CommInterface *interface) {
	if(nextInterface < 5) {
		interfaces[nextInterface++] = interface;
	}
}

void CommManager::showConfiguration() {
	for(int i = 0; i < nextInterface; i++) {
		if(interfaces[i] != NULL) {
			interfaces[i]->showConfiguration();
		}
	}
}

void CommManager::showInitInfo() {
	for(int i = 0; i < nextInterface; i++) {
		if(interfaces[i] != NULL) {
			interfaces[i]->showInitInfo();	
		}
	}
}

void CommManager::broadcast(const __FlashStringHelper* input, ...) {
  for(int i = 0; i < nextInterface; i++) {
		if(interfaces[i] != NULL) {
			Print* mPrint = interfaces[i]->getStream();	

      va_list args;
      va_start(args, input);
      send2(mPrint, input, args);
      va_end(args);
		}
	}
}

void CommManager::print(const __FlashStringHelper* input, ...) {
	va_list args;
  va_start(args, input);
  send2(&DIAGSERIAL, input, args);
  va_end(args);
}

void CommManager::send(Print* stream, const __FlashStringHelper* input, ...) {
  va_list args;
  va_start(args, input);
  send2(stream, input, args);
  va_end(args);
}

void CommManager::send2(Print* stream, const __FlashStringHelper* format, va_list args) {
  char* flash = (char*)format;
  for(int i=0; ; ++i) {
    char c=pgm_read_byte_near(flash + i);
    if(c == '\0') return;
    if(c != '%') { 
      stream->print(c); 
      continue; 
    }
    i++;
    c = pgm_read_byte_near(flash + i);
    switch(c) {
      case '%': stream->print('%'); break;
      case 'c': stream->print((char) va_arg(args, int)); break;
      case 's': stream->print(va_arg(args, char*)); break;
      case 'e': printEscapes(stream,va_arg(args, char*)); break;
      case 'E': printEscapes(stream,(const __FlashStringHelper*)va_arg(args, char*)); break;
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

void CommManager::printEscapes(Print* stream, char * input) {
  for(int i=0; ; ++i) {
    char c=input[i];
    printEscape(stream,c);
    if (c=='\0') return;
  }
}

void CommManager::printEscapes(Print* stream, const __FlashStringHelper* input) {
  char* flash=(char*)input;
    for(int i=0; ; ++i) {
    char c=pgm_read_byte_near(flash+i);
    printEscape(stream,c);
    if (c=='\0') return;
  }
}

void CommManager::printEscape(Print* stream, char c) {
  switch(c) {
    case '\n': stream->print(F("\\n")); break; 
    case '\r': stream->print(F("\\r")); break; 
    case '\0': stream->print(F("\\0")); return; 
    case '\t': stream->print(F("\\t")); break;
    case '\\': stream->print(F("\\")); break;
    default: stream->print(c);
  }
}