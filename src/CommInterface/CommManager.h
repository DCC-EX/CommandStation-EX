/*
 *  CommManager.h
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

#ifndef COMMINTERFACE_COMMMANAGER_H_
#define COMMINTERFACE_COMMMANAGER_H_

#include "CommInterface.h"

#include "CommInterfaceSerial.h"
#if defined (ARDUINO_ARCH_SAMD)
  #include "CommInterfaceUSB.h"
#endif

#if defined(ARDUINO_ARCH_SAMD)
  #include <cstdarg>
  #define DIAGSERIAL SerialUSB
#elif defined(ARDUINO_ARCH_SAMC)
  #include <cstdarg>
  #define DIAGSERIAL Serial
#elif defined(ARDUINO_ARCH_AVR)
  #define DIAGSERIAL Serial
#elif defined(ARDUINO_ARCH_MEGAAVR)
  #define DIAGSERIAL Serial
  #define __FlashStringHelper char
#endif

class CommManager {
public:
  static void update();
  static void registerInterface(CommInterface *interface);
  static void showConfiguration();
  static void showInitInfo();
  static void broadcast(const __FlashStringHelper* input, ...);
  static void print(const __FlashStringHelper* input, ...);
  static void doNotPrint(const __FlashStringHelper* input, ...) { (void)input;}
  static void send(Print* stream, const __FlashStringHelper* input, ...);
  static void printEscapes(Print* stream, char * input);
  static void printEscapes(Print* stream, const __FlashStringHelper* input);
  static void printEscape(Print* stream, char c);
private:
  static void send2(Print* stream, const __FlashStringHelper* format, va_list args);
  static CommInterface *interfaces[5];
  static int nextInterface;
};

#endif	// COMMINTERFACE_COMMMANAGER_H_