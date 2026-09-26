/*
 *  © 2022 Paul M Antoine
 *  © 2021 Neil McKechnie
 *  © 2021 Mike S
 *  © 2021 Fred Decker
 *  © 2020-2022 Harald Barth
 *  © 2020-2025 Chris Harlow
 *
 *  This file is part of CommandStation-EX
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
 *
 */

#ifndef DEFINES_H
#define DEFINES_H
// defines.h relies on macros defined in config.h
// but it may have already been included (for cosmetic convenience) by the .ino

#if __has_include ( "config.h")
    #include "config.h"
#endif


////////////////////////////////////////////////////////////////////////////////
// Create a cpu type we can share and 
// figure out if we have enough memory for advanced features
// so define HAS_ENOUGH_MEMORY until proved otherwise.
#define HAS_ENOUGH_MEMORY
#undef USB_SERIAL     // Teensy has this defined by default...
#define USB_SERIAL Serial
#define USB_SERIAL_WEB 
// Include extended addresses unless specifically excluded
#define I2C_EXTENDED_ADDRESS

#if defined(ARDUINO_ARCH_ESP32)
  #define ARDUINO_TYPE "ESP32"
  #ifndef DISABLE_EEPROM
  #define DISABLE_EEPROM
  #endif


#elif defined(ARDUINO_ARCH_STM32)
  #define ARDUINO_TYPE "Nucleo"
  // STM32 no EEPROM by default 
  #ifndef DISABLE_EEPROM
    #define DISABLE_EEPROM
  #endif

  // STM32 support for native I2C is awaiting development 
  // #ifndef I2C_USE_WIRE
  // #define I2C_USE_WIRE
  // #endif


#else
  #define CPU_TYPE_ERROR
#endif

// replace board type if provided by compiler
#ifdef BOARD_NAME
  #undef ARDUINO_TYPE
  #define ARDUINO_TYPE BOARD_NAME
#endif


// configure serial log browser feature if possible
// Replace USB_SERIAL with SerialLog so we can browse it!
#undef USB_SERIAL
#include "SerialUsbLog.h"
#define USB_SERIAL SerialLog



#if __has_include ( "myAutomation.h")
  #if defined(HAS_ENOUGH_MEMORY) || defined(DISABLE_EEPROM) || defined(DISABLE_PROG)
    #define EXRAIL_ACTIVE
  #else
    #define EXRAIL_WARNING
  #endif
#endif

#if defined(ARDUINO_ARCH_STM32)
// The LwIP library for the STM32 wired ethernet has by default 10 TCP
// clients defined but because of a bug in the library #11 is not
// rejected but kicks out any old connection. By restricting our limit
// to 9 the #10 will be rejected by our code so that the number can
// never get to 11 which would kick an existing connection.
// If you want to change this value, do that in
// config.h AND in STM32lwipopts.h.
 #ifndef MAX_NUM_TCP_CLIENTS
  #define MAX_NUM_TCP_CLIENTS 9
 #endif
#else
 #if defined(ARDUINO_ARCH_ESP32)
// Espressif LWIP stack
  #define MAX_NUM_TCP_CLIENTS 10
 #else
// Wifi shields etc
  #define MAX_NUM_TCP_CLIENTS 8
 #endif
#endif


// Default MAX_LOCOS if not found in config.h
#ifndef DEFAULT_MAX_LOCOS 
   #define DEFAULT_MAX_LOCOS 120
#endif   
#ifndef MAX_LOCOS 
   #define MAX_LOCOS DEFAULT_MAX_LOCOS
#endif   

// Default IP_PORT if not found in config.h
#ifndef IP_PORT
    #define IP_PORT 2560
#endif



#endif //DEFINES_H
