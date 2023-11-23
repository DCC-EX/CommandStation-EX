/*
 *  © 2022 Paul M Antoine
 *  © 2021 Neil McKechnie
 *  © 2021 Mike S
 *  © 2021 Fred Decker
 *  © 2020-2022 Harald Barth
 *  © 2020-2021 Chris Harlow
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
// but it may have already been included (for cosmetic convenence) by the .ino
#ifndef MOTOR_SHIELD_TYPE
  #if __has_include ( "config.h")
    #include "config.h"
  #else
    #include "config.example.h"
  #endif
#endif

////////////////////////////////////////////////////////////////////////////////
// Create a cpu type we can share and 
// figure out if we have enough memory for advanced features
// so define HAS_ENOUGH_MEMORY until proved otherwise.
#define HAS_ENOUGH_MEMORY
#undef USB_SERIAL     // Teensy has this defined by default...
#define USB_SERIAL Serial

// Include extended addresses unless specifically excluded
#define I2C_EXTENDED_ADDRESS

#if defined(ARDUINO_AVR_UNO)
  #define ARDUINO_TYPE "UNO"
  #undef HAS_ENOUGH_MEMORY
  #define NO_EXTENDED_CHARACTERS
  #undef I2C_EXTENDED_ADDRESS
#elif defined(ARDUINO_AVR_NANO)
  #define ARDUINO_TYPE "NANO"
  #undef HAS_ENOUGH_MEMORY
  #define NO_EXTENDED_CHARACTERS
  #undef I2C_EXTENDED_ADDRESS
#elif defined(ARDUINO_AVR_MEGA)
  #define ARDUINO_TYPE "MEGA"
#elif defined(ARDUINO_AVR_MEGA2560)
  #define ARDUINO_TYPE "MEGA"
#elif defined(ARDUINO_ARCH_MEGAAVR)
  #define ARDUINO_TYPE "MEGAAVR"
  #undef HAS_ENOUGH_MEMORY
  #define NO_EXTENDED_CHARACTERS
  #undef I2C_EXTENDED_ADDRESS
#elif defined(ARDUINO_TEENSY31)
  #define ARDUINO_TYPE "TEENSY3132"
  #undef USB_SERIAL
  #define USB_SERIAL SerialUSB
  #ifndef DISABLE_EEPROM
    #define DISABLE_EEPROM
  #endif
  // Teensy support for native I2C is awaiting development 
  #ifndef I2C_USE_WIRE
  #define I2C_USE_WIRE
  #endif
#elif defined(ARDUINO_TEENSY35)
  #define ARDUINO_TYPE "TEENSY35"
  #undef USB_SERIAL
  #define USB_SERIAL SerialUSB
  // Teensy support for I2C is awaiting development 
  #ifndef DISABLE_EEPROM
    #define DISABLE_EEPROM
  #endif
  // Teensy support for native I2C is awaiting development 
  #ifndef I2C_USE_WIRE
  #define I2C_USE_WIRE
  #endif
#elif defined(ARDUINO_TEENSY36)
  #define ARDUINO_TYPE "TEENSY36"
  #undef USB_SERIAL
  #define USB_SERIAL SerialUSB
  #ifndef DISABLE_EEPROM
    #define DISABLE_EEPROM
  #endif
  // Teensy support for native I2C is awaiting development 
  #ifndef I2C_USE_WIRE
  #define I2C_USE_WIRE
  #endif
#elif defined(ARDUINO_TEENSY40)
  #define ARDUINO_TYPE "TEENSY40"
  #undef USB_SERIAL
  #define USB_SERIAL SerialUSB
  #ifndef DISABLE_EEPROM
    #define DISABLE_EEPROM
  #endif
  // Teensy support for native I2C is awaiting development 
  #ifndef I2C_USE_WIRE
  #define I2C_USE_WIRE
  #endif
#elif defined(ARDUINO_TEENSY41)
  #define ARDUINO_TYPE "TEENSY41"
  #undef USB_SERIAL
  #define USB_SERIAL SerialUSB
  #ifndef DISABLE_EEPROM
    #define DISABLE_EEPROM
  #endif
  // Teensy support for native I2C is awaiting development 
  #ifndef I2C_USE_WIRE
    #define I2C_USE_WIRE
  #endif
#elif defined(ARDUINO_ARCH_ESP8266)
  #define ARDUINO_TYPE "ESP8266"
  #warning "ESP8266 platform untested, you are on your own"
#elif defined(ARDUINO_ARCH_ESP32)
  #define ARDUINO_TYPE "ESP32"
  #ifndef DISABLE_EEPROM
  #define DISABLE_EEPROM
  #endif
#elif defined(ARDUINO_ARCH_SAMD)
  #define ARDUINO_TYPE "SAMD21"
  #undef USB_SERIAL
  #define USB_SERIAL SerialUSB
  // SAMD no EEPROM by default 
  #ifndef DISABLE_EEPROM
    #define DISABLE_EEPROM
  #endif
#elif defined(ARDUINO_ARCH_STM32)
  #define ARDUINO_TYPE "STM32"
  // STM32 no EEPROM by default 
  #ifndef DISABLE_EEPROM
    #define DISABLE_EEPROM
  #endif
  // STM32 support for native I2C is awaiting development 
  // #ifndef I2C_USE_WIRE
  // #define I2C_USE_WIRE
  // #endif

/* TODO when ready 
#elif defined(ARDUINO_ARCH_RP2040)
  #define ARDUINO_TYPE "RP2040"
*/

#else
  #define CPU_TYPE_ERROR
#endif

// replace board type if provided by compiler
#ifdef BOARD_NAME
  #undef ARDUINO_TYPE
  #define ARDUINO_TYPE BOARD_NAME
#endif

////////////////////////////////////////////////////////////////////////////////
//
// WIFI_ON: All prereqs for running with WIFI are met
// Note: WIFI_CHANNEL may not exist in early config.h files so is added here if needed.

#if ENABLE_WIFI
  #if defined(HAS_ENOUGH_MEMORY)
    #define WIFI_ON true
    #ifndef WIFI_CHANNEL
      #define WIFI_CHANNEL 1
    #endif
  #else
    #define WIFI_WARNING
    #define WIFI_ON false
  #endif
#else
  #define WIFI_ON false
#endif

#ifndef WIFI_FORCE_AP
  #define WIFI_FORCE_AP false
#else
  #if WIFI_FORCE_AP==true || WIFI_FORCE_AP==false
  #else
    #error WIFI_FORCE_AP needs to be true or false
  #endif
#endif

#if ENABLE_ETHERNET
  #if defined(HAS_ENOUGH_MEMORY)
    #define ETHERNET_ON true
  #else
    #define ETHERNET_WARNING
    #define ETHERNET_ON false
  #endif
#else
  #define ETHERNET_ON false
#endif

#if WIFI_ON && ETHERNET_ON
 #error Command Station does not support WIFI and ETHERNET at the same time.
#endif
  
////////////////////////////////////////////////////////////////////////////////
//
// This defines the speed at which the Arduino will communicate with the ESP8266 module.
// Currently only devices which can communicate at 115200 are supported.
//
#define WIFI_SERIAL_LINK_SPEED 115200

////////////////////////////////////////////////////////////////////////////////
//
// Define symbol IO_NO_HAL to reduce FLASH footprint when HAL features not required
// The HAL is disabled by default on Nano and Uno platforms, because of limited flash space.
// 
#if defined(ARDUINO_AVR_NANO) || defined(ARDUINO_AVR_UNO)
#define IO_NO_HAL // HAL too big whatever you disable otherwise
#ifndef ENABLE_VDPY
#define DISABLE_VDPY
#endif
#endif

#if __has_include ( "myAutomation.h")
  #if defined(HAS_ENOUGH_MEMORY) || defined(DISABLE_EEPROM) || defined(DISABLE_PROG)
    #define EXRAIL_ACTIVE
  #else
    #define EXRAIL_WARNING
  #endif
#endif

#endif
