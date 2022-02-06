/*
 *  © 2021 Neil McKechnie
 *  © 2021 Mike S
 *  © 2021 Fred Decker
 *  © 2020-2021 Harald Barth
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
//
#if defined (ARDUINO_ARCH_ESP8266)
#define ESP_FAMILY
//#define ESP_DEBUG
#define SLOW_ANALOG_READ
#endif

////////////////////////////////////////////////////////////////////////////////
//
// WIFI_ON: All prereqs for running with WIFI are met
// Note: WIFI_CHANNEL may not exist in early config.h files so is added here if needed.

#if (defined(ARDUINO_AVR_MEGA) || defined(ARDUINO_AVR_MEGA2560) || defined(ARDUINO_SAMD_ZERO)  || defined(TEENSYDUINO)) || defined(ARDUINO_AVR_NANO_EVERY) || defined (ESP_FAMILY))
 #define BIG_RAM
#endif 
#if ENABLE_WIFI
  #if defined(BIG_RAM)
    #define WIFI_ON true
    #ifndef WIFI_CHANNEL
      #define WIFI_CHANNEL 1
    #endif
  #else
    #define WIFI_WARNING
    #define WIFI_ON false
  #endif
#endif

#if ENABLE_ETHERNET
  #if defined(BIG_RAM)
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

#if __has_include ( "myAutomation.h")
  #if defined(BIG_RAM) || defined(DISABLE_EEPROM)
    #define EXRAIL_ACTIVE
  #else
    #define EXRAIL_WARNING
  #endif
#endif

#endif
