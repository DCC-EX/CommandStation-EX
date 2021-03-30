/*
    © 2020, Harald Barth.

    This file is part of CommandStation-EX

    This is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    It is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with CommandStation.  If not, see <https://www.gnu.org/licenses/>.

*/

////////////////////////////////////////////////////////////////////////////////
//
// WIFI_ON: All prereqs for running with WIFI are met
// Note: WIFI_CHANNEL may not exist in early config.h files so is added here if needed.

#if ENABLE_WIFI && (defined(ARDUINO_AVR_MEGA) || defined(ARDUINO_AVR_MEGA2560) || defined(ARDUINO_SAMD_ZERO)  || defined(TEENSYDUINO))
#define WIFI_ON true
#ifndef WIFI_CHANNEL
#define WIFI_CHANNEL 1
#endif
#else
#define WIFI_ON false
#endif

#if ENABLE_ETHERNET && (defined(ARDUINO_AVR_MEGA) || defined(ARDUINO_AVR_MEGA2560) || defined(ARDUINO_SAMD_ZERO) || defined(TEENSYDUINO)) 
#define ETHERNET_ON true
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
