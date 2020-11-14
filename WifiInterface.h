/*
 *  © 2020, Chris Harlow. All rights reserved.
 *  © 2020, Harald Barth.
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
 */
#ifndef WifiInterface_h
#define WifiInterface_h
#include "config.h"
#include "DCCEXParser.h"
#include <Arduino.h>
#include <avr/pgmspace.h>

enum wifiSerialState { WIFI_NOAT, WIFI_DISCONNECTED, WIFI_CONNECTED };

class WifiInterface
{

public:
  static bool setup(long serial_link_speed, 
                          const __FlashStringHelper *wifiESSID,
                          const __FlashStringHelper *wifiPassword,
                          const __FlashStringHelper *hostname,
                          const int port = 2560);
  static void loop();
  static void ATCommand(const byte *command);

private:
  static wifiSerialState setup(Stream &setupStream, const __FlashStringHelper *SSSid, const __FlashStringHelper *password,
                    const __FlashStringHelper *hostname, int port);
  static Stream *wifiStream;
  static DCCEXParser parser;
  static wifiSerialState setup2(const __FlashStringHelper *SSSid, const __FlashStringHelper *password,
                     const __FlashStringHelper *hostname, int port);
  static bool checkForOK(const unsigned int timeout, const char *waitfor, bool echo, bool escapeEcho = true);
  static bool connected;
  static bool closeAfter;
  static byte loopstate;
  static int datalength;
  static int connectionId;
  static unsigned long loopTimeoutStart;
};
#endif
