/*
 *  © 2020-2021 Chris Harlow
 *  © 2020, Harald Barth.
 *  © 2023 Nathan Kellenicki
 *  All rights reserved.
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
#include "FSH.h"
#include "DCCEXParser.h"
#include <Arduino.h>
//#include <avr/pgmspace.h>

enum wifiSerialState { WIFI_NOAT, WIFI_DISCONNECTED, WIFI_CONNECTED };

class WifiInterface
{

public:
  static bool setup(long serial_link_speed, 
                          const FSH *wifiESSID,
                          const FSH *wifiPassword,
                          const FSH *hostname,
                          const int port,
                          const byte channel,
                          const bool forceAP);
  static void loop();
  static void ATCommand(HardwareSerial * stream,const byte *command);
  
private:
  static wifiSerialState setup(Stream &setupStream, const FSH *SSSid, const FSH *password,
                    const FSH *hostname, int port, byte channel, bool forceAP);
  static Stream *wifiStream;
  static DCCEXParser parser;
  static wifiSerialState setup2(const FSH *SSSid, const FSH *password,
                     const FSH *hostname, int port, byte channel, bool forceAP);
  static bool checkForOK(const unsigned int timeout, bool echo, bool escapeEcho = true);
  static bool checkForOK(const unsigned int timeout, const FSH *waitfor, bool echo, bool escapeEcho = true);
  static bool connected;
};
#endif
