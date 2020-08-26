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

#ifndef WifiInterface_h
#define WifiInterface_h
#include "DCCEXParser.h"
#include "MemStream.h"
#include <Arduino.h>
#include <avr/pgmspace.h>

typedef void (*HTTP_CALLBACK)(Print * stream, byte * cmd);

class WifiInterface {

 public:
    static void setup(Stream & setupStream, const __FlashStringHelper* SSSid, const __FlashStringHelper* password,
          const __FlashStringHelper* hostname, int port);
    static void loop();
    static void ATCommand(const byte * command);
    static void setHTTPCallback(HTTP_CALLBACK callback);
    
  private:
    static Stream * wifiStream;
    static DCCEXParser parser;
    static bool setup2( const __FlashStringHelper* SSSid, const __FlashStringHelper* password,
           const __FlashStringHelper* hostname,  int port);
    static bool checkForOK(const unsigned int timeout, const char* waitfor, bool echo, bool escapeEcho=true);
    static bool isHTTP();
    static HTTP_CALLBACK httpCallback;
    static bool connected;
    static bool closeAfter;
    static byte loopstate;
    static int  datalength;
    static int connectionId;
    static unsigned long loopTimeoutStart;
    static const byte MAX_WIFI_BUFFER=250;
    static byte buffer[MAX_WIFI_BUFFER+1];
    static MemStream  streamer;
};

#endif
