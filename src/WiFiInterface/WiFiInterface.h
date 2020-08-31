/*
 *  WiFiInterface.h
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

#ifndef WIFIINTERFACE_WIFIINTERFACE_H_
#define WIFIINTERFACE_WIFIINTERFACE_H_

#include "../CommInterface/DCCEXParser.h"
#include "../Utils/MemStream.h"
#include <avr/pgmspace.h>

typedef void (*HTTP_CALLBACK)(Print * stream, byte * cmd);

class WiFiInterface {
public:
  static void setup(Stream* setupStream, const __FlashStringHelper* ssid, const __FlashStringHelper* password,
    const __FlashStringHelper* hostname, const __FlashStringHelper* servername, int port);
  static void loop();
  static void ATCommand(const char * command);
  static void setHTTPCallback(HTTP_CALLBACK callback);
private:
  static Stream* wifiStream;
  static bool setup2(const __FlashStringHelper* ssid, const __FlashStringHelper* password,
    const __FlashStringHelper* hostname, const __FlashStringHelper* servername, int port);
  static bool checkForOK(const unsigned int timeout, const char* waitfor, bool echo, bool escapeEcho=true);
  static bool isHTTP();
  static HTTP_CALLBACK httpCallback;
  static bool connected;
  static bool closeAfter;
  static uint8_t loopstate;
  static int  datalength;
  static int connectionId;
  static unsigned long loopTimeoutStart;
  static const uint8_t MAX_WIFI_BUFFER=250;
  static uint8_t buffer[MAX_WIFI_BUFFER];
  static MemStream streamer;
};

#endif  // WIFIINTERFACE_WIFIINTERFACE_H_