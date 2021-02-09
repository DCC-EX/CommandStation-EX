/*
 *  © 2021, Chris Harlow. All rights reserved.
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
#include <SPI.h>
#include <WiFiNINA.h>
#include "FSH.h"
#include "RingStream.h"

#define MAX_NINA_BUFFER 512
#define OUTBOUND_RING_SIZE 2048

class WifiInterface
{

public:
  static bool setup(long serial_link_speed,  // ignored 
                          const FSH *wifiESSID,
                          const FSH *wifiPassword,
                          const FSH *hostname,
                          const int port = 2560); // ignored
  static void loop();
private:
   static WiFiServer server;
   static bool connected;  
   static RingStream * outboundRing;
};
#endif
