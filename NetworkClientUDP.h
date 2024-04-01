/*
 *  © 2024 Harald Barth
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
#ifdef Z21_PROTOCOL
#include <WiFi.h>
#include <WiFiClient.h>

class NetworkClientUDP {
public:
  NetworkClientUDP() {
  };
  bool ok() {
    return (inUse);
  };
  
  bool inUse = true;
  bool connected = false;
  IPAddress remoteIP;
  int remotePort;
  
  static WiFiUDP client;
};
#endif
