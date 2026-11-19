/*
 *  © 2026 DCC-EX  AI generated
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

#ifndef EXNetworkUdp_h
#define EXNetworkUdp_h

#include <Arduino.h>
#include <IPAddress.h>

constexpr size_t EXNETWORK_UDP_MAX_SIZE = 1472;

struct EXNetworkUdpPacket {
  byte data[EXNETWORK_UDP_MAX_SIZE + 1];
  size_t length;
  IPAddress remoteIP;
  uint16_t localPort;
};

#endif
