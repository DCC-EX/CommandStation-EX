/*
 *  © 2022 Harald Barth
 *  © 2020-2021 Chris Harlow
 *  © 2020 Gregor Baues
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
#ifndef CommandDistributor_h
#define CommandDistributor_h
#include "DCCEXParser.h"
#include "RingStream.h"
#include "StringBuffer.h"
#include "defines.h"

#if WIFI_ON | ETHERNET_ON 
  // Command Distributor must handle a RingStream of clients
  #define CD_HANDLE_RING
#endif 

class CommandDistributor {

public :
  static void parse(byte clientId,byte* buffer, RingStream * ring);
  static void broadcastLoco(byte slot);
  static void broadcastSensor(int16_t id, bool value);
  static void broadcastTurnout(int16_t id, bool isClosed);
  static void broadcastPower();
  static void broadcastText(const FSH * msg);
  static void forget(byte clientId);
private:
  enum clientType: byte {NONE_TYPE,COMMAND_TYPE,WITHROTTLE_TYPE};
  static void broadcastToClients(clientType type);
  static StringBuffer * broadcastBufferWriter;
  #ifdef CD_HANDLE_RING
    static RingStream * ring;
    static clientType clients[8];
  #endif
};

#endif
