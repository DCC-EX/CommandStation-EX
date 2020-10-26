/*
 *  © 2020 Gregor Baues. All rights reserved.
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
#ifndef NetworkSetup_h
#define NetworkSetup_h

#include <Ethernet.h>

#include "NetworkConfig.h"
#include "NetworkInterface.h"

class NetworkSetup
{
private:
public:
    IPAddress       dnsip;
    IPAddress       ip;
    uint8_t         mac[6] = MAC_ADDRESS;
    uint8_t         maxConnections;
    bool            connected;                  // semantics is that the server has successfullt started  or not; client connections will be started in the Transport object
    protocolType    protocol;
    uint16_t        port = LISTEN_PORT;         // Default port

    NetworkSetup();
    ~NetworkSetup();
};

#endif