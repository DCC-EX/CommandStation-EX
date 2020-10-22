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

#include "Ethernet.h"
#include "NetworkInterface.h"

/* some generated mac addresses as EthernetShields don't have one by default in HW.
 * Sometimes they come on a sticker on the EthernetShield then use this address otherwise
 * just choose one from below or generate one yourself. Only condition is that there is no 
 * other device on your network with the same Mac address.
 * 
 * 52:b8:8a:8e:ce:21
 * e3:e9:73:e1:db:0d
 * 54:2b:13:52:ac:0c
 * c2:d8:d4:7d:7c:cb
 * 86:cf:fa:9f:07:79
 */

/**
 * @brief Network Configuration
 * 
 */
#define MAC_ADDRESS                        \
    {                                      \
        0x52, 0xB8, 0x8A, 0x8E, 0xCE, 0x21 \
    }                            // MAC address of your networking card found on the sticker on your card or take one from above
#define IP_ADDRESS 10, 0, 0, 101 // Just in case we don't get an adress from DHCP try a static one;

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