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

#ifndef EthernetSetup_h
#define EthernetSetup_h

#include <Ethernet.h>
#include "NetworkSetup.h"

class EthernetSetup: public NetworkSetup {

private:

    EthernetServer*         server;
    EthernetUDP             udp;

public:

    // EthernetServer *setup(uint16_t port);
    EthernetServer *setup();

    EthernetSetup();
    EthernetSetup(uint16_t port, protocolType protocol);
    ~EthernetSetup();
};

#endif