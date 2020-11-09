
/*
 *  © 2020, Gregor Baues. All rights reserved.
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

#ifndef TransportProcessor_h
#define TransportProcessor_h

#include <Arduino.h>
#include <Ethernet.h>
#include <WiFiEspAT.h>

#include "RingStream.h"
#include "Transport.h"
#include "NetworkConfig.h"
#include "NetworkInterface.h"


class TransportProcessor
{
private:
#ifdef DCCEX_ENABLED
    void sendToDCC(Connection *c, TransportProcessor* t, bool blocking);
#endif
    

public:
    UDP *udp;                                 // need to carry the single UDP server instance over to the processor for sending packest
    NetworkInterface *nwi;
    uint8_t buffer[MAX_ETH_BUFFER];
    char command[MAX_JMRI_CMD];

    void readStream(Connection *c, bool read); // process incomming packets and processes them; if read = false the buffer has already been filled 

    TransportProcessor(){};
    ~TransportProcessor(){};
};

#endif // !Transport_h