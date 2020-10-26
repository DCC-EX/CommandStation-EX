
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

#include "NetworkConfig.h"
#include "NetworkInterface.h"

// #define DCCEX_ENABLED

typedef enum
{
    DCCEX,      // if char[0] = < opening bracket the client should be a JMRI / DCC EX client_h
    WITHROTTLE, //
    HTTP,       // If char[0] = G || P || D; if P then char [1] = U || O || A
    N_DIAG,     // '#' send form a telnet client as FIRST message will then reroute all DIAG messages to that client whilst being able to send jmri type commands
    UNKNOWN_PROTOCOL
} appProtocol;

// Needed forward declarations
struct Connection;
class TransportProcessor;

using appProtocolCallback = void (*)(Connection* c, TransportProcessor* t);

struct Connection
{
    uint8_t id;
    Client *client;
    char overflow[MAX_OVERFLOW];
    appProtocol p;
    char delimiter = '\0';
    bool isProtocolDefined = false;
    appProtocolCallback appProtocolHandler;
};

class TransportProcessor
{
private:
#ifdef DCCEX_ENABLED
    void sendToDCC(Connection *c, TransportProcessor* t, bool blocking);
#endif

public:

    NetworkInterface *nwi;
    uint8_t buffer[MAX_ETH_BUFFER];
    char command[MAX_JMRI_CMD];
    
    void readStream(Connection *c); // reads incomming packets and hands over to the commandHandle for taking the stream apart for commands

    TransportProcessor(){};
    ~TransportProcessor(){};
};

#endif // !Transport_h