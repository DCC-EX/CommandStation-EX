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

#include <Arduino.h>

#include "DIAG.h"
#include "NetworkInterface.h"
#include "Transport.h"

extern bool diagNetwork;
extern uint8_t diagNetworkClient;

template<class S, class C, class U> 
bool Transport<S,C,U>::setup(NetworkInterface *nw) {
    if (protocol == TCP) { 
        connectionPool(server);     // server should have started here so create the connection pool only for TCP though
    }
    t = new TransportProcessor();
    t->nwi = nw;                    // The TransportProcesor needs to know which Interface he is connected to
    connected = true;               // server & clients which will recieve/send data have all e setup and are available
    return true;
} 

template<class S, class C, class U> 
void Transport<S,C,U>::loop() {
    switch (protocol)
    {
    case UDP:
    {
        udpHandler();
        break;
    };
    case TCP:
    {
        tcpSessionHandler(server);    
    };
    case MQTT:
    {
        // MQTT
        break;
    };
    }
}

template<class S, class C, class U> 
void Transport<S, C, U>::connectionPool(S *server)
{
    for (int i = 0; i < Transport::maxConnections; i++)
    {
        clients[i] = server->accept();
        connections[i].client = &clients[i];
        connections[i].id = i;
        DIAG(F("\nConnection pool:       [%d:%x]"), i, connections[i].client);
    }
}

template<class S, class C, class U> 
void Transport<S, C, U>::udpHandler()
{
    // DIAG(F("UdpHandler\n"));
    int packetSize = udp->parsePacket();
    if (packetSize)
    {
        DIAG(F("\nReceived packet of size:[%d]\n"), packetSize);
        IPAddress remote = udp->remoteIP();
        DIAG(F("From:                   [%d.%d.%d.%d:"), remote[0], remote[1], remote[2], remote[3]);
        char portBuffer[6];
        DIAG(F("%s]\n"), utoa(udp->remotePort(), portBuffer, 10)); // DIAG has issues with unsigend int's so go through utoa

        // read the packet into packetBufffer
        // udp.read(buffer, MAX_ETH_BUFFER); /////////// Put into the TransportProcessor
        // terminate buffer properly
        // buffer[packetSize] = '\0';

        // DIAG(F("Command:                 [%s]\n"),buffer);
        // execute the command via the parser
        // check if we have a response if yes then
        // send the reply
        // udp.beginPacket(udp.remoteIP(), udp.remotePort());
        // parse(&udp, (byte *)buffer, true); //////////// Put into the TransportProcessor
        // udp.endPacket();

        // clear out the PacketBuffer
        // memset(buffer, 0, MAX_ETH_BUFFER); // reset PacktBuffer
        return;
    }
}

/**
 * @brief As tcpHandler but this time the connections are kept open (thus creating a statefull session) as long as the client doesn't disconnect. A connection
 * pool has been setup beforehand and determines the number of available sessions depending on the network hardware.  Commands crossing packet boundaries will be captured
 *  
 */
template<class S, class C, class U> 
void Transport<S,C,U>::tcpSessionHandler(S* server)
{
    // get client from the server
    C client = server->accept();
    
    // check for new client 
    if (client)
    {
        for (byte i = 0; i < maxConnections; i++)
        {
            if (!clients[i])
            {
                // On accept() the EthernetServer doesn't track the client anymore
                // so we store it in our client array
                clients[i] = client;
                DIAG(F("\nNew Client:             [%d:%x]"), i, clients[i]);
                break;
            }
        }
    }

    // check for incoming data from all possible clients
    for (byte i = 0; i < maxConnections; i++)
    {
        if (clients[i] && clients[i].available() > 0)
        {
            t->readStream(&connections[i]);
        }
        // stop any clients which disconnect
        for (byte i = 0; i < maxConnections; i++)
        {
            if (clients[i] && !clients[i].connected())
            {
                DIAG(F("\nDisconnect client #%d"), i);
                clients[i].stop();
                connections[i].isProtocolDefined = false;
                if (diagNetworkClient == i && diagNetwork) 
                {
                    diagNetwork = false;
                    StringFormatter::resetDiagOut();
                }
            }
        }
    }
}

template<class S, class C, class U> 
Transport<S,C,U>::Transport(){}
template<class S, class C, class U> 
Transport<S,C,U>::~Transport(){}

// explicitly instatiate to get the relevant copies for ethernet / wifi build @compile time
template class Transport<EthernetServer,EthernetClient,EthernetUDP>;
template class Transport<WiFiServer, WiFiClient, WiFiUDP>;
