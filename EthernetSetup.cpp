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

#include <Arduino.h>
#include "DIAG.h"
#include "EthernetSetup.h"

EthernetServer* EthernetSetup::setup() 
{

    DIAG(F("\nInitialize Ethernet with DHCP"));
    if (Ethernet.begin(mac) == 0)
    {
        DIAG(F("\nFailed to configure Ethernet using DHCP ... Trying with fixed IP"));
        Ethernet.begin(mac, IPAddress(IP_ADDRESS)); // default ip address

        if (Ethernet.hardwareStatus() == EthernetNoHardware)
        {
            DIAG(F("\nEthernet shield was not found. Sorry, can't run without hardware. :("));
            return 0;
        };
        if (Ethernet.linkStatus() == LinkOFF)
        {
            DIAG(F("\nEthernet cable is not connected."));
            return 0;
        }
    }

    maxConnections = MAX_SOCK_NUM;

    if (Ethernet.hardwareStatus() == EthernetW5100)
    {
        DIAG(F("\nW5100 Ethernet controller detected."));
        maxConnections = 4;  // Max supported officaly by the W5100 but i have been running over 8 as well. Perf has to be evaluated though comparing 4 vs. 8 connections
    }
    else if (Ethernet.hardwareStatus() == EthernetW5200)
    {
        DIAG(F("\nW5200 Ethernet controller detected."));
        maxConnections = 8;
    }
    else if (Ethernet.hardwareStatus() == EthernetW5500)
    {
        DIAG(F("W5500 Ethernet controller detected."));
        maxConnections = 8;
    }

   DIAG(F("\nNetwork Protocol:      [%s]"), protocol ? "UDP" : "TCP");
    switch (protocol)
    {
        case UDP:
        { 
            if (udp.begin(port)) 
            {
                maxConnections = 1;             // there is only one UDP object listening for incomming data
                connected = true;
            }
            else
            {
                DIAG(F("\nUDP client failed to start"));
                connected = false;
            }
            break;
        };
        case TCP:
        {
            server = new EthernetServer(port);
            server->begin();
            connected = true;
            break;
        };
        case MQTT:
        {
            // do the MQTT setup stuff ...
        };
        default:
        {
            DIAG(F("\nUnkown Ethernet protocol; Setup failed"));
            connected = false;
            break;
        }
    }
    if (connected)
    {
        ip = Ethernet.localIP();
        DIAG(F("\nLocal IP address:      [%d.%d.%d.%d]"), ip[0], ip[1], ip[2], ip[3]);
        DIAG(F("\nListening on port:     [%d]"), port);
        dnsip = Ethernet.dnsServerIP();
        DIAG(F("\nDNS server IP address: [%d.%d.%d.%d] "), dnsip[0], dnsip[1], dnsip[2], dnsip[3]);
        DIAG(F("\nNumber of connections: [%d]"), maxConnections);
        if( protocol == UDP ) return 0;  // no server here as we use UDB
        return server;
    }
    return 0;
}

EthernetSetup::EthernetSetup() {}
EthernetSetup::EthernetSetup(uint16_t p, protocolType pt ) { port = p; protocol = pt; }
EthernetSetup::~EthernetSetup() {}

