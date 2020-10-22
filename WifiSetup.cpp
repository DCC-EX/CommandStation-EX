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
#include "WifiSetup.h"

bool WifiSetup::setup() {
    Serial1.begin(AT_BAUD_RATE);
    WiFi.init(Serial1);

    maxConnections = MAX_WIFI_SOCK;

    if (WiFi.status() == WL_NO_MODULE)
    {
        DIAG(F("Communication with WiFi module failed!\n"));
        return 0;
    }

    DIAG(F("Waiting for connection to WiFi "));
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(1000);
        DIAG(F("."));
    }
    // Setup the protocol handler
    DIAG(F("\n\nNetwork Protocol:      [%s]"), protocol ? "UDP" : "TCP");

    switch (protocol)
    {
    case UDP:
    {
        DIAG(F("\nUDP over Wifi is not yet supported\n"));
        connected = false;
        /*
        udp = new WiFiUDP(); 
        
        maxConnections = 1;
        // DIAG(F("Wifi/UDP: [%x:%d]"), udp, port);
        if (udp->begin(port))     // no need to call begin for the WiFiEspAT library but doesn't run properly in the context of the application
        {
            connected = true;
        }
        else
        {
            DIAG(F("\nUDP client failed to start"));
            connected = false;
        }
        */
        break;
    };
    case TCP:
    {
        server = new WiFiServer(port);
        server->begin(MAX_WIFI_SOCK, 240);
        if(server->status()) {
            connected = true;
        
        } else {
            DIAG(F("\nWiFi server failed to start"));
            connected = false;
        } // Connection pool not used for WiFi
        break;
    };
    case MQTT: {
        // do the MQTT setup stuff here 
    };
    default:
    {
        DIAG(F("Unkown Ethernet protocol; Setup failed"));
        connected = false;
        break;
    }
    }

    if (connected)
    {
        ip = WiFi.localIP();
        DIAG(F("\nLocal IP address:      [%d.%d.%d.%d]"), ip[0], ip[1], ip[2], ip[3]);
        DIAG(F("\nListening on port:     [%d]"), port);
        dnsip = WiFi.dnsServer1();
        DIAG(F("\nDNS server IP address: [%d.%d.%d.%d] "), dnsip[0], dnsip[1], dnsip[2], dnsip[3]);
        DIAG(F("\nNumber of connections: [%d]"), maxConnections);
        if( protocol == UDP ) return 0;  // no server here as we use UDP
        return true;
    }
    // something went wrong
    return false;

};


WifiSetup::WifiSetup() {}
WifiSetup::WifiSetup(uint16_t p, protocolType pt ) { port = p; protocol = pt; }
WifiSetup::~WifiSetup() {}