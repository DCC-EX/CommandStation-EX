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

#ifndef NetworkInterface_h
#define NetworkInterface_h

#include <Arduino.h>

#include "Transport.h"
#include "HttpRequest.h"

typedef enum protocolType {
    TCP,
    UDP,
    MQTT                
} protocolType;

typedef enum transportType {
    WIFI,                   // using an AT (Version >= V1.7) command enabled ESP8266 not to be used in conjunction with the WifiInterface though! not tested for conflicts
    ETHERNET                // using the EthernetShield
} transoprtType;

// typedef void (*HttpCallback)(ParsedRequest *req, Client *client);
using HttpCallback = void(*)(ParsedRequest *req, Client *client);

class NetworkInterface
{
private:

    static Transport<WiFiServer,WiFiClient,WiFiUDP>* wifiTransport;
    static Transport<EthernetServer,EthernetClient,EthernetUDP>* ethernetTransport;
    static HttpCallback httpCallback;

public:
    
    static void setHttpCallback(HttpCallback callback);
    static HttpCallback getHttpCallback();
    static void setup(transportType t, protocolType p, uint16_t port);        // specific port nummber
    static void setup(transportType t, protocolType p);                       // uses default port number
    static void setup(transportType t);                                       // defaults for protocol/port 
    
    static void setup();                                                      // defaults for all as above plus CABLE (i.e. using EthernetShield ) as default
    static void loop();

    NetworkInterface();
    ~NetworkInterface();
};

#endif