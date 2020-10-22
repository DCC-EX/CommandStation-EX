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
// #include "Singelton.h"
// #include "WifiTransport.h"
#include "EthernetSetup.h"
#include "WifiSetup.h"

HttpCallback NetworkInterface::httpCallback;

Transport<WiFiServer, WiFiClient, WiFiUDP> *NetworkInterface::wifiTransport;
Transport<EthernetServer, EthernetClient, EthernetUDP> *NetworkInterface::ethernetTransport;
transportType t;

void NetworkInterface::setup(transportType transport, protocolType protocol, uint16_t port)
{

    bool ok = false;

    DIAG(F("\n[%s] Transport Setup In Progress ...\n"), transport ? "Ethernet" : "Wifi");

    // configure the Transport and get Ethernet/Wifi server up and running

    t = transport;
    switch (transport)
    {
    case WIFI:
    {
        WifiSetup wSetup(port, protocol);

        wifiTransport = new Transport<WiFiServer, WiFiClient, WiFiUDP>();
        ok = wSetup.setup();
        if (ok)
        {
            wifiTransport->server = wSetup.getServer();
            wifiTransport->port = port;
            wifiTransport->protocol = protocol;
            wifiTransport->transport = transport;
            wifiTransport->maxConnections = wSetup.maxConnections;
            ok = wifiTransport->setup();
        }
        break;
    };
    case ETHERNET:
    {
        EthernetSetup eSetup(port, protocol);

        ethernetTransport = new Transport<EthernetServer, EthernetClient, EthernetUDP>();
        ethernetTransport->server = eSetup.setup(); // returns (NULL) 0 if we run over UDP
        ethernetTransport->port = port;
        ethernetTransport->protocol = protocol;
        ethernetTransport->transport = transport;
        ethernetTransport->maxConnections = eSetup.maxConnections; // that has been determined during the ethernet/wifi setup
        ok = ethernetTransport->setup();                           // start the transport i.e. setup all the client connections; We don't need the setup object anymore from here on
        break;
    };
    default:
    {
        DIAG(F("\nERROR: Unknown Transport")); // Something went wrong
        break;
    }
    }
    DIAG(F("\n\n[%s] Transport %s ..."), transport ? "Ethernet" : "Wifi", ok ? "OK" : "Failed");
}

void NetworkInterface::setup(transportType tt, protocolType pt)
{
    NetworkInterface::setup(tt, pt, LISTEN_PORT);
}

void NetworkInterface::setup(transportType tt)
{
    NetworkInterface::setup(tt, TCP, LISTEN_PORT);
}

void NetworkInterface::setup()
{
    NetworkInterface::setup(ETHERNET, TCP, LISTEN_PORT);
}

void NetworkInterface::loop()
{
    switch (t)
    {
    case WIFI:
    {
        if (wifiTransport->isConnected()){ 
            wifiTransport->loop();
        }
        break;
    }
    case ETHERNET:
    {
        if (ethernetTransport->isConnected()) {  
            ethernetTransport->loop();
        }
        break;
    }
    }
}

void NetworkInterface::setHttpCallback(HttpCallback callback)
{
    httpCallback = callback;
}
HttpCallback NetworkInterface::getHttpCallback()
{
    return httpCallback;
}

NetworkInterface::NetworkInterface()
{
    // DIAG(F("NetworkInterface created "));
}

NetworkInterface::~NetworkInterface()
{
    // DIAG(F("NetworkInterface destroyed"));
}