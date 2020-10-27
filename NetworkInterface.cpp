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
#include "EthernetSetup.h"
#include "WifiSetup.h"

Transport<WiFiServer, WiFiClient, WiFiUDP> *wifiTransport;
Transport<EthernetServer, EthernetClient, EthernetUDP> *ethernetTransport;

DCCNetwork _dccNet;

void DCCNetwork::loop()
{
    for (byte i = 0; i < _tCounter; i++)
    {

        Transport<EthernetServer, EthernetClient, EthernetUDP> *e;
        Transport<WiFiServer, WiFiClient, WiFiUDP> *w;

        switch (_t[i])
        {
            case ETHERNET:
            {
                e = (Transport<EthernetServer, EthernetClient, EthernetUDP> *)transports[i];
                e->loop();
                break;
            }
            case WIFI:
            {
                w = (Transport<WiFiServer, WiFiClient, WiFiUDP> *)transports[i];
                w->loop();
                break;
            }
        }
    }
}

byte DCCNetwork::add(AbstractTransport *t, transportType transport)
{
    if (_tCounter != MAX_INTERFACES)
    {
        _t[_tCounter] = transport;
        transports[_tCounter] = t;  // add to array of network interfaces returns the index + 1 if added
        _tCounter++;                // if max intefaces is reached returns 0 for too many ...
        return _tCounter;           // normally a delete shall not be necessary as all is setup at the beginning and shall not change over a session
    }
    else
    {
        return 0;
    }
}

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
        ok = wSetup.setup();
        if (ok)
        {
            wifiTransport = new Transport<WiFiServer, WiFiClient, WiFiUDP>;
            wifiTransport->id = _dccNet.add(wifiTransport, transport);
            wifiTransport->server = wSetup.getServer();
            wifiTransport->port = port;
            wifiTransport->protocol = protocol;
            wifiTransport->transport = transport;
            wifiTransport->maxConnections = wSetup.maxConnections;
            ok = wifiTransport->setup(this);
            DIAG(F("\n\nInterface [%x] bound to transport id [%d:%x]"), this, wifiTransport->id, wifiTransport);
            break;
        };
    };
    case ETHERNET:
    {
        EthernetSetup eSetup(port, protocol);
        ethernetTransport = new Transport<EthernetServer, EthernetClient, EthernetUDP>;
        ethernetTransport->id = _dccNet.add(ethernetTransport, transport);
        ethernetTransport->server = eSetup.setup(); // returns (NULL) 0 if we run over UDP; todo: error handling if something goes wrong in the init
        ethernetTransport->port = port;
        ethernetTransport->protocol = protocol;
        ethernetTransport->transport = transport;
        ethernetTransport->maxConnections = eSetup.maxConnections; // that has been determined during the ethernet/wifi setup
        ok = ethernetTransport->setup(this);                       // start the transport i.e. setup all the client connections; We don't need the setup object anymore from here on
        DIAG(F("\n\nInterface [%x] bound to transport id [%d:%x]"), this, ethernetTransport->id, ethernetTransport);
        break;
    };
    default:
    {
        DIAG(F("\nERROR: Unknown Transport")); // Something went wrong
        break;
    }
    }

    DIAG(F("\n[%s] Transport %s ..."), transport ? "Ethernet" : "Wifi", ok ? "OK" : "Failed");
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
    // loop over all the transports in
    _dccNet.loop();

}

void NetworkInterface::setHttpCallback(HttpCallback callback)
{
    this->httpCallback = callback;
}

HttpCallback NetworkInterface::getHttpCallback()
{
    return this->httpCallback;
}

NetworkInterface::NetworkInterface() {}
NetworkInterface::~NetworkInterface() {}