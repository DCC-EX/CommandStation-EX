/*
 * © 2020 Gregor Baues. All rights reserved.
 *  
 * This is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the 
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 * 
 * See the GNU General Public License for more details <https://www.gnu.org/licenses/>
 */

#include <Arduino.h>

#include "NetworkDiag.h"
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
    _nLogLevel = 4;    // set the log level to ERROR during setup to get proper information 

    INFO(F("[%s] Transport Setup In Progress ..."), transport ? "Ethernet" : "Wifi");

    // configure the Transport and get Ethernet/Wifi server up and running

    t = transport;
    switch (transport)
    {
    case WIFI:
    {
        WifiSetup wSetup(port, protocol);
        if (wSetup.setup())
        {
            wifiTransport = new Transport<WiFiServer, WiFiClient, WiFiUDP>;
            wifiTransport->id = _dccNet.add(wifiTransport, transport);
            wifiTransport->server = wSetup.getTCPServer();
            wifiTransport->port = port;
            wifiTransport->protocol = protocol;
            wifiTransport->transport = transport;
            wifiTransport->udp = wSetup.getUDPServer();             // 0 if TCP is used
            wifiTransport->maxConnections = wSetup.maxConnections;
            ok = wifiTransport->setup(this);
            DBG(F("Interface [%x] bound to transport id [%d:%x]"), this, wifiTransport->id, wifiTransport);
        } else {
            ok = false;
        }
        break;
    };
    case ETHERNET:
    {
        EthernetSetup eSetup(port, protocol);
        if( eSetup.setup() ) {
            ethernetTransport = new Transport<EthernetServer, EthernetClient, EthernetUDP>;
            ethernetTransport->id = _dccNet.add(ethernetTransport, transport);
            ethernetTransport->server = eSetup.getTCPServer();          // 0 if UDP is used
            ethernetTransport->port = port;
            ethernetTransport->protocol = protocol;
            ethernetTransport->transport = transport;
            ethernetTransport->udp = eSetup.getUDPServer();             // 0 if TCP is used
            ethernetTransport->maxConnections = eSetup.maxConnections;  // that has been determined during the ethernet/wifi setup
            ok = ethernetTransport->setup(this);                        // start the transport i.e. setup all the client connections; We don't need the setup object anymore from here on
            DBG(F("Interface [%x] bound to transport id [%d:%x]"), this, ethernetTransport->id, ethernetTransport);
        } else {
            ok = false;
        }
        break;
    };
    default:
    {
        ERR(F("ERROR: Unknown Transport")); // Something went wrong
        break;
    }
    }

    INFO(F("[%s] Transport %s ..."), transport ? "Ethernet" : "Wifi", ok ? "OK" : "Failed");
    _nLogLevel = 0;   // set loging back to silent;
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