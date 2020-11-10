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
#include "NetworkSetup.h"
#include "WifiSetup.h"

bool WifiSetup::setup() {
    /**
     * @todo setup using SoftwareSerial or any other Hardware Serial port on the mega (i.e. 2 or 3);
     * 
     */
    Serial1.begin(AT_BAUD_RATE);  
    WiFi.init(Serial1);

    maxConnections = MAX_WIFI_SOCK;

    if (WiFi.status() == WL_NO_MODULE)
    {
        ERR(F("Communication with WiFi module failed!"));
        return 0;
    }

    INFO(F("Waiting for connection to WiFi "));
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(1000);
        DBG(F("."));
    }
    
    INFO(F("Network Protocol: [%s]"), protocol ? "UDP" : "TCP");

    // Setup the protocol handler
    switch (protocol)
    {
    case UDPR:
    {
        connected = false;
        udp = new WiFiUDP(); 
        byte udpState = udp->begin(port);
        if (udpState) 
        {
            TRC(F("UDP status: %d"), udpState);
            maxConnections = 1;             // there is only one UDP object listening for incomming data
            connected = true;
        }
        else
        {
            ERR(F("UDP failed to start"));
            connected = false;
        }
        break;
    };
    case TCP:
    {
        server = new WiFiServer(port);
        server->begin(MAX_WIFI_SOCK, 240);
        if(server->status()) {
            connected = true;
        
        } else {
            ERR(F("\nWiFi server failed to start"));
            connected = false;
        } // Connection pool not used for WiFi
        break;
    };
    case MQTT: {
        // do the MQTT setup stuff here 
    };
    default:
    {
        ERR(F("Unkown Ethernet protocol; Setup failed"));
        connected = false;
        break;
    }
    }

    if (connected)
    {
        ip = WiFi.localIP();
        INFO(F("Local IP address:      [%d.%d.%d.%d]"), ip[0], ip[1], ip[2], ip[3]);
        INFO(F("Listening on port:     [%d]"), port);
        dnsip = WiFi.dnsServer1();
        INFO(F("DNS server IP address: [%d.%d.%d.%d] "), dnsip[0], dnsip[1], dnsip[2], dnsip[3]);
        INFO(F("Number of connections: [%d]"), maxConnections);
        return true;
    }
    return false; // something went wrong

};

WifiSetup::WifiSetup() {}
WifiSetup::WifiSetup(uint16_t p, protocolType pt ) { port = p; protocol = pt; }
WifiSetup::~WifiSetup() {}