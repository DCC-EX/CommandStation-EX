
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

#ifndef Transport_h
#define Transport_h

#include <Arduino.h>
#include <Ethernet.h>
#include <WiFiEspAT.h>

#include <NetworkInterface.h>
#include "TransportProcessor.h"

 
#define MAX_SOCK_NUM 8                      // Maximum number of sockets allowed for any WizNet based EthernetShield. The W5100 only supports 4
#define MAX_WIFI_SOCK 5                     // ESP8266 doesn't support more than 5 connections in //
#define LISTEN_PORT 2560                    // default listen port for the server


#define MAC_ADDRESS                        \
    {                                      \
        0x52, 0xB8, 0x8A, 0x8E, 0xCE, 0x21 \
    }                            // MAC address of your networking card found on the sticker on your card or take one from above
#define IP_ADDRESS 10, 0, 0, 101 // Just in case we don't get an adress from DHCP try a static one;

// Emulate Serial1 on pins 6/7 if not present
#if defined(ARDUINO_ARCH_AVR) && !defined(HAVE_HWSERIAL1)
#include <SoftwareSerial.h>
SoftwareSerial Serial1(6, 7); // RX, TX
#define AT_BAUD_RATE 9600
#else
#define AT_BAUD_RATE 115200
#endif

template <class S, class C, class U> class Transport 
{

private:
    C               clients[MAX_SOCK_NUM];              // Client objects created by the connectionPool
    Connection      connections[MAX_SOCK_NUM];          // All the connections build by the connectionPool
    bool            connected = false;                          
    TransportProcessor* t;                              // pointer to the object which handles the incomming flow

    void udpHandler();                                  // Reads from a Udp socket - todo add incomming queue for processing when the flow is faster than we can process commands
    void tcpSessionHandler(S* server);                  // tcpSessionHandler -> connections are maintained open until close by the client
    void connectionPool(S* server);                     // allocates the Sockets at setup time and creates the Connections
   
public:
    uint16_t        port;
    uint8_t         protocol;               // TCP or UDP  
    uint8_t         transport;              // WIFI or ETHERNET 
    S*              server;                 // WiFiServer or EthernetServer 
    U*              udp;                    // UDP socket object
    uint8_t         maxConnections;         // number of supported connections depending on the network equipment used

    bool setup();
    void loop(); 

    bool isConnected() {
        return connected;
    }

    Transport<S,C,U>();
    ~Transport<S,C,U>();
    
};

#endif // !Transport_h