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

#ifndef Transport_h
#define Transport_h

#include <Arduino.h>
#include <Ethernet.h>
#include <WiFiEspAT.h>

#include "NetworkConfig.h"
#include "NetworkInterface.h"
#include "TransportProcessor.h"


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
    uint8_t id;                             // initalized when the pool is setup
    Client *client;                         // idem
    char overflow[MAX_OVERFLOW];            // idem
    appProtocol p;                          // dynamically determined upon message reception; first message wins
    char delimiter = '\0';                  // idem
    bool isProtocolDefined = false;         // idem
    appProtocolCallback appProtocolHandler; // idem
};




template <class S, class C, class U> class Transport: public AbstractTransport
{

private:
    C                   clients[MAX_SOCK_NUM];          // Client objects created by the connectionPool
    Connection          connections[MAX_SOCK_NUM];      // All the connections build by the connectionPool
    bool                connected = false;                          
    TransportProcessor* t;                              // pointer to the object which handles the incomming/outgoing flow

    void udpHandler(U* udp);                            // Reads from a Udp socket - todo add incomming queue for processing when the flow is faster than we can process commands
    void tcpSessionHandler(S* server);                  // tcpSessionHandler -> connections are maintained open until close by the client
    void connectionPool(S* server);                     // allocates the Sockets at setup time and creates the Connections
    void connectionPool(U* udp);                        // allocates the UDP Sockets at setup time and creates the Connection
   
public:

    uint8_t         id;
    uint16_t        port;
    uint8_t         protocol;               // TCP or UDP  
    uint8_t         transport;              // WIFI or ETHERNET 
    S*              server;                 // WiFiServer or EthernetServer 
    U*              udp;                    // UDP socket object
    uint8_t         maxConnections;         // number of supported connections depending on the network equipment use

    bool setup(NetworkInterface* nwi);      // we get the callbacks from the NetworkInterface 
    void loop(); 

    bool isConnected() {
        return connected;
    }

    Transport<S,C,U>();
    ~Transport<S,C,U>();
    
};

#endif // !Transport_h