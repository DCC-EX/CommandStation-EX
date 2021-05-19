/*
 *  © 2021, Gregor Baues, All rights reserved.
 *  
 *  This file is part of DCC-EX/CommandStation-EX
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
 * 
 */

#ifndef _MQTTInterface_h_
#define _MQTTInterface_h_

#if __has_include("config.h")
#include "config.h"
#else
#warning config.h not found. Using defaults from config.example.h
#include "config.example.h"
#endif
#include "defines.h"

#include <PubSubClient.h>
#include <DCCEXParser.h>
#include <Queue.h>
#include <Arduino.h>
#include <Ethernet.h>
#include <Dns.h>
#include <ObjectPool.h>

#define MAXPAYLOAD 64        // max length of a payload recieved
#define MAXDOMAINLENGTH 32   // domain name length for the broker e.g. test.mosquitto.org 

#define MAXTBUF 50            //!< max length of the buffer for building the topic name ;to be checked
#define MAXTMSG 120           //!< max length of the messages for a topic               ;to be checked PROGMEM ?
#define MAXTSTR 30            //!< max length of a topic string
#define MAXCONNECTID 40       // broker connection id length incl possible prefixes
#define CLIENTIDSIZE 6        // max length of the clientid used for connection to the broker
#define MAXRECONNECT 5        // reconnection tries before final failure
#define MAXMQTTCONNECTIONS 20 // maximum number of unique tpoics available for subscribers
#define OUT_BOUND_SIZE 256    // Size of the RingStream used to provide results from the parser and publish
#define MAX_POOL_SIZE 32      // recieved command store size

// Define Broker configurations; Values are provided in the following order
// MQTT_BROKER_PORT 9883
// MQTT_BROKER_DOMAIN "dcclms.modelrailroad.ovh"
// MQTT_BROKER_ADDRESS 51, 210, 151, 143
// MQTT_BROKER_USER "dcccs"
// MQTT_BROKER_PASSWD "dcccs$3020"
// MQTT_BROKER_CLIENTID_PREFIX "dcc$lms-"
struct MQTTBroker
{
    int port;
    IPAddress ip;
    const FSH *domain = nullptr;
    const FSH *user = nullptr;
    const FSH *pwd = nullptr;
    const FSH *prefix = nullptr;
    byte cType; // connection type to identify valid params

    IPAddress resovleBroker(const FSH *d)
    {
        DNSClient dns;
        IPAddress bip;

        char domain[MAXDOMAINLENGTH];
        strcpy_P(domain, (const char *)d);

        dns.begin(Ethernet.dnsServerIP());
        if (dns.getHostByName(domain, bip) == 1)
        {
            DIAG(F("MQTT Broker/ %s = %d.%d.%d.%d"), domain, bip[0], bip[1], bip[2], bip[3]);
        }
        else
        {
            DIAG(F("MQTT Dns lookup for %s failed"), domain);
        }
        return bip;
    }

    // all boils down to the ip address type = 1 without user authentication 2 with user authentication 
    // no ssl support !  

    // port & ip address
    MQTTBroker(int p, IPAddress i) : port(p), ip(i), cType(1){};
    // port & domain name
    MQTTBroker(int p, const FSH *d) : port(p), domain(d), cType(1)
    {
        ip = resovleBroker(d);
    };

    // port & ip & prefix
    MQTTBroker(int p, IPAddress i, const FSH *uid, const FSH *pass, const FSH *pfix) : port(p), ip(i), prefix(pfix), cType(1){};
    // port & domain & prefix
    MQTTBroker(int p, const FSH *d, const FSH *uid, const FSH *pass, const FSH *pfix) : port(p), domain(d), prefix(pfix), cType(1)
    {
        ip = resovleBroker(d);
    };

    // port & ip & user & pwd
    MQTTBroker(int p, IPAddress i, const FSH *uid, const FSH *pass) : port(p), ip(i), user(uid), pwd(pass), cType(2){};
    // port & domain & user & pwd
    MQTTBroker(int p, const FSH *d, const FSH *uid, const FSH *pass) : port(p), domain(d), user(uid), pwd(pass), cType(2)
    {
        ip = resovleBroker(d);
    };
    
    // port & ip & user & pwd & prefix
    MQTTBroker(int p, IPAddress i, const FSH *uid, const FSH *pass, const FSH *pfix) : port(p), ip(i), user(uid), pwd(pass), prefix(pfix), cType(2){};
    // port & domain & user & pwd & prefix
    MQTTBroker(int p, const FSH *d, const FSH *uid, const FSH *pass, const FSH *pfix) : port(p), domain(d), user(uid), pwd(pass), prefix(pfix), cType(2)
    {
        ip = resovleBroker(d);
    };

};

/**
 * @brief dcc-ex command as recieved via MQ
 * 
 */
typedef struct csmsg_t
{
    char cmd[MAXPAYLOAD]; // recieved command message
    byte mqsocket;        // from which mqsocket / subscriberid
} csmsg_t;

typedef struct csmqttclient_t
{
    int distant;   // random int number recieved from the subscriber
    byte mqsocket; // mqtt socket = subscriberid provided by the cs
    long topic;    // cantor(subscriber,cs) encoded tpoic used to send / recieve commands
    bool open;     // true as soon as we have send the id to the mq broker for the client to pickup
} csmqttclient_t;

enum MQTTInterfaceState
{
    INIT,
    CONFIGURED,       // server/client objects set
    CONNECTED,        // mqtt broker is connected
    CONNECTION_FAILED // Impossible to get the connection set after MAXRECONNECT tries
};

class MQTTInterface
{
private:
    // Methods
    MQTTInterface();
    MQTTInterface(const MQTTInterface &);            // non construction-copyable
    MQTTInterface &operator=(const MQTTInterface &); // non copyable

    void setup(const FSH *id, MQTTBroker *broker);  // instantiates the broker 
    void connect();                                 // (re)connects to the broker
    bool setupNetwork();                            // sets up the network connection for the PubSub system
    void loop2();

    // Members
    static MQTTInterface    *singleton;         // unique instance of the MQTTInterface object
    EthernetClient          ethClient;          // TCP Client object for the MQ Connection
    byte                    mac[6];             // simulated mac address
    IPAddress               server;             // MQTT server object
    MQTTBroker              *broker;            // Broker configuration object as set in config.h

    ObjectPool<csmsg_t, MAXPOOLSIZE> pool;      // Pool of commands recieved for the CS
    Queue<int>              in;                 // Queue of indexes into the pool according to incomming cmds
    Queue<int>              subscriberQueue;    // Queue for incomming subscribers; push the subscriber into the queue for setup in a loop cycle

    char                    clientID[(CLIENTIDSIZE * 2) + 1];   // unique ID of the commandstation; not to confused with the connectionID
    csmqttclient_t          clients[MAXMQTTCONNECTIONS];        // array of connected mqtt clients
    char                    connectID[MAXCONNECTID];            // clientId plus possible prefix if required by the broker
    uint8_t                 subscriberid = 0;                   // id assigned to a mqtt client when recieving the inital handshake; +1 at each connection

    bool                    connected = false;  // set to true if the ethernet connection is available
    MQTTInterfaceState      mqState = INIT;     // Status of the MQBroker connection
    RingStream             *outboundRing;       // Buffer for collecting the results from the command parser
    PubSubClient           *mqttClient;         // PubSub Endpoint for data exchange

public:
    static MQTTInterface *get() noexcept
    {
        return singleton;
    }

    boolean subscribe(const char *topic);

    void publish(const char *topic, const char *payload);

    ObjectPool<csmsg_t, MAXPOOLSIZE> *getPool() { return &pool; };
    Queue<int> *getIncomming() { return &in; };
    Queue<int> *getSubscriptionQueue() { return &subscriberQueue; };

    char *getClientID() { return clientID; };
    uint8_t getClientSize() { return subscriberid; }

    // initalized to 0 so that the first id comming back is 1
    // index 0 in the clients array is not used therefore
    //! improvement here to be done to save some bytes
    
    uint8_t obtainSubscriberID()
    {
        if (subscriberid == MAXMQTTCONNECTIONS)
        {
            return 0; // no more subscriber id available
        }
        return (++subscriberid);
    }

    csmqttclient_t *getClients() { return clients; };
    RingStream *getRingStream() { return outboundRing; }; // debug only

    static void setup();
    static void loop();

    ~MQTTInterface() = default;
};

#endif