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

#if __has_include("config.h")
#include "config.h"
#else
#warning config.h not found. Using defaults from config.example.h
#include "config.example.h"
#endif
#include "defines.h"

#include <errno.h>
#include <limits.h>

#include "MQTTInterface.h"
#include "MQTTBrokers.h"
#include "DCCTimer.h"
#include "CommandDistributor.h"

MQTTInterface *MQTTInterface::singleton = NULL;

long cantorEncode(long a, long b)
{
    return (((a + b) * (a + b + 1)) / 2) + b;
}

void cantorDecode(int32_t c, int *a, int *b)
{

    int w = floor((sqrt(8 * c + 1) - 1) / 2);
    int t = (w * (w + 1)) / 2;
    *b = c - t;
    *a = w - *b;
}

void MQTTInterface::setup()
{
    singleton = new MQTTInterface();

    if (!singleton->connected)
    {
        singleton = NULL;
    }
    if (Diag::MQTT)
        DIAG(F("MQTT Interface instance: [%x] - Setup done"), singleton);
};

MQTTInterface::MQTTInterface()
{

    this->connected = this->setupNetwork();
    if (!this->connected)
    {
        DIAG(F("Network setup failed"));
    }
    else
    {
        this->setup(CSMQTTBROKER);
    }
    this->outboundRing = new RingStream(OUT_BOUND_SIZE);
};

/**
 * @brief MQTT Interface callback recieving all incomming messages from the PubSubClient
 * 
 * @param topic 
 * @param payload 
 * @param length 
 */
void mqttCallback(char *topic, byte *payload, unsigned int length)
{
    MQTTInterface *mqtt = MQTTInterface::get();
    auto clients = mqtt->getClients();

    errno = 0;
    payload[length] = '\0'; // make sure we have the string terminator in place
    if (Diag::MQTT)
        DIAG(F("MQTT Callback:[%s] [%s] [%d] on interface [%x]"), topic, (char *)payload, length, mqtt);
    switch (payload[0])
    {
    case '<':   // Recieved a DCC-EX Command
    {
        const char s[2] = "/"; // topic delimiter is /
        char *token;
        byte mqsocket;

        /* get the first token = ClientID */
        token = strtok(topic, s);
        /* get the second token = topicID */
        token = strtok(NULL, s);
        if (token == NULL)
        {
            DIAG(F("MQTT Can't identify sender #1; command send on wrong topic"));
            return;
            // don't do anything as we wont know where to send the results
            // normally the topicid shall be valid as we only have subscribed to that one and nothing else
            // comes here; The only issue is when recieveing on the open csid channel ( which stays open in order to
            // able to accept other connections )
        }
        else
        {
            auto topicid = atoi(token);
            // verify that there is a MQTT client with that topic id connected
            bool isClient = false;
            // check in the array of clients if we have one with the topicid
            // start at 1 as 0 is not allocated as mqsocket
            for (int i = 1; i <= mqtt->getClientSize(); i++)
            // for (int i = 1; i <= subscriberid; i++)
            {
                if (clients[i].topic == topicid)
                {
                    isClient = true;
                    mqsocket = i;
                    break;
                }
            }
            if (!isClient)
            {
                // no such client connected
                DIAG(F("MQTT Can't identify sender #2; command send on wrong topic"));
                return;
            }
        }
        // if we make it until here we dont even need to test the last "cmd" element from the topic as there is no
        // subscription for anything else

        // Prepare the DCC-EX command
        csmsg_t tm; // topic message

        if (length >= MAXPAYLOAD)
        {
            DIAG(F("MQTT Command too long (> [%d] characters)"), MAXPAYLOAD);
        }
        memset(tm.cmd, 0, MAXPAYLOAD);                // Clean up the cmd buffer  - should not be necessary
        strlcpy(tm.cmd, (char *)payload, length + 1); // Message payload
        tm.mqsocket = mqsocket;                       // On which socket did we recieve the mq message
        int idx = mqtt->getPool()->setItem(tm);       // Add the recieved command to the pool

        if (idx == -1)
        {
            DIAG(F("MQTT Command pool full. Could not handle recieved command."));
            return;
        }

        mqtt->getIncomming()->push(idx); // Add the index of the pool item to the incomming queue

        if (Diag::MQTT)
            DIAG(F("MQTT Message arrived [%s]: [%s]"), topic, tm.cmd);
        break;
    }
    case 'm':   // Recieved an MQTT Connection management message
    {
        switch (payload[1])
        {
        case 'i':   // Inital handshake message to create the tunnel
        {
            char buffer[MAXPAYLOAD];
            char *tmp = (char *)payload + 3;
            strlcpy(buffer, tmp, length);
            buffer[length - 4] = '\0';

            // DIAG(F("MQTT buffer %s - %s - %s - %d"), payload, tmp, buffer, length);

            auto distantid = strtol(buffer, NULL, 10);

            if (errno == ERANGE || distantid > UCHAR_MAX)
            {
                DIAG(F("MQTT Invalid Handshake ID; must be between 0 and 255"));
                return;
            }
            if (distantid == 0)
            {
                DIAG(F("MQTT Invalid Handshake ID"));
                return;
            }

            // Create a new MQTT client

            auto subscriberid = mqtt->obtainSubscriberID(); // to be used in the parsing process for the clientid in the ringbuffer

            if (subscriberid == 0)
            {
                DIAG(F("MQTT no more connections are available"));
                return;
            }

            auto topicid = cantorEncode((long)subscriberid, (long)distantid);
            DIAG(F("MQTT Client connected : subscriber [%d] : distant [%d] : topic: [%d]"), subscriberid, (int)distantid, topicid);

            // extract the number delivered from & initalize the new mqtt client object
            clients[subscriberid] = {(int)distantid, subscriberid, topicid, false}; // set to true once the channels are available

            auto sq = mqtt->getSubscriptionQueue();
            sq->push(subscriberid);

            return;
        }
        default:    // Invalid message
        {
            // ignore
            return;
        }
        }
    }
    default:    // invalid command / message
    {   
        // this may be the echo comming back on the main channel to which we are also subscribed
        // si just ignore for now 
        // DIAG(F("MQTT Invalid DCC-EX command: %s"), (char *)payload);
        break;
    }
    }
}

/**
 * @brief Copies an byte array to a hex representation as string; used for generating the unique Arduino ID
 * 
 * @param array array containing bytes
 * @param len length of the array
 * @param buffer buffer to which the string will be written; make sure the buffer has appropriate length
 */
static void array_to_string(byte array[], unsigned int len, char buffer[])
{
    for (unsigned int i = 0; i < len; i++)
    {
        byte nib1 = (array[i] >> 4) & 0x0F;
        byte nib2 = (array[i] >> 0) & 0x0F;
        buffer[i * 2 + 0] = nib1 < 0xA ? '0' + nib1 : 'A' + nib1 - 0xA;
        buffer[i * 2 + 1] = nib2 < 0xA ? '0' + nib2 : 'A' + nib2 - 0xA;
    }
    buffer[len * 2] = '\0';
}

/**
 * @brief Connect to the MQTT broker; Parameters for this function are defined in 
 * like the motoshield configurations there are mqtt broker configurations in config.h
 * 
 * @param id Name provided to the broker configuration
 * @param b MQTT broker object containing the main configuration parameters
 */
void MQTTInterface::setup(const FSH *id, MQTTBroker *b)
{

    //Create the MQTT environment and establish inital connection to the Broker
    broker = b;

    DIAG(F("MQTT Connect to %S at %S/%d.%d.%d.%d:%d"), id, broker->domain, broker->ip[0], broker->ip[1], broker->ip[2], broker->ip[3], broker->port);

    // initalize MQ Broker
    mqttClient = new PubSubClient(broker->ip, broker->port, mqttCallback, ethClient);

    if (Diag::MQTT)
        DIAG(F("MQTT Client created ok..."));
    array_to_string(mac, CLIENTIDSIZE, clientID);
    DIAG(F("MQTT Client ID : %s"), clientID);
    connect();                                     // inital connection as well as reconnects
}

/**
 * @brief MQTT broker connection / reconnection
 * 
 */
void MQTTInterface::connect()
{

    int reconnectCount = 0;
    connectID[0] = '\0';

    // Build the connect ID : Prefix + clientID
    if (broker->prefix != nullptr)
    {
        strcpy_P(connectID, (const char *)broker->prefix);
    }
    strcat(connectID, clientID);

    // Connect to the broker
    DIAG(F("MQTT %s (re)connecting ..."), connectID);
    while (!mqttClient->connected() && reconnectCount < MAXRECONNECT)
    {
        switch (broker->cType)
        {
        // no uid no pwd
        case 1:
        { // port(p), ip(i), domain(d),
            DIAG(F("MQTT Broker connecting anonymous ..."));
            if (mqttClient->connect(connectID))
            {
                DIAG(F("MQTT Broker connected ..."));
                auto sub = subscribe(clientID);                // set up the main subscription on which we will recieve the intal mi message from a subscriber
                if (Diag::MQTT)
                    DIAG(F("MQTT subscriptons %s..."), sub ? "ok" : "failed");
                mqState = CONNECTED;
            }
            else
            {
                DIAG(F("MQTT broker connection failed, rc=%d, trying to reconnect"), mqttClient->state());
                reconnectCount++;
            }
            break;
        }
        // with uid passwd
        case 2:
        {   
            DIAG(F("MQTT Broker connecting with uid/pwd ..."));
            char user[strlen_P((const char *) broker->user)];
            char pwd[strlen_P((const char *) broker->pwd)];

            // need to copy from progmem to lacal
            strcpy_P(user, (const char *)broker->user);
            strcpy_P(pwd, (const char *)broker->pwd);

            if (mqttClient->connect(connectID, user, pwd))
            {
                DIAG(F("MQTT Broker connected ..."));
                auto sub = subscribe(clientID);                // set up the main subscription on which we will recieve the intal mi message from a subscriber
                if (Diag::MQTT)
                    DIAG(F("MQTT subscriptons %s..."), sub ? "ok" : "failed");
                mqState = CONNECTED;
            }
            else
            {
                DIAG(F("MQTT broker connection failed, rc=%d, trying to reconnect"), mqttClient->state());
                reconnectCount++;
            }
            break;          
            // ! add last will messages for the client     
            // (connectID, MQTT_BROKER_USER, MQTT_BROKER_PASSWD, "$connected", 0, true, "0", 0))
        }
        }
        if (reconnectCount == MAXRECONNECT)
        {
            DIAG(F("MQTT Connection aborted after %d tries"), MAXRECONNECT);
            mqState = CONNECTION_FAILED;
        }
    }
}

/**
 * @brief for the time being only one topic at the root 
 * which is the unique clientID from the MCU
 * QoS is 0 by default
 * 
 * @param topic to subsribe to
 * @return boolean true if successful false otherwise
 */
boolean MQTTInterface::subscribe(const char *topic)
{
    auto res = mqttClient->subscribe(topic);
    return res;
}

void MQTTInterface::publish(const char *topic, const char *payload)
{
    mqttClient->publish(topic, payload);
}

/**
 * @brief Connect the Ethernet network; 
 * 
 * @return true if connections was successful
 */
bool MQTTInterface::setupNetwork()
{

    // setup Ethernet connection first

    DCCTimer::getSimulatedMacAddress(mac);

#ifdef IP_ADDRESS
    Ethernet.begin(mac, IP_ADDRESS);
#else
    if (Ethernet.begin(mac) == 0)
    {
        DIAG(F("Ethernet.begin FAILED"));
        return false;
    }
#endif
    DIAG(F("Ethernet.begin OK."));
    if (Ethernet.hardwareStatus() == EthernetNoHardware)
    {
        DIAG(F("Ethernet shield not found"));
        return false;
    }
    if (Ethernet.linkStatus() == LinkOFF)
    {
        DIAG(F("Ethernet cable not connected"));
        return false;
    }

    IPAddress ip = Ethernet.localIP(); // reassign the obtained ip address
    DIAG(F("IP: %d.%d.%d.%d"), ip[0], ip[1], ip[2], ip[3]);
    DIAG(F("Port:%d"), IP_PORT);

    return true;
}

/**
 * @brief handle the incomming queue in the loop
 * 
 */
void inLoop(Queue<int> &in, ObjectPool<csmsg_t, MAXPOOLSIZE> &pool, RingStream *outboundRing)
{

    bool state;
    if (in.count() > 0)
    {
        // pop a command index from the incomming queue and get the command from the pool
        int idx = in.pop();
        csmsg_t *c = pool.getItem(idx, &state);

        // execute the command and collect results
        outboundRing->mark((uint8_t)c->mqsocket);
        CommandDistributor::parse(c->mqsocket, (byte *)c->cmd, outboundRing);
        outboundRing->commit();

        // free the slot in the command pool
        pool.returnItem(idx);
    }
}

/**
 * @brief handle the outgoing messages in the loop
 * 
 */
void outLoop(PubSubClient *mq)
{
    // handle at most 1 outbound transmission
    MQTTInterface *mqtt = MQTTInterface::get();

    auto clients = mqtt->getClients();
    auto outboundRing = mqtt->getRingStream();

    int mqSocket = outboundRing->read();
    if (mqSocket >= 0) // mqsocket / clientid can't be 0 ....
    {
        int count = outboundRing->count();
        char buffer[MAXTSTR];
        buffer[0] = '\0';
        sprintf(buffer, "%s/%d/result", mqtt->getClientID(), (int)clients[mqSocket].topic);
        if (Diag::MQTT)
            DIAG(F("MQTT publish to mqSocket=%d, count=:%d on topic %s"), mqSocket, count, buffer);

        if (mq->beginPublish(buffer, count, false))
        {
            for (; count > 0; count--)
            {
                mq->write(outboundRing->read());
            }
        }
        else
        {
            DIAG(F("MQTT error start publishing result)"));
        };

        if (!mq->endPublish())
        {
            DIAG(F("MQTT error finalizing published result)"));
        };
    }
}

/**
 * @brief check if there are new subscribers connected and create the channels
 * 
 * @param sq        if the callback captured a client there will be an entry in the sq with the subscriber number
 * @param clients   the clients array where we find the info to setup the subsciptions and print out the publish topics for info
 */
void checkSubscribers(Queue<int> &sq, csmqttclient_t *clients)
{
    MQTTInterface *mqtt = MQTTInterface::get();
    if (sq.count() > 0)
    {
        // new subscriber
        auto s = sq.pop();

        char tbuffer[(CLIENTIDSIZE * 2) + 1 + MAXTSTR];
        sprintf(tbuffer, "%s/%ld/cmd", mqtt->getClientID(), clients[s].topic);

        auto ok = mqtt->subscribe(tbuffer);

        if (Diag::MQTT)
            DIAG(F("MQTT new subscriber topic: %s %s"), tbuffer, ok ? "OK" : "NOK");

        // send the topic on which the CS will listen for commands and the ones on which it will publish for the connecting
        // client to pickup. Once the connecting client has setup other topic setup messages on the main channel shall be
        // ignored
        // JSON message { init: <number> channels: {result: <string>, diag: <string> }}

        char buffer[MAXPAYLOAD*2];
        memset(buffer, 0, MAXPAYLOAD*2);

        // sprintf(buffer, "mc(%d,%ld)", (int)clients[s].distant, clients[s].topic);

        sprintf(buffer, "{ \"init\": %d, \"subscribeto\": {\"result\": \"%s/%ld/result\" , \"diag\": \"%s/%ld/diag\" }, \"publishto\": {\"cmd\": \"%s/%ld/cmd\" } }", 
                        (int)clients[s].distant, 
                        mqtt->getClientID(), 
                        clients[s].topic, 
                        mqtt->getClientID(), 
                        clients[s].topic, 
                        mqtt->getClientID(), 
                        clients[s].topic
                );

        if (Diag::MQTT)
            DIAG(F("MQTT channel setup message: [%s]"), buffer);
            
        mqtt->publish(mqtt->getClientID(), buffer);

        // on the cs side all is set and we declare that the cs is open for business
        clients[s].open = true;
    }
}

void MQTTInterface::loop()
{

    if (!singleton)
        return;
    singleton->loop2();
}


bool showonce = false;
void MQTTInterface::loop2()
{
    // Connection impossible so just don't do anything
    if (singleton->mqState == CONNECTION_FAILED)
    {
        if(!showonce) {
            DIAG(F("MQTT connection failed..."));
            showonce = true;
        }
        return;
    }
    if (!mqttClient->connected())
    {
        DIAG(F("MQTT no connection trying to reconnect ..."));
        connect();
    }
    if (!mqttClient->loop())
    {
        DIAG(F("mqttClient returned with error; state: %d"), mqttClient->state());
        return;
    };

    checkSubscribers(subscriberQueue, clients);
    inLoop(in, pool, outboundRing);
    outLoop(mqttClient);
}