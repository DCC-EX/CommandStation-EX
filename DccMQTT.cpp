/**
 * @file DccMQTT.cpp
 * @author Gregor Baues
 * @brief  MQTT protocol controller for DCC-EX. Sets up and maintains the connection to the MQTT broker incl setting up the topics.
 * Topics are created specifically for the command station on which the code runs. Manages subsriptions as well as recieving/sending of messages on the different topics.
 * @version 0.1
 * @date 2020-07-08
 * 
 * @copyright Copyright (c) 2020
 *
 *  This is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  It is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details <https://www.gnu.org/licenses/>.
 * 
 * Notes:
 * At most 20 channels are allowed (MAXMQTTCONNECTIONS ) this can be pushed put to 255 
 * notwithstanding memeory consumption for maintaining the session info
 * 
 * once the channel is open i.E the topic id has been send via MQ the CS will subscribe to 
 * clientid/topicid/cmd for recieveing commands and publish on 
 * clientid/topicid/diag
 * clientid/topicid/result
 * i.e. the consumer connected via MQTT to the cs has/should to subscribe to 
 * clientid/topicid/diag
 * clientid/topicid/result
 * and publish on clientid/topicid/cmd
 */

#if __has_include("config.h")
#include "config.h"
#else
#warning config.h not found. Using defaults from config.example.h
#include "config.example.h"
#endif
#include "defines.h"

#include <Arduino.h>
#include <avr/pgmspace.h>
#include <EthernetInterface.h>
#include <PubSubClient.h>
#include <DIAG.h>
#include <Ethernet.h>
#include <Dns.h>
#include <DCCTimer.h>
#include <DccMQTT.h>
#include <CommandDistributor.h>
#include <Queue.h>
#include <ObjectPool.h>
#include <errno.h>
#include <limits.h>
#include <inttypes.h>
#include <freeMemory.h>

//---------
// Variables
//---------

DccMQTT DccMQTT::singleton;
auto mqtt = DccMQTT::get();

// The RingBuffer size / no Withrottle over MQ so we don't need the huge buffer as for ethernet
#define OUT_BOUND_SIZE 256

// pairing functions used for creating a client identifier
// when a external system connects via MQ to the CS i.e. subscribes to the main channel the first message to be published
// shall be a admin message with a random number
// the cs will based on a counter use a second number to create the cantor encoding of both numbers and publish the cantor code
// this message will be seen by all throttles and they can decode the number which provides the first number they send and the
// second number  to be used as tpoic for the external system from then on. The CS will recieve on all topics the commands and
// during processing then send the replies to the topic from which the command was recieved.
// Thus the main channel shall not be used for any p2p coms ev just for broadcast from the CS to the subscribed clients

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

// callback when a message arrives from the broker; push cmd into the incommming queue
void mqttCallback(char *topic, byte *payload, unsigned int length)
{

  auto clients = mqtt->getClients();
  errno = 0;
  payload[length] = '\0'; // make sure we have the string terminator in place

  DIAG(F("MQTT Callback:[%s] [%s] [%d]"), topic, (char *)payload, length);

  switch (payload[0])
  {
  case '<':
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
      DIAG(F("Can't identify sender #1; command send on wrong topic"));
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
        DIAG(F("Can't identify sender #2; command send on wrong topic"));
        return;
      }
    }
    // if we make it until here we dont even need to test the last "cmd" element from the topic as there is no
    // subscription for anything else

    // DIAG(F("MQTT Message arrived on [%s]: [%d]"), buf, topicid);

    // Prepare the DCC-EX command
    auto pool = mqtt->getPool();   // message pool
    auto q = mqtt->getIncomming(); // incomming queue

    csmsg_t tm; // topic message

    if (length + 1 > MAXPAYLOAD)
    {
      DIAG(F("MQTT Command too long (> %d characters)"), MAXPAYLOAD);
    }
    strlcpy(tm.cmd, (char *)payload, length + 1); // message payload
    tm.mqsocket = mqsocket;                       // on which socket did we recieve the mq message
    int idx = pool->setItem(tm);                  // Add the recieved command to the pool

    if (idx == -1)
    {
      DIAG(F("MQTT Command pool full. Could not handle recieved command."));
      return;
    }

    q->push(idx); // Add the index of the pool item to the incomming queue

    DIAG(F("MQTT Message arrived [%s]: [%s]"), topic, tm.cmd);

    break;
  }
  case 'm':
  {
    switch (payload[1])
    {
    case 'i':
    {

      char buffer[30];
      char *tmp = (char *)payload + 3;
      strlcpy(buffer, tmp, length);
      buffer[length - 4] = '\0';

      // DIAG(F("MQTT buffer %s - %s - %s - %d"), payload, tmp, buffer, length);

      auto distantid = strtol(buffer, NULL, 10);

      if (errno == ERANGE || distantid > UCHAR_MAX)
      {
        DIAG(F("Invalid Handshake ID; must be between 0 and 255"));
        return;
      }
      if (distantid == 0)
      {
        DIAG(F("Invalid Handshake ID"));
        return;
      }
      // ---------------------------
      // Create a new MQTT client
      // ---------------------------

      // check in the clients if the distantid has been set already somewhere
      // if so we either have a new one with the same id then we have a collision -> publish a collision
      // or its the same i.e; the message comming back as we are subscribed -> stop here

      // All is ok so set up the channel; MQTT Ctrl command

      auto subscriberid = DccMQTT::get()->obtainSubscriberID(); // to be used in the parsing process for the clientid in the ringbuffer

      if (subscriberid == 0)
      {
        DIAG(F("MQTT no more connections are available"));
        return;
      }

      auto topicid = cantorEncode((long)subscriberid, (long)distantid);
      DIAG(F("MQTT Ctrl Message arrived [%s] : subscriber [%d] : distant [%d] : topic: [%d]"), buffer, subscriberid, (int)distantid, topicid);
      // extract the number delivered from
      // we need to check if the id we got from the client has been used allready and if yes reject and ask for a different one

      // initalize the new mqtt client object
      clients[subscriberid] = {(int)distantid, subscriberid, topicid, true};

      // add/subcribe to the topic for listening on cmds recieved via the channel for the client with
      // subscriberid as identifier

      char tbuffer[(CLIENTIDSIZE * 2) + 1 + MAXTOPICLENGTH];
      mqtt->getSubscriberTopic(subscriberid, tbuffer);
      auto ok = mqtt->subscribe(tbuffer);
      DIAG(F("MQTT new subscriber topic: %s %s"), tbuffer, ok ? "OK" : "NOK");

      // send the topicid on which the CS will listen for commands to the MQTT client on the root topic
      memset(buffer, 0, 30);
      sprintf(buffer, "mc(%d,%ld)", (int)distantid, topicid);
      DIAG(F("Publishing: [%s] to [%s]"), buffer, mqtt->getClientID());
      mqtt->publish(mqtt->getClientID(), buffer);

      // on the cs side all is set and we declare that the cs is open for business
      clients[subscriberid].open = true;

      // we now need to subscribe to the ../clientid/topicid/cmd topic as we shall recieve the cmds from there
      // in the < case we should test that we got the command on the right topic ...

      DIAG(F("MQTT CS is listening for commands on [%s]"), tbuffer);
      memset(buffer, 0, 30);
      sprintf(buffer, "%s/%ld/result", mqtt->getClientID(), topicid);
      DIAG(F("MQTT CS is publishing return information to [%s]"), buffer);
      memset(buffer, 0, 30);
      sprintf(buffer, "%s/%ld/diag", mqtt->getClientID(), topicid);
      DIAG(F("MQTT CS is publishing diagnostic information to [%s]"), buffer);

      return;
    }
    default:
    {
      // ignore
      return;
    }
    }
  }
  default:
  {
    // invalid command
    DIAG(F("MQTT Invalid DCC-EX command: %s"), (char *)payload);
    break;
  }
  }
}

/**
 * @brief MQTT broker connection / reconnection
 * 
 */
void DccMQTT::connect()
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
  while (!mqttClient.connected() && reconnectCount < MAXRECONNECT)
  {
    DIAG(F("Attempting MQTT Broker connection[%d]..."), broker->cType);
    switch (broker->cType)
    {
    // no uid no pwd
    case 6:
    case 1:
    { // port(p), ip(i), domain(d),
      if (mqttClient.connect(connectID))
      {
        DIAG(F("MQTT Broker connected ..."));
        mqState = CONNECTED;
      }
      else
      {
        DIAG(F("MQTT broker connection failed, rc=%d, trying to reconnect"), mqttClient.state());
        reconnectCount++;
      }
      break;
    }
    // with uid passwd
    case 5:
    case 2:
    { // port(p), ip(i), domain(d), user(uid), pwd(pass),
      break;
    }
    // with uid, passwd & prefix
    case 4:
    case 3:
    { // port(p), ip(i), domain(d), user(uid), pwd(pass), prefix(pfix)
      // port(p), domain(d), user(uid), pwd(pass), prefix(pfix)
      // mqttClient.connect(connectID, MQTT_BROKER_USER, MQTT_BROKER_PASSWD, "$connected", 0, true, "0", 0))
      break;
    }
    }
    if (reconnectCount == MAXRECONNECT)
    {
      DIAG(F("MQTT Connection aborted after %d tries"), MAXRECONNECT);
      mqState = CONNECTION_FAILED;
    }
  }
}

// for the time being only one topic at the root which os the unique clientID from the MCU
// QoS is 0 by default
boolean DccMQTT::subscribe(char *topic)
{
  return mqttClient.subscribe(topic);
}

void DccMQTT::publish(char *topic, char *payload)
{
  mqttClient.publish(topic, payload);
}

/**
 * @brief Public part of the MQTT setup function. Will call the secondary private setup function following the broker 
 * configuration from config.h
 * 
 */
void DccMQTT::setup()
{
  // setup Ethnet connection first
  byte mac[6];
  DCCTimer::getSimulatedMacAddress(mac);

#ifdef IP_ADDRESS
  Ethernet.begin(mac, IP_ADDRESS);
#else
  if (Ethernet.begin(mac) == 0)
  {
    DIAG(F("Ethernet.begin FAILED"));
    return;
  }
#endif
  DIAG(F("Ethernet.begin OK."));
  if (Ethernet.hardwareStatus() == EthernetNoHardware)
  {
    DIAG(F("Ethernet shield not found"));
    return;
  }
  if (Ethernet.linkStatus() == LinkOFF)
  {
    DIAG(F("Ethernet cable not connected"));
    return;
  }

  IPAddress ip = Ethernet.localIP(); // reassign the obtained ip address

  DIAG(F("IP: %d.%d.%d.%d"), ip[0], ip[1], ip[2], ip[3]);
  DIAG(F("Port:%d"), IP_PORT);

  // setup the MQBroker
  setup(CSMQTTBROKER);
}

/**
 * @brief Private part of the MQTT setup function. Realizes all required actions for establishing the MQTT connection.
 * 
 * @param id Name provided to the broker configuration
 * @param b MQTT broker object containing the main configuration parameters
 */
void DccMQTT::setup(const FSH *id, MQTTBroker *b)
{

  //Create the MQTT environment and establish inital connection to the Broker
  broker = b;

  DIAG(F("MQTT Connect to %S at %S/%d.%d.%d.%d:%d"), id, broker->domain, broker->ip[0], broker->ip[1], broker->ip[2], broker->ip[3], broker->port);


  byte mqbid[CLIENTIDSIZE] = {0};
  DCCTimer::getSimulatedMacAddress(mqbid);

  // initalize MQ Broker

  mqttClient = PubSubClient(broker->ip, broker->port, mqttCallback, ethClient);
  DIAG(F("MQTT Client created ok..."));

  array_to_string(mqbid, CLIENTIDSIZE, clientID);
  DIAG(F("MQTT Client ID : %s"), clientID);

  connect();                               // inital connection as well as reconnects
  auto sub = DccMQTT::subscribe(clientID); // set up all subscriptions

  DIAG(F("MQTT subscriptons %s..."), sub ? "ok" : "failed");
  outboundRing = new RingStream(OUT_BOUND_SIZE);
}

void DccMQTT::loop()
{
  // Connection impossible so just don't do anything
  if (mqState == CONNECTION_FAILED)
  {
    return;
  }
  if (!mqttClient.connected())
  {
    connect();
  }
  if (!mqttClient.loop())
  {
    DIAG(F("mqttClient returned with error; state: %d"), mqttClient.state());
  };

  // read incomming queue for processing; one per loop
  bool state;
  DIAG(F("in.count: %d"), in.count());
  if (in.count() > 0)
  {
    auto idx = in.peek();
    auto c = pool.getItem(in.pop(), &state);
    DIAG(F("MQTT Processing pool: %d with command: %s from client %d"), idx, c->cmd, c->mqsocket);
    DIAG(F("Ring free space1: %d"), outboundRing->freeSpace());
    outboundRing->mark((uint8_t)c->mqsocket);
    CommandDistributor::parse(c->mqsocket, (byte *)c->cmd, outboundRing);
    // StringFormatter::send(outboundRing, F("Test result message"));
    outboundRing->commit();
    DIAG(F("Ring free space2: %d"), outboundRing->freeSpace());
    pool.returnItem(idx);
  }

  // handle at most 1 outbound transmission
  int socketOut = outboundRing->read();
  DIAG(F("socketOut: %d"), socketOut);
  if (socketOut > 0) // mqsocket / clientid can't be 0 ....
  {
    int count = outboundRing->count();
    buffer[0] = '\0';
    sprintf(buffer, "%s/%d/result", clientID, (int)clients[socketOut].topic);
    DIAG(F("MQTT publish to mqsocket=%d, count=:%d on topic %s"), socketOut, count, buffer);
    // construct the payload
    char payload[count];
    payload[count] = '\0';
    char *tmp = payload;
    for (; count > 0; count--)
    {
      *tmp = (char)outboundRing->read();
      tmp++;
    }
    // DIAG(F("Ring free space4: %d"),outboundRing->freeSpace());
    DIAG(F("MQTT publish with payload:\n%s"), payload);
    // mqtt->publish(buffer, payload);
  }
}