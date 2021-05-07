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
#include <PubSubClient.h> // Base (sync) MQTT library
#include <DIAG.h>
#include <Ethernet.h>
#include <Dns.h>
#include <DCCTimer.h>
#include <DccMQTT.h>
#include <Queue.h>
#include <ObjectPool.h>
#include <errno.h>
#include <limits.h>
#include <inttypes.h>

//---------
// Variables
//---------

DccMQTT DccMQTT::singleton;
auto mqtt = DccMQTT::get();

// pairing functions used for creating a client identifier
// when a external system connects via MQ to the CS i.e. subscribes to the main channel the first message to be published
// shall be a admin message with a random number
// the cs will based on a counter use a second number to create the cantor encoding of both numbers and publish the cantor code
// this message will be seen by all throttles and they can decode the number which provides the first number they send and the
// second number  to be used as tpoic for the external system from then on. The CS will recieve on all topics the commands and
// during processing then send the replies to the topic from which the command was recieved.
// Thus the main channel shall not be used for any p2p coms ev just for broadcast from the CS to the subscribed clients

int32_t cantorEncode(int a, int b)
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
  errno = 0;
  payload[length] = '\0'; // make sure we have the string terminator in place
  DIAG(F("MQTT Callback:[%s] [%s] [%d]"), topic, (char *)payload, length);

  switch (payload[0])
  {
  case '<':
  {
    // DCC-EX command
    auto pool = mqtt->getPool();
    auto q = mqtt->getIncomming();

    csmsg_t tm;
    strlcpy(tm.cmd, (char *)payload, length + 1);
    // Add the recieved command to the pool
    int idx = pool->setItem(tm);
    if (idx == -1)
    {
      DIAG(F("MQTT Command pool full. Could not handle recieved command."));
      return;
    }
    // Add the index of the pool item to the incomming queue
    q->push(idx);
    DIAG(F("MQTT Message arrived [%s]: [%s]"), topic, tm.cmd);
    break;
  }
  case 'm':
  {
    switch (payload[1])
    {
    case 'i':
    {
      auto clients = mqtt->getClients();

      char buffer[30];
      memset(buffer, 0, 30);
      

      char *tmp = (char *)payload + 3;
      strlcpy(buffer, tmp, length);
      buffer[length - 4] = '\0';

      DIAG(F("MQTT buffer %s - %s - %s - %d"), payload, tmp, buffer, length);

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

      // check in the clients if the distantid has been set already somewhere
      // if so we either have a new one with the same id then we have a collision -> publish a collision
      // or its the same i.e; the message comming back as we are subscribed -> stop here

      // All is ok so set up the channel; MQTT Ctrl command
      auto subscriberid = DccMQTT::get()->obtainSubscriberID(); // to be used in the parsing process for the clientid in the ringbuffer

      if(subscriberid == 0) {
        DIAG(F("MQTT no more connections are available"));
        return;
      }

      auto topicid = cantorEncode(subscriberid, (int)distantid);
      
      DIAG(F("MQTT Ctrl Message arrived [%s] : subscriber [%d] : distant [%d] : topic: [%d]"), buffer, subscriberid, (int)distantid, topicid);
      // extract the number delivered from
      // we need to check if the id we got from the client has been used allready and if yes reject and ask for a different one

      clients[subscriberid] = {(int)distantid, subscriberid, topicid, true}; // add subscribertopic

      char tbuffer[(CLIENTIDSIZE * 2) + 1 + MAXTOPICLENGTH];
      mqtt->getSubscriberTopic(subscriberid, tbuffer);
      auto ok = mqtt->subscribe(tbuffer);
      DIAG(F("MQTT new subscriber topic: %s %s"), tbuffer, ok ? "OK" : "NOK");

      memset(buffer, 0, 30);
      sprintf(buffer, "mc(%d,%ld)", (int)distantid, (long) topicid);
      DIAG(F("Publishing: [%s] to [%s]"), buffer, mqtt->getClientID());
      mqtt->publish(mqtt->getClientID(), buffer);
      clients[subscriberid].open = true;
      // we are done 
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
    payload[length] = '\0';
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

  char *connectID = new char[MAXCONNECTID];
  connectID[0] = '\0';

  int reconnectCount = 0;

  if (broker->prefix != nullptr)
  {
    char tmp[20];
    strcpy_P(tmp, (const char *)broker->prefix);
    connectID[0] = '\0';
    strcat(connectID, tmp);
  }

  strcat(connectID, clientID);

  DIAG(F("MQTT %s (re)connecting ..."), connectID);
  // Build the connect ID : Prefix + clientID

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
  // get a eth client session

  DIAG(F("MQTT Connect to %S at %S/%d.%d.%d.%d:%d"), id, broker->domain, broker->ip[0], broker->ip[1], broker->ip[2], broker->ip[3], broker->port);

  // ethClient = EthernetInterface::get()->getServer()->available();
  ethClient = EthernetInterface::get()->getServer()->accept();
  // initalize MQ Broker

  mqttClient = PubSubClient(ethClient);
  mqttClient.setServer(broker->ip, broker->port);

  DIAG(F("MQTT Client : Server ok ...%x/%x"), ethClient, mqttClient);

  mqttClient.setCallback(mqttCallback); // Initalize callback function for incomming messages

  DIAG(F("MQTT Client : Callback set ..."));

  byte mqbid[CLIENTIDSIZE] = {0};
  DCCTimer::getSimulatedMacAddress(mqbid);

  array_to_string(mqbid, CLIENTIDSIZE, clientID);
  DIAG(F("MQTT Client ID : %s"), clientID);

  connect();                               // inital connection as well as reconnects
  auto sub = DccMQTT::subscribe(clientID); // set up all subscriptions

  DIAG(F("MQTT subscriptons %s..."), sub ? "ok" : "failed");

  // mqttClient.publish(clientID, "Hello from DccEX");
}

void DccMQTT::loop()
{

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
  if (in.count() > 0)
  {
    auto idx = in.peek();
    auto c = pool.getItem(in.pop(), &state);
    DIAG(F("MQTT Processing pool: %d with command: %s"), idx, c->cmd);
  }

  // read outgoing queue for publishing replies; one per loop
  if (out.count() > 0)
  {
    auto m = pool.getItem(out.pop(), &state);
    DIAG(F("MQTT Publish reply from command %s"), m->cmd);
  }

  // DccMQTTProc::loop(); //!< give time to the command processor to handle msg ..
  // take a command from the incomming queue
  // execute it
  // store the results in the outgoing queue

  // DccMQTT::publish();  //!< publish waiting messages from the outgoing queue
  // if there is someting in the outgoing queue publish on response
}