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
 */

#if __has_include ( "config.h")
  #include "config.h"
#else
  #warning config.h not found. Using defaults from config.example.h 
  #include "config.example.h"
#endif
#include "defines.h" 

#include <Arduino.h>
#include <EthernetInterface.h>
#include <PubSubClient.h> // Base (sync) MQTT library

#include <DccMQTT.h>

void DccMQTT::setup()
{
    
  // IPAddress server(MQTT_BROKER_ADDRESS);
// EthernetClient ethClient = ETHNetwork::getServer().available();

// // MQTT connection
// PubSubClient mqttClient(ethClient);
// PubSubClient *DccMQTT::mqClient = &mqttClient;

  server = IPAddress(MQTT_BROKER_ADDRESS);
  
  // char _csidMsg[64]{'\0'}; //!< string buffer for the serialized message to return
  // mqttClient.setServer(server, MQTT_BROKER_PORT); // Initalize MQ broker


  // DBG(F("MQTT Client : Server ok ..."));
  // mqttClient.setCallback(mqttCallback); // Initalize callback function for incomming messages
  // DBG(F("MQTT Client : Callback set ..."));

  // DccMQTT::setDeviceID();                     // set the unique device ID to bu used for creating / listening to topic

  // /**
  //  * @todo check for connection failure
  //  */
  // reconnect();                                          // inital connection as well as reconnects
  // DccMQTT::subscribe();                                 // set up all subscriptionn
  // INFO(F("MQTT subscriptons done..."));
  // sprintf_P(_csidMsg, csidfmt, DccMQTT::getDeviceID());
  // mqttClient.publish(DccMQTT::topics[ADMIN], _csidMsg); // say hello to the broker and the API who listens to this topic
  
  
  // /**
  //  * @todo set the connect status with a retained message on the $connected topic /admin/<csid>/$connected as used in the connect
  //  * 
  //  */

  // mqttDccExParser = p;
}


// #include <avr/pgmspace.h> // for PROGMEM use

// #include <Diag/DIAG.h>                   // Diagnostig output to the serial terminal
// #include <Diag/Telemetry/DccTelemetry.h> // Diagnostic/metrics output to MQTT
// #include <ETHNetwork/ETHNetwork.h>       // Ethernet setup; todo: abstract this away into a network interface so that we can use WiFi or Ethernet
// #include <Ethernet.h>                    // Std Ethernet library

// #include <ArduinoUniqueID.h>
// #include <Transport/MQTT/DccMQTT.h>           // MQTT Message controller
// #include <Transport/MQTT/DccMQTTProc.h>       // MQTT Message processor
// #include <Transport/MQTT/DccMQTTCommandMsg.h> // MQTT Message model

// #include <DCCEXParser.h> // DCC++-EX Parser;; used for pushing JRMI commands directly recieved

// //---------
// // Defines
// //---------

// #define MAXTBUF 50  //!< max length of the buffer for building the topic name ;to be checked
// #define MAXTMSG 120 //!< max length of the messages for a topic               ;to be checked PROGMEM ?
// #define MAXTSTR 30  //!< max length of a topic string

// //---------
// // Variables
// //---------

// char topicName[MAXTBUF];
// char topicMessage[MAXTMSG];
// char keyword[MAX_KEYWORD_LENGTH];

// void mqttCallback(char *topic, byte *payload, unsigned int length);
// void dccmqttCommandHandler(char *topicName, char *topicMessage);
// bool setMsgParams(int mdix, Parameters p, uint16_t v);
// bool setMsgParamsByObj(int mdix, Parameters p, JsonObject v, bool mandatory);
// bool validateParam(Parameters p, uint16_t v);
// char *getPGMkeyword(const char *keyword);

// // Ethernet connection to the MQTT server
// IPAddress server(MQTT_BROKER_ADDRESS);
// EthernetClient ethClient = ETHNetwork::getServer().available();

// // MQTT connection
// PubSubClient mqttClient(ethClient);
// PubSubClient *DccMQTT::mqClient = &mqttClient;

// char _deviceId[MAXDEVICEID];                    //!< string holding the device id
// char *DccMQTT::deviceID = _deviceId;            //!< assign device ID to the class member
// char **DccMQTT::topics = new char *[MAXTOPICS]; //!< array of pointesr to the topic

// // Queue setup
// Queue DccMQTT::inComming(sizeof(DccMQTTCommandMsg *), MAXQUEUE, FIFO); //!< incomming messages : struct after preprocessing. type to be defined
// Queue DccMQTT::outGoing(sizeof(DccMQTTCommandMsg *), MAXQUEUE, FIFO);  //!< outgoing messages; string in JSON format

// // JSON buffer
// StaticJsonDocument<MAXTMSG> doc;

// // index into the message pool
// uint8_t midx;

// DCCEXParser mqttDccExParser;

// //---------
// // Functions
// //---------

// /**
//  * @brief Copies an array to a string; used for generating the unique Arduino ID
//  * 
//  * @param array array containing bytes
//  * @param len length of the array
//  * @param buffer buffer to which the string will be written; make sure the buffer has appropriate length
//  */
// static void array_to_string(byte array[], unsigned int len, char buffer[])
// {
//   for (unsigned int i = 0; i < len; i++)
//   {
//     byte nib1 = (array[i] >> 4) & 0x0F;
//     byte nib2 = (array[i] >> 0) & 0x0F;
//     buffer[i * 2 + 0] = nib1 < 0xA ? '0' + nib1 : 'A' + nib1 - 0xA;
//     buffer[i * 2 + 1] = nib2 < 0xA ? '0' + nib2 : 'A' + nib2 - 0xA;
//   }
//   buffer[len * 2] = '\0';
// }

// /**
//  * @brief Maps a command recieved from the JSON string to the corresponding enum value
//  * 
//  * @param c  string containing the command
//  * @return Commands enum value from the Command enum
//  * @throw Returns an INVALID_C - invalid command for unknown Commands send
//  * @see Commands
//  */
// Commands resolveCommand(const char *c)
// {
//   DBG(F("Resolving Command: %s"), c);
//   if (strcmp_P(c, _kRead) == 0)
//     return READ;
//   if (strcmp_P(c, _kWrite) == 0)
//     return WRITE;
//   if (strcmp_P(c, _kPower) == 0)
//     return POWER;
//   if (strcmp_P(c, _kThrottle) == 0)
//     return THROTTLE;
//   if (strcmp_P(c, _kFunction) == 0)
//     return FUNCTION;
//   return INVALID_C;
// };

// DccTopics resolveTopics(const char *c)
// {
//   int i = 0;
//   while ((strcmp(c, DccMQTT::topics[i]) != 0))
//   {
//     i++;
//     if (i > NOOFDCCTOPICS)
//     {
//       return INVALID_T;
//     }
//   }
//   return (DccTopics)i;
// }

// /**
//  * @brief Maps parameters names recieved from the JSON string to the corresponding enum value
//  * 
//  * @param c  string containing the parameter name
//  * @return parameters enum value from the Parameter enum
//  */
// Parameters resolveParameters(char *c)
// {
//   DBG(F("Resolving parameter: %s"), c);
//   if (strcmp_P(c, _kCv) == 0)
//     return CV;
//   if (strcmp_P(c, _kValue) == 0)
//     return VALUE;
//   if (strcmp_P(c, _kLocomotive) == 0)
//     return LCOCMOTIVE;
//   if (strcmp_P(c, _kSpeed) == 0)
//     return SPEED;
//   if (strcmp_P(c, _kDirection) == 0)
//     return DIRECTION;
//   if (strcmp_P(c, _kFn) == 0)
//     return FN;
//   if (strcmp_P(c, _kState) == 0)
//     return STATE;
//   if (strcmp_P(c, _kTrack) == 0)
//     return TRACK;
//   if (strcmp_P(c, _kBit) == 0)
//     return BIT;
//   return INVALID_P;
// };

// /**
//  * @brief Maps Parameters enum values to the corresponding string
//  * 
//  * @param p : Enum value of the Parameter
//  * @return const char* 
//  */
// const char *resolveParameters(Parameters p)
// {
//   switch (p)
//   {
//   case CV:
//     return _kCv;
//   case VALUE:
//     return _kValue;
//   case LCOCMOTIVE:
//     return _kLocomotive;
//   case SPEED:
//     return _kSpeed;
//   case DIRECTION:
//     return _kDirection;
//   case FN:
//     return _kFn;
//   case STATE:
//     return _kState;
//   case TRACK:
//     return _kTrack;
//   case BIT:
//     return _kBit;
//   case INVALID_P:
//     return NULL;
//   }
//   return NULL;
// }

// /**
//  * @brief Callback executed upon reception of a message by the PubSubClient
//  * 
//  * @param topic   topic string
//  * @param payload serialized content of the message
//  * @param length  length of the recieved message
//  * 
//  */
// void mqttCallback(char *topic, byte *payload, unsigned int length)
// {
//   topicName[0] = '\0';
//   topicMessage[0] = '\0';
//   strcpy(topicName, topic);
//   strlcpy(topicMessage, (char *)payload, length + 1);

//   DccTopics t = resolveTopics(topicName);

//   switch (t)
//   {
//   case CMD_L:
//   case CMD_T:
//   case CMD_S:
//   case CMD_A:
//   {
//     INFO(F("MQTT Message arrived [%s]: %s"), topicName, topicMessage);
//     dccmqttCommandHandler(topicName, topicMessage); // /command topic recieved
//     break;
//   }
//   case JRMI:
//   {
//     INFO(F("JRMI Message arrived [%s]: %s"), topicName, topicMessage);
//     // mqttDccExParser.parse(Serial, (const byte *)topicMessage, 0); // send the message to the DCC parser for handling and return;
//     mqttDccExParser.parse(&Serial, (byte *)topicMessage, 0); // send the message to the DCC parser for handling and return;
//     return;
//   }
//   case ADMIN: {
//     INFO(F("Admin Message arrived [%s]: %s"), topicName, topicMessage);
//     return;
//   }
//   case TELEMETRY:
//   case RESULT:
//   case DIAGNOSTIC:
//   case INVALID_T:
//   {
//     // don't do anything for those
//     return;
//   }
//   }
// }

// /**
//  * @brief Central nervous system. All messages are passed through here parse and a pool item gets build. Syntax and Semantic checks are done here on all commands parameters etc ... 
//  * All error get printed to Serial for now but will be send over MQ as well in the future for further processing by the client
//  * 
//  * @param topicName : Topic on which the message has been recieved
//  * @param topicMessage : Message that has been recieved
//  * 
//  */
// void dccmqttCommandHandler(char *topicName, char *topicMessage)
// {

//   DeserializationError err = deserializeJson(doc, topicMessage);

//   if (err)
//   {
//     ERR(F("deserializeJson() failed: %s"), err.c_str());
//     ERR(F("Message ignored ..."));
//     return;
//   }

//   JsonObject p;
//   if (doc.containsKey("p"))
//   {
//     p = doc["p"]; // parameters recieved - present in any command recieved the payload may differ though
//   }
//   else
//   {
//     ERR(F("No parameters provided in Message"));
//     return;
//   }

//   midx = DccMQTTCommandMsg::setMsg(resolveCommand(doc["c"]), doc["cid"]);
//   DccMQTTCommandMsg::msg[midx].nParams = 0;

//   DBG(F("Recieved command: [%d] : cid [%s]"), DccMQTTCommandMsg::msg[midx].cmd, DccMQTTCommandMsg::msg[midx]._cid);

//   switch (DccMQTTCommandMsg::msg[midx].cmd)
//   {
//   case READ:
//   {
//     // Mandatory parameters
//     if (!setMsgParamsByObj(midx, CV, p, true)) // reqd parameter not available; error printing done in setMsgParams
//     {
//       return;
//     }
//     DccMQTTCommandMsg::msg[midx].nParams++;
//     DBG(F("Read: recieved parameter cv: [%d]"), DccMQTTCommandMsg::msg[midx].params[CV]);

//     // Optional parameters
//     if (setMsgParamsByObj(midx, BIT, p, false)) // found correct bit parameter if true
//     {
//       DccMQTTCommandMsg::msg[midx].nParams++;
//       DBG(F("Read: recieved optional parameter bit: [%d]"), DccMQTTCommandMsg::msg[midx].params[BIT]);
//     }
//     break;
//   }
//   case WRITE:
//   {
//     // Mandatory parameters
//     if (!setMsgParamsByObj(midx, CV, p, true))
//       return;
//     if (!setMsgParamsByObj(midx, VALUE, p, true))
//       return;

//     DccMQTTCommandMsg::msg[midx].nParams = 2;

//     // Optional parameters
//     // If loco is present we need the track to be main
//     // Writing on Prog doesn't need a loco to be present as there shall be only one loco anyway
//     if (setMsgParamsByObj(midx, LCOCMOTIVE, p, false))
//     {
//       DccMQTTCommandMsg::msg[midx].nParams++;
//       // verify that track is M
//       if (setMsgParamsByObj(midx, TRACK, p, false))
//       {
//         if (DccMQTTCommandMsg::msg[midx].params[TRACK] == MAIN)
//         {
//           DccMQTTCommandMsg::msg[midx].nParams++;
//         }
//         else
//         {
//           ERR(F("Track has to be main as Locomotive has been provided "));
//           return;
//         }
//       }
//     }

//     if (setMsgParamsByObj(midx, BIT, p, false)) // found correct bit parameter if true
//     {
//       // verify if value is 0 or 1
//       uint16_t v = DccMQTTCommandMsg::msg[midx].params[VALUE];
//       if (v == 0 || v == 1)
//       {
//         DccMQTTCommandMsg::msg[midx].nParams++;
//       }
//       else
//       {
//         ERR(F("Value has to be 0 or 1 as bit to be written has been set"));
//         return;
//       }
//     }
//     break;
//   }
//   case POWER:
//   {
//     // Mandatory parameters
//     if (!setMsgParamsByObj(midx, TRACK, p, true)) // reqd parameter track not available; error printing done in setMsgParams
//     {
//       return;
//     };
//     DccMQTTCommandMsg::msg[midx].nParams++;
//     if (!setMsgParamsByObj(midx, STATE, p, true)) // reqd parameter state not available; error printing done in setMsgParams
//     {
//       return;
//     }
//     DccMQTTCommandMsg::msg[midx].nParams++;
//     DBG(F("Power: recieved parameter track: %d state: %d "), DccMQTTCommandMsg::msg[midx].params[TRACK], DccMQTTCommandMsg::msg[midx].params[STATE]);
//     break;
//   }
//   case THROTTLE:
//   {
//     // Mandatory parameters
//     if (!setMsgParamsByObj(midx, LCOCMOTIVE, p, true))
//       return;
//     if (!setMsgParamsByObj(midx, SPEED, p, true))
//       return;
//     if (!setMsgParamsByObj(midx, DIRECTION, p, true))
//       return;
//     DccMQTTCommandMsg::msg[midx].nParams = 3;
//     DBG(F("Throttle: recieved parameter locomotive: %d speed: %d direction: %d "), DccMQTTCommandMsg::msg[midx].params[LCOCMOTIVE], DccMQTTCommandMsg::msg[midx].params[SPEED], DccMQTTCommandMsg::msg[midx].params[DIRECTION]);
//     break;
//   }
//   case FUNCTION:
//   {
//     // Mandatory parameters
//     if (!setMsgParamsByObj(midx, LCOCMOTIVE, p, true))
//       return;
//     if (!setMsgParamsByObj(midx, FN, p, true))
//       return;
//     if (!setMsgParamsByObj(midx, STATE, p, true))
//       return;
//     DccMQTTCommandMsg::msg[midx].nParams = 3;
//     break;
//   }
//   case INVALID_C:
//   default:
//   {
//     ERR(F("Invalid command recieved"));
//     return;
//   }
//   }
//   // Note: Do not use the client in the callback to publish, subscribe or
//   // unsubscribe as it may cause deadlocks when other things arrive while
//   // sending and receiving acknowledgments. Instead, change a global variable,
//   // or push to a queue and handle it in the loop after calling `DccMQTT::loop()`

//   // enqueue the cmdMsg pool item used for passing the commands
//   DccMQTT::pushIn(midx);
// }

// /**
//  * @brief Set the parameters in the pool item to be send to the processor
//  * 
//  * @param mdix : Index of the pool item
//  * @param p : Parameter to be set
//  * @param v : Json object of the parameter value recieved
//  * @param mandatory : if the parameter is mandatory
//  * @return true 
//  * @return false  the value is 0 return false
//  */
// bool setMsgParamsByObj(int mdix, Parameters p, JsonObject v, bool mandatory)
// {
//   // This does make two calls deserialzing the parameter; may possibly be optimized
//   bool success = true;
//   char *kw = getPGMkeyword(resolveParameters(p));
//   if (v.containsKey(kw) == false)
//   {
//     if (mandatory)
//     {
//       ERR(F("Wrong parameter provided: [%s]"), kw);
//       return false;
//     }
//     else
//     {
//       INFO(F("Ignoring optional parameter: [%s]"), kw);
//       return false;
//     }
//   }
//   switch (p)
//   {
//   case BIT:
//   {
//     DccMQTTCommandMsg::msg[midx].params[p] = v[kw];
//     success = validateParam(BIT, v[kw]);
//     break;
//   }
//   case TRACK:
//   {
//     char tr[2] = {0};
//     strncpy(tr, v[kw], 2);
//     DccMQTTCommandMsg::msg[midx].params[p] = tr[0];
//     success = validateParam(TRACK, tr[0]);
//     break;
//   }
//   case DIRECTION:
//   {
//     char tr[2] = {0};
//     strncpy(tr, v[kw], 2);
//     success = validateParam(DIRECTION, tr[0]);
//     if (tr[0] == 'F') {
//       DccMQTTCommandMsg::msg[midx].params[p] = 1;
//     } else {
//       DccMQTTCommandMsg::msg[midx].params[p] = 0;
//     }
//     break;
//   }
//   case STATE:
//   {
//     char st[4] = {0};
//     strncpy(st, v[kw], 3);
//     /**
//      * @todo validate ON/OFF keywords here so that we don't handle any garbage comming in 
//      * 
//      */
//     DBG(F("State keyword [%s] [%s]"), kw, st); 
//     if (st[1] == 'N') {
//       DccMQTTCommandMsg::msg[midx].params[p] = 1;
//     } else {
//       DccMQTTCommandMsg::msg[midx].params[p] = 0;
//     }
//     // DccMQTTCommandMsg::msg[midx].params[p] = v.getMember(kw);
//     break;
//   }
//   case CV:
//   case VALUE:
//   case LCOCMOTIVE:
//   case SPEED:
//   case FN:
//   {
//     DccMQTTCommandMsg::msg[midx].params[p] = v.getMember(kw);
//     break;
//   }
//   case INVALID_P:
//   {
//     success = false;
//     break;
//   }
//   }
//   DBG(F("Set Parameter [%s] to [%d] [%s]"), kw, DccMQTTCommandMsg::msg[midx].params[p], success ? "OK" : "FAILED");
//   return success;
// };

// /**
//  * @brief Validates permitted values for Parameters
//  * 
//  * @param p : Parameter
//  * @param v : value to validate for the given Parameter
//  * @return true : if value is allowed
//  * @return false : false if value is not allowed
//  */
// bool validateParam(Parameters p, uint16_t v)
// {

//   bool valid;

//   switch (p)
//   {
//   case CV:
//   case VALUE:
//   case LCOCMOTIVE:
//   case SPEED:
//   case FN:
//   case STATE:
//   case INVALID_P:
//   {
//     valid = true;
//     break;
//   }
//   case DIRECTION:
//   {
//     if (v == 'F' || v == 'R')
//     {
//       valid = true;
//     }
//     else
//     {
//       ERR(F("Wrong value for direction provided (F or R)"));
//       valid = false;
//     }
//     break;
//   }
//   case TRACK:
//   {
//     if (v == MAIN || v == ALL || v == PROG)
//     {
//       valid = true;
//     }
//     else
//     {
//       ERR(F("Wrong value for track provided (M or A or P)"));
//       valid = false;
//     }
//     break;
//   }
//   case BIT:
//   {
//     if (v >= 0 && v <= 7)
//     {
//       valid = true;
//     }
//     else
//     {
//       ERR(F("Wrong parameter value provided for bit (>= 0 and <= 7)"));
//       valid = false;
//     }
//     break;
//   }
//   }
//   DBG(F("Validated parameter:[%s] Value:[%d] [%s]"), getPGMkeyword(resolveParameters(p)), v, valid ? "OK" : "FAILED");
//   return valid;
// }

// /**
//  * @brief Retrieves a keyword from PROGMEM
//  * 
//  * @param k keyword to retrieve
//  * @return char* string copied into SRAM
//  */
// char *getPGMkeyword(const char *k)
// {
//   strncpy_P(keyword, k, MAX_KEYWORD_LENGTH);
//   return keyword;
// };

// /**
//  * @brief Pushes a message into the the in comming queue; locks the pool item
//  * 
//  * @param midx the index of the message in the pool to be pushed into the queue
//  */
// void DccMQTT::pushIn(uint8_t midx)
// {
//   static uint32_t msgInCnt = 101;
//   if (!DccMQTT::inComming.isFull())
//   {
//     DccMQTTCommandMsg::msg[midx].msgId = msgInCnt;
//     DccMQTTCommandMsg::msg[midx].free = false;
//     DccMQTTCommandMsg::msg[midx].mIdx = midx;
//     DccMQTT::inComming.push(&DccMQTTCommandMsg::msg[midx].mIdx);
//     DBG(F("Enqueued incomming msg[%d] #%d"), midx, DccMQTTCommandMsg::msg[midx].msgId);
//     // DccMQTTCommandMsg::printCmdMsg(&DccMQTTCommandMsg::msg[midx]);

//     msgInCnt++;
//   }
//   else
//   {
//     ERR(F("Dcc incomming message queue is full; Message ignored"));
//   }
// }

// /**
//  * @brief Pushes a message into the the outgoing queue; locks the pool item;
//  * 
//  * @param midx the index of the message in the pool to be pushed into the queue
//  */
// void DccMQTT::pushOut(uint8_t midx)
// {
//   static uint32_t msgOutCnt = 501;
//   if (!DccMQTT::outGoing.isFull())
//   {
//     DccMQTTCommandMsg::msg[midx].msgId = msgOutCnt;
//     DccMQTTCommandMsg::msg[midx].free = false;
//     DccMQTTCommandMsg::msg[midx].mIdx = midx;
//     DccMQTT::outGoing.push(&DccMQTTCommandMsg::msg[midx].mIdx);
//     DBG(F("Enqueued outgoing msg #%d"), DccMQTTCommandMsg::msg[midx].msgId);
//     msgOutCnt++;
//   }
//   else
//   {
//     ERR(F("Dcc outgoing message queue is full; Message ignored"));
//   }
// }

// /**
//  * @brief pops a message from the the in comming queue; The pool item used is still in use and stays locked
//  * 
//  * @param midx the index of the message in the pool to be poped
//  * @return index in the message pool of the message which has been poped; -1 if the queue is empty
//  */
// uint8_t DccMQTT::popIn()
// {
//   uint8_t i;
//   if (!DccMQTT::inComming.isEmpty())
//   {
//     DccMQTT::inComming.pop(&i);
//     DccMQTTCommandMsg::msg[i].free = false;
//     DBG(F("Dequeued incomming msg #%d, cid: %s"), DccMQTTCommandMsg::msg[i].msgId, DccMQTTCommandMsg::msg[i]._cid);
//     return i;
//   }
//   return -1;
// }
// /**
//  * @brief pops a message from the the outgoing queue; The pool item used is freed and can be reused
//  * 
//  * @param midx the index of the message in the pool to be poped
//  * @return index in the message pool of the message which has been poped; -1 if the queue is empty
//  */
// uint8_t DccMQTT::popOut()
// {
//   uint8_t i;
//   if (!DccMQTT::outGoing.isEmpty())
//   {
//     DccMQTT::outGoing.pop(&i);
//     DccMQTTCommandMsg::msg[i].free = true;
//     DBG(F("Dequeued outgoing msg #%d, cid: %s"), DccMQTTCommandMsg::msg[i].msgId, DccMQTTCommandMsg::msg[i]._cid);
//     // DccMQTTCommandMsg::printCmdMsg(&DccMQTTCommandMsg::msg[i]);
//     // DccMQTTCommandMsg::printCmdMsgPool();
//     return i;
//   }
//   return -1;
// }

// /**
//  * @brief in case we lost the connection to the MQTT broker try to restablish a conection
//  * 
//  */
// static void reconnect()
// {
//   DBG(F("MQTT (re)connecting ..."));

//   while (!mqttClient.connected())
//   {
//     INFO(F("Attempting MQTT Broker connection..."));
//     // Attempt to connect
// #ifdef CLOUDBROKER
//     char *connectID = new char[40];
    
//     connectID[0] = '\0';
//     strcat(connectID, MQTT_BROKER_CLIENTID_PREFIX);
//     strcat(connectID,DccMQTT::getDeviceID());

//     INFO(F("ConnectID: %s %s %s"), connectID, MQTT_BROKER_USER, MQTT_BROKER_PASSWD);
//     #ifdef MQTT_BROKER_USER
//     DBG(F("MQTT (re)connecting (Cloud/User) ..."));
//     if (mqttClient.connect(connectID, MQTT_BROKER_USER, MQTT_BROKER_PASSWD, "$connected", 0, true, "0", 0))
//     #else
//     DBG(F("MQTT (re)connecting (Cloud) ..."));
//     if (mqttClient.connect(DccMQTT::getDeviceID()))
//     #endif
// #else
//     #ifdef MQTT_BROKER_USER
//     DBG(F("MQTT (re)connecting (Local/User) ..."));
//     if (mqttClient.connect(DccMQTT::getDeviceID(), MQTT_BROKER_USER, MQTT_BROKER_PASSWD))
//     #else
//     DBG(F("MQTT (re)connecting (Local) ..."));
//     if (mqttClient.connect(DccMQTT::getDeviceID()))
//     #endif
// #endif
//     {
//       INFO(F("MQTT broker connected ..."));
//       // publish on the  $connected topic 
//       DccMQTT::subscribe(); // required in case of a connection loss to do it again (this causes a mem leak !! of 200bytes each time!!)
//     }
//     else
//     {
//       INFO(F("MQTT broker connection failed, rc=%d, trying to reconnect"), mqttClient.state());
//     }
//   }
// }

// /**
//  * @brief Test if the mqtt client is connected
//  * 
//  * @return true - connected
//  * @return false - not connected
//  */
// bool DccMQTT::connected()
// {
//   return mqttClient.connected();
// }

// /**
//  * @brief builds the topic strings for the intents together with the unique ID and the Ressource letter
//  * 
//  * @param c char for specific chnalle under the intent; if '_' it will not be used
//  * @param t index for the topic in the list of topics 
//  * @return char* of the topic string build
//  */
// static char *buildTopicID(char c, DccTopics t)
// {
//   char *str = new char[MAXTSTR];
//   str[0] = '\0'; // flush the string
//   switch (t)
//   {
//   case RESULT:
//   {
//     strcat(str, TCMRESROOT); // results channel
//     strcat(str, DccMQTT::getDeviceID());
//     break;
//   }
//   case ADMIN:
//   {
//     strcat(str, ADMROOT); // admin
//     strcat(str, DccMQTT::getDeviceID());
//     break;
//   }
//   case TELEMETRY:
//   {
//     strcat(str, TELEMETRYROOT); // telemetry
//     strcat(str, DccMQTT::getDeviceID());
//     break;
//   }
//   case DIAGNOSTIC:
//   {
//     strcat(str, DIAGROOT); // diganostics
//     strcat(str, DccMQTT::getDeviceID());
//     break;
//   }
//   case JRMI:
//   {
//     strcat(str, JRMIROOT); // for JRMI formmated input from the JRMI topic
//     strcat(str, DccMQTT::getDeviceID());
//     break;
//   }
//   case CMD_L:
//   case CMD_A:
//   case CMD_T:
//   case CMD_S:
//   {
//     strcat(str, TCMDROOT);
//     strcat(str, DccMQTT::getDeviceID());
//     strncat(str, "/", 1);
//     strncat(str, &c, 1);
//     break;
//   }
//   case INVALID_T:
//     return NULL;
//   }

//   DccMQTT::topics[t] = str;
//   INFO(F("Topic: %s created"), DccMQTT::topics[t]);
//   return DccMQTT::topics[t];
// }

// void DccMQTT::printTopics()
// {
//   INFO(F("List of subcribed Topics ... "));
//   for (int i = 0; i < MAXTOPICS; i++)
//   {
//     INFO(F("Topic: %s"), DccMQTT::topics[i]);
//   }
// }

// void DccMQTT::subscribe()
// {
//   static bool subscribed = false;
//   if (subscribed == false) // Subscribe for the first time
//   {
//     mqttClient.subscribe(buildTopicID('L', CMD_L));      // command/<CSID>/L - Subscription CSID is 18 in length
//     mqttClient.subscribe(buildTopicID('T', CMD_T));      // command/<CSID>/T - Subscription only
//     mqttClient.subscribe(buildTopicID('S', CMD_S));      // command/<CSID>/S - Subscription only
//     mqttClient.subscribe(buildTopicID('A', CMD_A));      // command/<CSID>/A - Subscription only
//     mqttClient.subscribe(buildTopicID('_', RESULT));     // command Response - Publish only
//     mqttClient.subscribe(buildTopicID('_', ADMIN));      // admin - Publish / Subscribe
//     mqttClient.subscribe(buildTopicID('_', DIAGNOSTIC)); // diagnostics - Publish / Subscribe
//     mqttClient.subscribe(buildTopicID('_', TELEMETRY));  // metrics - Publish / Subscribe
//     mqttClient.subscribe(buildTopicID('_', JRMI));       // metrics - Publish / Subscribe
//     subscribed = true;
//   }
//   else // already subscribed once so we have the topics stored no need to rebuild them
//   {
//     for (int i = 0; i < MAXTOPICS; i++)
//     {
//       mqttClient.subscribe(DccMQTT::topics[i]);
//       // mqttClient.subscribe("diag/memory", 0);  // to be replaced by telemetry
//     }
//   }
// }

// void DccMQTT::subscribeT(char *topic)
// {
//   mqttClient.subscribe(topic);
// }

// #define PUB_RES_FMT "{\"cid\":\"%s\", \"result\":\"%d\"}"
// PROGMEM const char resfmt[] = {PUB_RES_FMT};

// void DccMQTT::publish()
// {
//   char _memMsg[64]{'\0'}; //!< string buffer for the serialized message to return
//   if (!DccMQTT::outGoing.isEmpty())
//   {
//     uint8_t c = DccMQTT::popOut();
//     if (c != -1)
//     {
//       /**
//        * @todo check for the length of theoverall response and make sure its smalle than the _memMsg buffer size
//        */
//       sprintf_P(_memMsg, resfmt, DccMQTTCommandMsg::msg[c]._cid, DccMQTTCommandMsg::msg[c].result);

//       INFO(F("Sending result: %s length: %d"), _memMsg, strlen(_memMsg));
//       mqttClient.publish(DccMQTT::topics[RESULT], _memMsg);
//     };
//   };
// }
// /**
//  * @brief Initalizes the MQTT broker connection; subcribes to all reqd topics and sends the deviceID to the broker on the /admin channel
//  * 
//  */
// #define PUB_CSID_FMT "{\"csid\":\"%s\"}"
// PROGMEM const char csidfmt[] = {PUB_CSID_FMT};


// void DccMQTT::setup(DCCEXParser p)
// {
//   char _csidMsg[64]{'\0'}; //!< string buffer for the serialized message to return
//   mqttClient.setServer(server, MQTT_BROKER_PORT); // Initalize MQ broker
//   DBG(F("MQTT Client : Server ok ..."));
//   mqttClient.setCallback(mqttCallback); // Initalize callback function for incomming messages
//   DBG(F("MQTT Client : Callback set ..."));

//   DccMQTT::setDeviceID();                     // set the unique device ID to bu used for creating / listening to topic

//   /**
//    * @todo check for connection failure
//    */
//   reconnect();                                          // inital connection as well as reconnects
//   DccMQTT::subscribe();                                 // set up all subscriptionn
//   INFO(F("MQTT subscriptons done..."));
//   sprintf_P(_csidMsg, csidfmt, DccMQTT::getDeviceID());
//   mqttClient.publish(DccMQTT::topics[ADMIN], _csidMsg); // say hello to the broker and the API who listens to this topic
  
  
//   /**
//    * @todo set the connect status with a retained message on the $connected topic /admin/<csid>/$connected as used in the connect
//    * 
//    */

//   DccTelemetry::setup();
//   mqttDccExParser = p;
// }

// void DccMQTT::setDeviceID()
// {
//   array_to_string(UniqueID, UniqueIDsize, _deviceId);
//   INFO(F("UniqueID: %s"), _deviceId);
// }

// char *DccMQTT::getDeviceID()
// {
//   return DccMQTT::deviceID;
// };

// void DccMQTT::loop()
// {

//   DccTelemetry::deltaT(1);

//   if (!mqttClient.connected())
//   {
//     reconnect();
//   }
//   if (!mqttClient.loop())
//   {
//     ERR(F("mqttClient returned with error; state: %d"), mqttClient.state());
//   };
//   DccMQTTProc::loop(); //!< give time to the command processor to handle msg ..
//   DccMQTT::publish();  //!< publish waiting messages from the outgoing queue
//   DccTelemetry::deltaT(1);
// }

// bool DccMQTT::inIsEmpty()
// {
//   if (DccMQTT::inComming.isEmpty())
//   {
//     return true;
//   }
//   return false;
// }

// bool DccMQTT::outIsEmpty()
// {
//   if (DccMQTT::outGoing.isEmpty())
//   {
//     return true;
//   }
//   return false;
// };

// DccMQTT::DccMQTT()
// {
//   // long l = random();
// }

// DccMQTT::~DccMQTT()
// {
// }