#ifndef _DccMQTT_h_
#define _DccMQTT_h_
/**
 * @file DccMQTT.h
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

// #include <Transport/MQTT/DccMQTTCommandMsg.h>
#include <PubSubClient.h>
#include <DCCEXParser.h>
#include <Queue.h>

#define MAXPAYLOAD 64

struct DccMQTTMsg {
    char payload[MAXPAYLOAD];
};




// IPAddress server(MQTT_BROKER_ADDRESS);
// EthernetClient ethClient = ETHNetwork::getServer().available();

// // MQTT connection
// PubSubClient mqttClient(ethClient);
// PubSubClient *DccMQTT::mqClient = &mqttClient;


class DccMQTT 
{
private:
    
    static DccMQTT singleton;
    DccMQTT() = default;
    DccMQTT(const DccMQTT&); // non construction-copyable
    DccMQTT& operator=( const DccMQTT& ); // non copyable

    IPAddress server(MQTT_BROKER_ADDRESS);
    // EthernetClient ethClient = ETHNetwork::getServer().available();

    
    Queue<DccMQTTMsg> in; 
    Queue<DccMQTTMsg> out;

public:

    static DccMQTT *get() noexcept {
        return &singleton;
    }

    void setup();          // called at setup in the main ino file
    void loop();

    ~DccMQTT() = default;
};




// /**
//  * @brief MQTT broker configuration done in config.h
//  */


// // Class for setting up the MQTT connection / topics / queues for processing commands and sendig back results

// #define MAXDEVICEID 20                  // maximum length of the unique id / device id
// #define MAXTOPICS   8                   // command L,T,S,A plus response plus admin for inital handshake
// #define TCMDROOT    "command/"          // root of command topics
// #define TCMRESROOT  "result/"           // root of the result topic
// #define ADMROOT     "admin/"            // root of the admin topic where whe can do hanshakes for the inital setup
// ;                                       // esp for sec reasons i.e. making sure we are talking to the right device and
// ;                                       // not some one elses
// #define TELEMETRYROOT "telemetry/"      // telemetry topic
// #define DIAGROOT      "diag/"           // diagnostics
// #define JRMIROOT      "jrmi/"

// #define NOOFDCCTOPICS 11
// enum DccTopics {
//     CMD_L,                               // L is Loco or Layout(power on/off)            
//     CMD_T,
//     CMD_S,
//     CMD_A,
//     RESULT,
//     ADMIN,
//     TELEMETRY,
//     DIAGNOSTIC,
//     JRMI,
//     INVALID_T
// };

// /**
//  * @brief List of keywords used in the command protocol
//  * 
//  */
// #define MAX_KEYWORD_LENGTH 11
// PROGMEM const char _kRead[] = {"read"};
// PROGMEM const char _kWrite[] = {"write"};
// PROGMEM const char _kPower[] = {"power"};
// PROGMEM const char _kThrottle[] = {"throttle"};
// PROGMEM const char _kFunction[] = {"function"};
// PROGMEM const char _kCv[] = {"cv"};
// PROGMEM const char _kSpeed[] = {"speed"};
// PROGMEM const char _kLocomotive[] = {"locomotive"};
// PROGMEM const char _kValue[] = {"value"};
// PROGMEM const char _kDirection[] = {"direction"};
// PROGMEM const char _kState[] = {"state"};
// PROGMEM const char _kFn[] = {"fn"};
// PROGMEM const char _kTrack[] = {"track"};
// PROGMEM const char _kBit[] = {"bit"};

// /**
//  * @brief The ingoin and outgoing queues can hold 20 messages each; this should be bigger than the number 
//  * of statically allocated pool items whose pointers are getting pushed into the queues. 
//  * 
//  */
// #define MAXQUEUE 20     // MAX message queue length 

// class DccMQTT
// {
// private:

//     static char *deviceID;              // Unique Device Identifier; based on the chip
//     static Queue<int> inComming;             // incomming messages queue; the queue only contains indexes to the message pool
//     static Queue<int> outGoing;              // outgoing messages queue; the queue only contains indexes to the message pool

// public:
//     static char **topics;               // list of pub/sub topics            
//     static PubSubClient *mqClient;                 

//     static void setup(DCCEXParser p);   // main entry to get things going
//     static void loop();                 // recieveing commands / processing commands / publish results 
//     static bool connected();            // true if the MQ client is connected

//     static char *getDeviceID();
//     static void setDeviceID();
//     static void subscribe();            // subscribes to all relevant topics
//     static void subscribeT(char *topic);// subscribe to a particular topic for other than the std ones in subscribe (e.g. telemetry) 
//     static void publish();              // publishes a JSON message constructed from the outgoing queue (cid and result) 
//     static void printTopics();          // prints the list of subscribed topics - debug use 
//     static bool inIsEmpty();            // test if the incomming queue is empty
//     static bool outIsEmpty();           // test if the outgoing queue is empty
//     static void pushIn(uint8_t midx);   // push a command struct into the incomming queue for processing
//     static void pushOut(uint8_t midx);  // push a command struct into the incomming queue for processing  
//     static uint8_t popOut();            // pop a command struct with the result to be published 
//     static uint8_t popIn();             // pop a command struct from the in comming queue for processing
 
//     static void pub_free_memory(int fm); 

//     DccMQTT();
//     ~DccMQTT();
// };


#endif