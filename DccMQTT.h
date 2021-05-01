#ifndef _DccMQTT_h_
#define _DccMQTT_h_

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
#include <Ethernet.h>
#include <Dns.h>

#define MAXPAYLOAD 64
#define MAXDOMAINLENGTH 32

#define MAXTBUF 50  //!< max length of the buffer for building the topic name ;to be checked
#define MAXTMSG 120 //!< max length of the messages for a topic               ;to be checked PROGMEM ?
#define MAXTSTR 30  //!< max length of a topic string
#define MAXCONNECTID 40
#define CLIENTIDSIZE 6
#define MAXRECONNECT 5

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

    IPAddress resovleBroker(const FSH *d){
        DNSClient dns;
        IPAddress bip;

        char domain[MAXDOMAINLENGTH];      
        strcpy_P(domain, (const char *)d);

        dns.begin(Ethernet.dnsServerIP());
        if (dns.getHostByName(domain, bip) == 1)
        {
            DIAG(F("MQTT Broker/ %s = %d.%d.%d.%d"), domain, bip[0], bip[1],bip[2],bip[3]);
        }
        else
        {
            DIAG(F("MQTT Dns lookup for %s failed"), domain);
        }
        return bip;
    }

    MQTTBroker(int p, IPAddress i, const FSH *d) : port(p), ip(i), domain(d), cType(1) {};
    MQTTBroker(int p, IPAddress i, const FSH *d, const FSH *uid, const FSH *pass) : port(p), ip(i), domain(d), user(uid), pwd(pass), cType(2){};
    MQTTBroker(int p, IPAddress i, const FSH *d, const FSH *uid, const FSH *pass, const FSH *pfix) : port(p), ip(i), domain(d), user(uid), pwd(pass), prefix(pfix), cType(3){};
    MQTTBroker(int p, const FSH *d, const FSH *uid, const FSH *pass, const FSH *pfix) : port(p), domain(d), user(uid), pwd(pass), prefix(pfix), cType(4)
    {
        ip = resovleBroker(d);
    };
    MQTTBroker(int p, const FSH *d, const FSH *uid, const FSH *pass) : port(p), domain(d), user(uid), pwd(pass), cType(5)
    {
        ip = resovleBroker(d);
    };
    MQTTBroker(int p, const FSH *d) : port(p), domain(d), cType(6)
    {
        ip = resovleBroker(d);
    };
};

struct DccMQTTMsg
{
    char payload[MAXPAYLOAD];
};

enum DccMQTTState
{
    INIT,
    CONFIGURED, // server/client objects set
    CONNECTED   // mqtt broker is connected
};

class DccMQTT
{
private:
    static DccMQTT singleton;
    DccMQTT() = default;
    DccMQTT(const DccMQTT &);            // non construction-copyable
    DccMQTT &operator=(const DccMQTT &); // non copyable

    void setup(const FSH *id, MQTTBroker *broker);
    void connect();                 // (re)connects to the broker 
    boolean subscribe();           

    EthernetClient  ethClient;      // TCP Client object for the MQ Connection
    IPAddress       server;         // MQTT server object
    PubSubClient    mqttClient;     // PubSub Endpoint for data exchange
    MQTTBroker      *broker;        // Broker configuration object as set in config.h
 
    Queue<DccMQTTMsg> in;
    Queue<DccMQTTMsg> out;

    char clientID[(CLIENTIDSIZE*2)+1];

    DccMQTTState mqState = INIT;

public:
    static DccMQTT *get() noexcept
    {
        return &singleton;
    }

    bool isConfigured() { return mqState == CONFIGURED; };
    bool isConnected() { return mqState == CONNECTED; };
    void setState(DccMQTTState s) { mqState = s; };

    void setup(); // called at setup in the main ino file
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