#ifndef _MQTTBrokers_h_
#define _MQTTBrokers_h_

// Define Broker configurations; Values are provided in the following order 
// MQTT_BROKER_PORT 9883 
// MQTT_BROKER_DOMAIN "dcclms.modelrailroad.ovh"
// MQTT_BROKER_ADDRESS 51, 210, 151, 143
// MQTT_BROKER_USER "dcccs"
// MQTT_BROKER_PASSWD "dcccs$3020"
// MQTT_BROKER_CLIENTID_PREFIX "dcc$lms-"

// Local server no user / pwd / prefix required 
// EthernetShields / Arduino do not support securte transport i.e. on either port 443 or 8883 for MQTTS on most broker installations
// Once we support the ESP / Wifi as Transport medium we may get TLS capabilities for data in transit i.e. can use the 443/8883 ports
#define LOCAL_MQTT_BROKER F("LOCALMQ"), new MQTTBroker( 1883, {192, 168, 0, 51}, F("my.local.server"))
// Local server with user / pwd and no prefix 
#define LOCAL_MQTT_USER_BROKER F("LOCALMQ"), new MQTTBroker( 1883, {192, 168, 0, 51}, F("my.local.server"), F("myuser"), F("mypassword"))
// Cloud server 
#define DCCEX_MQTT_BROKER F("DCCEXMQ"), new MQTTBroker( 9883, {51, 210, 151, 143}, F("dcclms.modelrailroad.ovh"), F("dcccs"), F("dcccs$3020"), F("dcc$lms-"))
// Cloud server 
#define DCCEX_MQTT_DOMAIN_BROKER F("DCCEXMQ"), new MQTTBroker( 9883, F("dcclms.modelrailroad.ovh"), F("dcccs"), F("dcccs$3020"), F("dcc$lms-"))
// Mosquitto test server 
#define DCCEX_MOSQUITTO F("Mosquitto"), new MQTTBroker(1883, F("test.mosquitto.org"))
// HiveMQ test server 
#define DCCEX_HIVEMQ F("HiveMQ"), new MQTTBroker(1883, F("broker.hivemq.com"))

#endif