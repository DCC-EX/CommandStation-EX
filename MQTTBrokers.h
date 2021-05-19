
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
#ifndef _MQTTBrokers_h_
#define _MQTTBrokers_h_

// Defines preconfigured mqtt broker configurations

// EthernetShields / Arduino do not support secure transport i.e. on either port 443 or 8883 for MQTTS on most broker installations
// Once we support the ESP / Wifi as Transport medium we may get TLS capabilities for data in transit i.e. can use the 443/8883 ports


#define MQPWD F(MQTT_PWD)
#define MQUID F(MQTT_USER)
#define MQPREFIX F(MQTT_PREFIX)

// Cloud server provided by the DccEX team for testing purposes; apply for a uid/pwd on discord 
#define DCCEX_MQTT_BROKER F("DccexMQ"), new MQTTBroker( 9883, F("dcclms.modelrailroad.ovh"), MQUID, MQPWD, MQPREFIX)
// Mosquitto test server 
#define DCCEX_MOSQUITTO F("Mosquitto"), new MQTTBroker(1883, F("test.mosquitto.org"))
// HiveMQ test server 
#define DCCEX_HIVEMQ F("HiveMQ"), new MQTTBroker(1883, F("broker.hivemq.com"))


#endif