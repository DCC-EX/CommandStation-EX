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


// Fwd decl for the callback handlers
void mqttDCCEXCallback(MQTTInterface *mqtt, csmsg_t &tm);
void mqttProtocolCallback(MQTTInterface *mqtt, csmsg_t &tm);
void mqttMCallback(MQTTInterface *mqtt, csmsg_t &tm);

typedef void (*CallbackFunc)(MQTTInterface *mqtt, csmsg_t &tm);

template<class M, class N>
struct CallbackFunction {
	M first;
	N second;
};
using CallbackFunctions = CallbackFunction<char, CallbackFunc>[MAX_CALLBACKS];

// lookup table for the protocol handle functions
constexpr CallbackFunctions vec = {
		{'<', mqttDCCEXCallback},
		{'{', mqttProtocolCallback},
        {'m', mqttMCallback}
};

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
 * @brief lookup of the proper function for < or { based commands 
 *  
 * @param c 
 * @return CallbackFunc 
 */
auto protocolDistributor(const char c) -> CallbackFunc {
	for (auto &&f : vec)
	{
		if (f.first == c)
			return f.second;
	}
	return nullptr;
}

void protocolHandler(MQTTInterface *mqtt, csmsg_t &tm) {
	protocolDistributor(tm.cmd[0])(mqtt, tm);  
}

/**
 * @brief Callback for handling 'm' MQTT Protocol commands (deprecated)
 * @deprecated to be replaced by '{' commands in simple JSON format
 */
void mqttMCallback(MQTTInterface *mqtt, csmsg_t &tm)
{
    auto clients = mqtt->getClients();
    DIAG(F("MQTT m - Callback"));
    switch (tm.cmd[1])
        {
        case 'i': // Inital handshake message to create the tunnel
        {
            char buffer[MAXPAYLOAD];
            char *tmp = tm.cmd + 3;
            auto length = strlen(tm.cmd);
            strlcpy(buffer, tmp, length);
            buffer[length - 4] = '\0';

            DIAG(F("MQTT buffer %s - %s - %s - %d"), tm.cmd, tmp, buffer, length);

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
        default:
        {
            return;
        }
        }
}

/**
 * @brief Callback for handling '{' MQTT Protocol commands
 */
void mqttProtocolCallback(MQTTInterface *mqtt, csmsg_t &tm)
{
    DIAG(F("MQTT Protocol - Callback"));
}

/**
 * @brief Callback for handling '<' DccEX commands
 */
void mqttDCCEXCallback(MQTTInterface *mqtt, csmsg_t &tm)
// void mqttDCCEXCallback(MQTTInterface *mqtt, char *topic, char *payload, unsigned int length)
{
    DIAG(F("MQTT DCCEX - Callback"));
    if (!tm.mqsocket)
    {
        DIAG(F("MQTT Can't identify sender; command send on wrong topic"));
        return;
    }
    int idx = mqtt->getPool()->setItem(tm); // Add the recieved command to the pool
    if (idx == -1)
    {
        DIAG(F("MQTT Command pool full. Could not handle recieved command."));
        return;
    }
    mqtt->getIncomming()->push(idx); // Add the index of the pool item to the incomming queue

    // don't show the topic as we would have to save it also just like the payload
    if (Diag::MQTT)
        DIAG(F("MQTT Message arrived: [%s]"), tm.cmd);
}