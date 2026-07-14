/*
 *  © 2026 Chris Harlow
 *  All rights reserved.
 *  
 *  This file is part of DCC-EX
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
 */

#include "NodeManager.h"

#ifndef ARDUINO_ARCH_ESP32
// dummy NodeManager without ESP32 support
void NodeManager::setup(bool throttleNode) { (void)throttleNode;}
void NodeManager::cast(const FSH* format...) {
    (void)format; // avoid unused parameter warning
}
void NodeManager::cast(StringBuffer * buffer) {
    (void)buffer; // avoid unused parameter warning
}
bool NodeManager::isThrottleNode() {
    return true; // default to true for non-ESP32 platforms
}
#else
#include <AsyncUDP.h>
#include <WiFiUdp.h>
#include "WifiESP32.h"
#include "DIAG.h"
#include "StringFormatter.h"
#include "DCCEXParser.h"
#include "Turnouts.h"

constexpr uint16_t NODE_PORT = IP_PORT+1;
#ifndef NODE_GROUP
    #define NODE_GROUP 254
#endif 
const IPAddress nodeMulticastIP = {239, 255, 254, NODE_GROUP};
AsyncUDP udpNodeRx;
bool NodeManager::started = false;
bool NodeManager::isThrottleNodeFlag = true;

void NodeManager::setup(bool throttleNode) {
    isThrottleNodeFlag = throttleNode;
    if (!udpNodeRx.listenMulticast(nodeMulticastIP, NODE_PORT)) {
        DIAG(F("Failed to start UDP receiver for DCC-EX Node traffic"));
        return;
    }
    started = true;

    // the packet listener will push received packets to the command parser via a queue.
    udpNodeRx.onPacket(WifiESP::packet_listener);
    DIAG(F("UDP receiver for DCC-EX Node traffic started on multicast group %s:%d"),
         nodeMulticastIP.toString().c_str(), NODE_PORT);

    if (!throttleNode) {
        DIAG(F("This node is not a throttle node.  It will not accept throttle connectioms"));
    }
}

void NodeManager::cast(const FSH* format...) {
    if (!started) return;
    StringBuffer buffer(260); // max unfragmented UDP payload over Ethernet/WiFi
    va_list args;
    va_start(args, format);
    StringFormatter::send2(&buffer, format, args);
    va_end(args);
    cast(&buffer);
}

void NodeManager::cast(StringBuffer * buffer) {
    if (!started || buffer == nullptr || buffer->getLength() <= 0) return;
  WiFiUDP udpNodeTx;
  if (!udpNodeTx.beginPacket(nodeMulticastIP, NODE_PORT)) {
    DIAG(F("Failed to start UDP transmitter for node out %s"),buffer->getString());
    return;
  }
  udpNodeTx.print(buffer->getString());
  udpNodeTx.endPacket();
  if (Diag::NODE) DIAG(F("Node out: %s"), buffer->getString());
}

void NodeManager::parse(byte * cmd) {
    if (Diag::NODE) DIAG(F("Node in: %s"),cmd);
    DCCEXParser::parseNodeTraffic(cmd);
}

bool NodeManager::isThrottleNode() {
    return isThrottleNodeFlag;
}

#endif

