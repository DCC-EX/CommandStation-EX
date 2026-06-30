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
void NodeManager::setup(bool throttleNode) {}
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

constexpr uint16_t NODE_PORT = IP_PORT+1;
const IPAddress nodeMulticastIP = {239, 255, 254, 254};
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
}

void NodeManager::cast(const FSH* format...) {
    if (!started) return;
  WiFiUDP udpNodeTx;
  udpNodeTx.beginPacket(nodeMulticastIP, NODE_PORT);
  va_list args;
  va_start(args, format);
  StringFormatter::send2(&udpNodeTx, format, args);
  va_end(args);
  udpNodeTx.endPacket();
  if (Diag::NODE) {
    USB_SERIAL.print(F("<* Node out: "));
    va_start(args, format);
    StringFormatter::send2(&USB_SERIAL, format, args);
    va_end(args);
    USB_SERIAL.println(F(">\n"));
  }
}

void NodeManager::cast(StringBuffer * buffer) {
    if (!started || buffer == nullptr || buffer->getLength() <= 0) return;
  WiFiUDP udpNodeTx;
  udpNodeTx.beginPacket(nodeMulticastIP, NODE_PORT);
  udpNodeTx.print(buffer->getString());
  udpNodeTx.endPacket();
}

void NodeManager::parse(byte * cmd) {
    if (Diag::NODE) DIAG(F("Node in: %s"),cmd);
    DCCEXParser::parseNodeTraffic(cmd);
}

bool NodeManager::isThrottleNode() {
    return isThrottleNodeFlag;
}

#endif

