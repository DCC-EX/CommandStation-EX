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
 void NodeManager::setup() {}
 void NodeManager::loop() {}
void NodeManager::cast(const FSH* format...) {}
#else
#include <AsyncUDP.h>
#include <WiFiUdp.h>
#include "WifiESP32.h"
#include "DIAG.h"
#include "StringFormatter.h"
#include "DCCEXParser.h"

constexpr uint16_t NODE_PORT = IP_PORT+1;
const IPAddress nodeMulticastIP = {239, 255, 254, 254};
constexpr uint16_t NODE_UDP_MAX = 255;

AsyncUDP udpNodeRx;

void packetLoopbackFilter(AsyncUDPPacket &packet) {
    // ignore packets sent from this address to avoid processing our own multicasts.
    if (packet.remoteIP() == WiFi.localIP()) return;
    
    // Packets are sent in to the same queue as the throttle commands to avoid multi threading.
    // The packet_listener will know that this is a node packet and parse it differently when dequeueing.
    WifiESP::packet_listener(packet); 
}

void NodeManager::setup() {
    if (!udpNodeRx.listenMulticast(nodeMulticastIP, NODE_PORT)) {
        DIAG(F("Failed to start UDP receiver for DCC-EX Node traffic"));
        return;
    }

    // the packet listener will push received packets to the command parser via a queue.
    udpNodeRx.onPacket(packetLoopbackFilter);
    DIAG(F("UDP receiver for DCC-EX Node traffic started on multicast group %s:%d"),
         nodeMulticastIP.toString().c_str(), NODE_PORT);
}

void NodeManager::cast(const FSH* format...) {
  WiFiUDP udpNodeTx;
  udpNodeTx.beginPacket(nodeMulticastIP, NODE_PORT);
  va_list args;
  va_start(args, format);
  StringFormatter::send2(&udpNodeTx, format, args);
  va_end(args);
  udpNodeTx.endPacket();
}
 
void NodeManager::parse(byte * cmd) {
    DIAG(F("Received node message: %s"), cmd);
    DCCEXParser::parseNodeTraffic(cmd);
}
#endif

