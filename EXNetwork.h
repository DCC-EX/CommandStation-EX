/*
 *  © 2026 Chris Harlow (with Github copilot)
 *  © 2026 Mathew Winters (with Github copilot)
 *
 *  All rights reserved.
 *
 *  This file is part of CommandStation-EX
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

#ifndef EXNetwork_h
#define EXNetwork_h
#include "defines.h"
#if defined(ARDUINO_ARCH_ESP32)
#include "EXNetworkESP32.h"
#else
#include "EthernetInterface.h"
typedef EthernetClient EXNetworkClient;
typedef EthernetServer EXNetworkServer;
typedef EthernetUDP EXNetworkUDPRx;
typedef EthernetUDP EXNetworkUDPTx;
#define _SHIM_ EthernetInterface
#endif

class EXNetwork {
public:
  static void setup();
  static void teardown();
  static void loop();
  static bool isUp();
  static void udpMulticast(const char *buffer);
  static void udpNodeMulticast(const char *buffer);
  static EXNetworkClient acceptWebInput();
  
private:
  static bool sendUDP(const IPAddress &ip, uint16_t port, const uint8_t *data, size_t len);
  static void processUdpPacket(EXNetworkUDPRx &udp, uint16_t localPort);
#if defined(ARDUINO_ARCH_ESP32)
  static void restartEthernetServices();
#endif
  static IPAddress myipaddress;
};

#endif
