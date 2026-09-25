/*
 *  © 2023-2024 Paul M. Antoine
 *  © 2021 Neil McKechnie
 *  © 2021 Mike S
 *  © 2021 Fred Decker
 *  © 2020-2024 Harald Barth
 *  © 2020-2024 Chris Harlow
 *  © 2020 Gregor Baues
 *  All rights reserved.
 *  
 *  This file is part of DCC-EX/CommandStation-EX
 *
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
 *  Ethernet Interface added by Gregor Baues
 */

#ifndef EthernetInterface_h
#define EthernetInterface_h
#include "defines.h"
#if defined (ARDUINO_NUCLEO_F429ZI) || defined (ARDUINO_NUCLEO_F439ZI) || defined (ARDUINO_NUCLEO_F4X9ZI) || (defined(ARDUINO_ARCH_ESP32) && ETHERNET_ON)

#include <Arduino.h>
#if defined(ARDUINO_ARCH_ESP32)
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

// The ESP32 core's Server base class declares begin(uint16_t port=0) as pure
// virtual, but arduino-libraries/Ethernet's EthernetServer only overrides the
// no-arg begin(), leaving it abstract. Provide the missing override here.
// end() is also added since arduino-libraries/Ethernet has no such call.
class EthernetServerESP32 : public EthernetServer {
 public:
  EthernetServerESP32(uint16_t port) : EthernetServer(port) {}
  void begin(uint16_t /*port*/) { EthernetServer::begin(); }
  void end() {}
};
#else
#include <LwIP.h>
#include <STM32Ethernet.h>
#include <lwip/netif.h>
#include <EthernetUdp.h>

extern "C" struct netif gnetif;
#endif

class EthernetInterface {

 public:
     
  static bool setup();
  static void loop();
  static bool isUp();
  static IPAddress getIPAddress();
  static void setupMDNS();
  static void addService(const char *name, const char *proto, uint16_t port);
  static void addServiceTxt(const char *name, const char *proto, const char *key, const char *value);
  static void teardown();
  static bool startUDPListener(const IPAddress &ip, uint16_t port);
   
 private:
    static bool connected;
    
};
#endif // ETHERNET_ON
#endif
