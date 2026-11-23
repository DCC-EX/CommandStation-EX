/*
 *  © 2026 Mathew Winters (with Github copilot)
 *  © 2026 Herb Morton
 *  © 2023-2024 Paul M. Antoine
 *  © 2021 Neil McKechnie
 *  © 2021 Mike S
 *  © 2021 Fred Decker
 *  © 2020-2024 Harald Barth
 *  © 2020-2026 Chris Harlow
 *  © 2020 Gregor Baues
 * 
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
#include <Arduino.h>

#if defined(ETHERNET_CS_PIN) || (defined(ARDUINO_ARCH_STM32) && __has_include("STM32Ethernet.h"))
#define HAS_ETHERNET
#define HAS_ETHERNET_HW
#elif defined(ARDUINO_ARCH_ESP32)
#define HAS_ETHERNET
#endif

#ifdef HAS_ETHERNET

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
  using EthernetServer::begin;
  void begin(uint16_t) { EthernetServer::begin(); }
  void end() {}
};
#elif __has_include("STM32Ethernet.h") && !defined(ETHERNET_CS_PIN)
  // Nucleo devices with builtin ethernet
  // Refer to platformio.ini library configs
  #include <LwIP.h>
  #include <STM32Ethernet.h>
  #include <lwip/netif.h>
  extern "C" struct netif gnetif;
#else
  //Nucleo devices with external ethernet module
  // Refer to platformio.ini library configs
  #include <Ethernet.h>
#endif
#endif

#include <EthernetUdp.h>
#include "EXNetworkUdp.h"

#if defined(ETHERNET_CS_PIN)
#include <utility/w5100.h>
class EthernetUDP_Exposed : public EthernetUDP {
public:
  uint8_t getSocketIndex() const { return sockindex; }
};
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
#if defined(ARDUINO_ARCH_ESP32)
  static bool takeLinkLost();
  static bool takeLinkRestored();
  static void restartMDNS();
#endif
  static void teardown();
 static bool startUDPListener(const IPAddress &ip, uint16_t port);
  static void stopUDPListeners();
  static bool receiveUDPPacket(EXNetworkUdpPacket &packet);
   
 private:
    static bool connected;
    
};

#endif
