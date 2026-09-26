/*
 *  © 2023-2024 Paul M. Antoine
 *  © 2021 Neil McKechnie
 *  © 2021 Mike S
 *  © 2021 Fred Decker
 *  © 2020-2024 Harald Barth
 *  © 2020-2026 Chris Harlow
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
#if defined (ARDUINO_ARCH_STM32)
#include "defines.h"
#include <Arduino.h>

#if __has_include ( "STM32Ethernet.h")
  // Nucleo devices with builtin ethernet
  // Refer to platformio.ini library configs
  #include <LwIP.h>
  #include <STM32Ethernet.h>
  #include <lwip/netif.h>
#else
  //Nucleo devices with external ethernet module
  // Refer to platformio.ini library configs
  #define ETHERNET_CS_PIN 10
  #include <Ethernet.h>
#endif

#include <EthernetUdp.h>

extern "C" struct netif gnetif;

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
