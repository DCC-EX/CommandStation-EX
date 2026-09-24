/*
 *  © 2024 Morten "Doc" Nielsen
 *  © 2023-2024 Paul M. Antoine
 *  © 2022 Bruno Sanches
 *  © 2021 Fred Decker
 *  © 2020-2022 Harald Barth
 *  © 2020-2024 Chris Harlow
 *  © 2020 Gregor Baues
 *  All rights reserved.
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
#ifdef ARDUINO_ARCH_STM32
#include "defines.h" 
#include "EthernetInterface.h"
#include <LwIP.h>
#include <STM32Ethernet.h>
#include <lwip/netif.h>

#include <EthernetUdp.h>
extern "C" struct netif gnetif;

#include "DIAG.h"
#include "DCCTimer.h"
#if defined(ARDUINO_ARCH_ESP32)
#include <SPI.h>
#endif

#ifdef DO_MDNS
#include "EXmDNS.h"

EthernetUDP udpMdns;
MDNS mdns(udpMdns);

bool EthernetInterface::connected=false;
EXEthernetServer * EthernetInterface::server= nullptr;
EthernetClient EthernetInterface::clients[MAX_SOCK_NUM];                // accept up to MAX_SOCK_NUM client connections at the same time; This depends on the chipset used on the Shield
bool EthernetInterface::inUse[MAX_SOCK_NUM];                // accept up to MAX_SOCK_NUM client connections at the same time; This depends on the chipset used on the Shield
uint8_t EthernetInterface::buffer[MAX_ETH_BUFFER+1];                    // buffer used by TCP for the recv
RingStream * EthernetInterface::outboundRing = nullptr;
bool EthernetInterface::isUp() { 
  return connected;
}

bool EthernetInterface::setup() 
{
  connected=false;
  DIAG(F("Ethernet starting (with mDNS). Please be patient, especially if no cable is connected!"));
    // Set a HOSTNAME for the DHCP request - a nice to have, but hard it seems on LWIP for STM32
    // The default is "lwip", which is **always** set in STM32Ethernet/src/utility/ethernetif.cpp
    // for some reason. One can edit it to instead read:
    //      #if LWIP_NETIF_HOSTNAME
    //      /* Initialize interface hostname */
    //      if (netif->hostname == NULL)
    //         netif->hostname = "lwip";
    //      #endif /* LWIP_NETIF_HOSTNAME */
    // Which seems more useful! We should propose the patch... so the following line actually works!
    netif_set_hostname(&gnetif, ETHERNET_HOSTNAME);   // Should probably be passed in the contructor...

    byte mac[6];
    DCCTimer::getSimulatedMacAddress(mac);

  #if defined(ARDUINO_ARCH_ESP32)
    DIAG(F("Ethernet SPI pins: CS=%d SCK=%d MISO=%d MOSI=%d RST=%d"),
      ETHERNET_CS_PIN, ETHERNET_SCK_PIN, ETHERNET_MISO_PIN, ETHERNET_MOSI_PIN, ETHERNET_RST_PIN);
    Ethernet.init(ETHERNET_CS_PIN);
    #if ETHERNET_RST_PIN >= 0
      pinMode(ETHERNET_RST_PIN, OUTPUT);
      digitalWrite(ETHERNET_RST_PIN, LOW);
      delay(2);
      digitalWrite(ETHERNET_RST_PIN, HIGH);
      delay(150);
    #endif
  #endif
  
  #ifdef IP_ADDRESS
    static IPAddress myIP(IP_ADDRESS);
    Ethernet.begin(mac,myIP);
  #else
    if (Ethernet.begin(mac)==0)
  {
    LCD(4,F("IP: No DHCP"));
    auto hw = Ethernet.hardwareStatus();
    switch (hw) {
      case EthernetNoHardware:
        DIAG(F("Ethernet HW: none detected"));
        break;
      case EthernetW5100:
        DIAG(F("Ethernet HW: W5100"));
        break;
      case EthernetW5200:
        DIAG(F("Ethernet HW: W5200"));
        break;
      case EthernetW5500:
        DIAG(F("Ethernet HW: W5500"));
        break;
      default:
        DIAG(F("Ethernet HW: unknown (%d)"), (int)hw);
        break;
    }

    auto link = Ethernet.linkStatus();
    switch (link) {
      case LinkON:
        DIAG(F("Ethernet link: ON"));
        break;
      case LinkOFF:
        DIAG(F("Ethernet link: OFF"));
        break;
      case Unknown:
      default:
        DIAG(F("Ethernet link: UNKNOWN"));
        break;
    }
    return;
  }
  #endif

  auto hw = Ethernet.hardwareStatus();
  switch (hw) {
    case EthernetNoHardware:
      DIAG(F("Ethernet HW: none detected"));
      break;
    case EthernetW5100:
      DIAG(F("Ethernet HW: W5100"));
      break;
    case EthernetW5200:
      DIAG(F("Ethernet HW: W5200"));
      break;
    case EthernetW5500:
      DIAG(F("Ethernet HW: W5500"));
      break;
    default:
      DIAG(F("Ethernet HW: unknown (%d)"), (int)hw);
      break;
  }

  auto initialLink = Ethernet.linkStatus();
  switch (initialLink) {
    case LinkON:
      DIAG(F("Ethernet link: ON"));
      break;
    case LinkOFF:
      DIAG(F("Ethernet link: OFF"));
      break;
    case Unknown:
    default:
      DIAG(F("Ethernet link: UNKNOWN"));
      break;
  }

  auto ip = Ethernet.localIP();    // look what IP was obtained (dynamic or static)
  if (!ip) {
    LCD(4,F("IP: None"));
    return;
  }
  server = new EXEthernetServer(IP_PORT); // Ethernet Server listening on default port IP_PORT
  #if defined(ARDUINO_ARCH_ESP32)
  server->begin(IP_PORT);
  #else
  server->begin();
  #endif
    if (Ethernet.begin(mac)==0) {
      LCD(4,F("IP: No DHCP"));
      return false;
    }
  #endif

  // Accept all multicast frames. STM32Ethernet registers multicast MAC hashes,
  // but its hash-filter path is unreliable on this MAC. This does not enable
  // promiscuous unicast reception.
  ETH->MACFFR |= (1UL << 4);

  LCD(7, F("IP: %s"), Ethernet.localIP().toString().c_str());
  connected=true;
  return connected;
}

void EthernetInterface::setupMDNS() {
  mdns.begin(getIPAddress(), ETHERNET_HOSTNAME);
}

void EthernetInterface::addService(const char *name, const char *proto, uint16_t port) {
  auto ptype= MDNSServiceTCP;
  if (strcmp(proto, "udp") == 0) ptype = MDNSServiceUDP;
  
  if (!mdns.addServiceRecord(name, port ,ptype)) {
    DIAG(F("addService failed %s %s %d"), name, proto, port);
  }
}

void EthernetInterface::addServiceTxt(const char *name, const char *proto, const char *key, const char *value) {
  auto serviceProto = MDNSServiceTCP;
  if (strcmp(proto, "udp") == 0) serviceProto = MDNSServiceUDP;

  if (!mdns.addTextRecord(name, serviceProto, key, value)) {
    DIAG(F("addServiceTxt failed %s=%s"), key, value);
  }
}

IPAddress EthernetInterface::getIPAddress() {
  return connected ? Ethernet.localIP(): IPAddress(0,0,0,0);
}

void EthernetInterface::loop()
{
    if (!connected) return;
      
    static bool warnedAboutLink=false;
    if (Ethernet.linkStatus() == LinkOFF){
        if (warnedAboutLink) return;
        DIAG(F("Ethernet link OFF"));
        warnedAboutLink=true;
        return;
    }
    
    // link status must be ok here 
    if (warnedAboutLink) {
      DIAG(F("Ethernet link RESTORED"));
      warnedAboutLink=false;
    } 
    
    // Always do this because we don't want traffic to intefere with being found!
    mdns.run();
    
    switch (Ethernet.maintain()) {
    case 1:
        DIAG(F("Ethernet Error: renewed fail"));
        connected=false;
        return;
    case 3:
        DIAG(F("Ethernet Error: rebind fail"));
        connected=false;
        return;
    default:
        break;
    }   
}

#endif
