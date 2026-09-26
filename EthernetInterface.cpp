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

extern "C" struct netif gnetif;

#include "DIAG.h"
#include "DCCTimer.h"
#include "EXmDNS.h"

EthernetUDP udpMdns;
MDNS mdns(udpMdns);

bool EthernetInterface::connected=false;
bool EthernetInterface::isUp() { 
  return connected;
}

bool EthernetInterface::setup() 
{
  connected=false;
  DIAG(F("Ethernet starting (with mDNS). Please be patient, especially if no cable is connected!"));
    
    byte mac[6];
    DCCTimer::getSimulatedMacAddress(mac);
  
  #ifdef ETHERNET_CS_PIN
    Ethernet.init(ETHERNET_CS_PIN);
  #endif

  #ifdef IP_ADDRESS
    static IPAddress myIP(IP_ADDRESS);
    Ethernet.begin(mac,myIP);
  #else
    if (Ethernet.begin(mac)==0) {
      LCD(4,F("IP: No DHCP"));
      return false;
    }
  #endif

  #ifndef ETHERNET_CS_PIN
  // Accept all multicast frames. STM32Ethernet registers multicast MAC hashes,
  // but its hash-filter path is unreliable on this MAC. This does not enable
  // promiscuous unicast reception.
  ETH->MACFFR |= (1UL << 4);
  #endif

  LCD(7, F("IP: %s"), Ethernet.localIP().toString().c_str());
  connected=true;
  return connected;
}

void EthernetInterface::setupMDNS() {
  // TODO make hostname configurable from NVS (when wifipreferences moved to nvs)
  mdns.begin(getIPAddress(), "DCC-EX-NUCLEO");
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
