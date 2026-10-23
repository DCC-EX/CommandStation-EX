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
#include "defines.h"

#define DHCP_CHECK_RENEW_FAIL   (1)
#define DHCP_CHECK_REBIND_FAIL  (3)


#ifdef ARDUINO_ARCH_STM32
#include "EthernetInterface.h"
#include "EXNetworkPreferences.h"

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
  mdns.begin(getIPAddress(), EXNetworkPreferences::getHostName());
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
    case DHCP_CHECK_RENEW_FAIL:
        DIAG(F("Ethernet Error: renewed fail"));
        connected=false;
        return;
    case DHCP_CHECK_REBIND_FAIL:
        DIAG(F("Ethernet Error: rebind fail"));
        connected=false;
        return;
    default:
        break;
    }   
}

#elif defined(ARDUINO_ARCH_ESP32)
#include "defines.h"
#include "EthernetInterface.h"
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

#include "DIAG.h"
#include "DCCTimer.h"
#include "EXmDNS.h"
#include "EXNetworkPreferences.h"

// Sensible defaults for a bare ESP32 + W5500 module wired to the default VSPI pins.
// Override any of these with -D build flags if your board wires the module differently.
#ifndef ETHERNET_CS_PIN
#define ETHERNET_CS_PIN 5
#endif
#ifndef ETHERNET_SCK_PIN
#define ETHERNET_SCK_PIN 18
#endif
#ifndef ETHERNET_MISO_PIN
#define ETHERNET_MISO_PIN 19
#endif
#ifndef ETHERNET_MOSI_PIN
#define ETHERNET_MOSI_PIN 23
#endif
#ifndef ETHERNET_RST_PIN
#define ETHERNET_RST_PIN 4
#endif

EthernetUDP udpMdns;
MDNS mdns(udpMdns);

bool EthernetInterface::connected = false;
static bool ethernetLinkLost = false;
static bool ethernetLinkRestored = false;
bool EthernetInterface::isUp() {
  return connected;
}

bool EthernetInterface::setup()
{
  connected=false;
  ethernetLinkLost = false;
  ethernetLinkRestored = false;
  DIAG(F("Ethernet (W5500) starting (with mDNS). Please be patient, especially if no cable is connected!"));
  DIAG(F("SPI Pins: CS = %d, SCK = %d, MISO = %d, MOSI = %d, RST = %d"), ETHERNET_CS_PIN,ETHERNET_SCK_PIN,ETHERNET_MISO_PIN,ETHERNET_MOSI_PIN,ETHERNET_RST_PIN);
  #if defined(ETHERNET_RST_PIN) && ETHERNET_RST_PIN >= 0
  // Some W5500 boards need their reset line pulsed before SPI comms will work.
  pinMode(ETHERNET_RST_PIN, OUTPUT);
  digitalWrite(ETHERNET_RST_PIN, LOW);
  delay(10);
  digitalWrite(ETHERNET_RST_PIN, HIGH);
  delay(50);
  #endif

  SPI.begin(ETHERNET_SCK_PIN, ETHERNET_MISO_PIN, ETHERNET_MOSI_PIN, ETHERNET_CS_PIN);
  Ethernet.init(ETHERNET_CS_PIN);

  Ethernet.linkStatus(); // init the hardware
  switch (Ethernet.hardwareStatus()) {
    case EthernetW5100:
      DIAG(F("Ethernet: W5100 found"));
      break;
    case EthernetW5200:
      DIAG(F("Ethernet: W5200 found"));
      break;
    case EthernetW5500:
      DIAG(F("Ethernet: W5500 found"));
      break;

    case EthernetNoHardware:
    default:
      DIAG(F("Ethernet: W5*00 not found, using WiFi"));
      return false;
  }

  const unsigned long linkWaitStarted = millis();
  auto linkStatus = Ethernet.linkStatus();
  while (linkStatus == LinkOFF && millis() - linkWaitStarted < 5000) {
    delay(100);
    linkStatus = Ethernet.linkStatus();
  }
  if (linkStatus == LinkOFF) {
    DIAG(F("Ethernet: no link, falling back to WiFi"));
    return false;
  }

  DIAG(F("Ethernet: Link Found"));
  byte mac[6];
  DCCTimer::getSimulatedMacAddress(mac);

  #ifdef IP_ADDRESS
    static IPAddress myIP(IP_ADDRESS);
    Ethernet.begin(mac,myIP);
    DIAG(F("Ethernet static IP configured: %s"), Ethernet.localIP().toString().c_str());
  #else
    const int dhcpResult = Ethernet.begin(mac);
    DIAG(F("Ethernet DHCP begin result %d"), dhcpResult);
    if (dhcpResult == 0) {
      if (Ethernet.hardwareStatus() == EthernetNoHardware) {
        DIAG(F("Ethernet: W5*00 not found, check wiring/pins"));
      }
      LCD(4,F("IP: No DHCP"));
      return false;
    }
  #endif

  if (Ethernet.linkStatus() == LinkOFF) {
    DIAG(F("Ethernet: no link, falling back to WiFi"));
    return false;
  }

  LCD(7, F("IP: %s"), Ethernet.localIP().toString().c_str());
  DIAG(F("Ethernet ready: IP %s, gateway %s, subnet %s, link %d"),
       Ethernet.localIP().toString().c_str(),
       Ethernet.gatewayIP().toString().c_str(),
       Ethernet.subnetMask().toString().c_str(), Ethernet.linkStatus());
  connected = true;
  return connected;
}

void EthernetInterface::setupMDNS() {
  mdns.begin(getIPAddress(), EXNetworkPreferences::getHostName());
}

bool EthernetInterface::takeLinkLost() {
  const bool lost = ethernetLinkLost;
  ethernetLinkLost = false;
  return lost;
}

bool EthernetInterface::takeLinkRestored() {
  const bool restored = ethernetLinkRestored;
  ethernetLinkRestored = false;
  return restored;
}

void EthernetInterface::restartMDNS() {
  udpMdns.stop();
  const int result = mdns.begin(getIPAddress(), EXNetworkPreferences::getHostName());
  DIAG(F("Ethernet mDNS restart result %d on %s"),
       result, getIPAddress().toString().c_str());
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

void EthernetInterface::teardown() {
  connected = false;
}

void EthernetInterface::loop()
{
    static constexpr unsigned long LINK_DOWN_DEBOUNCE_MS = 500;
    static bool warnedAboutDisconnected = false;
    if (!connected) {
        if (!warnedAboutDisconnected) {
            DIAG(F("Ethernet link polling stopped: interface disconnected"));
            warnedAboutDisconnected = true;
        }
        return;
    }
    warnedAboutDisconnected = false;

    static bool warnedAboutLink = false;
    static bool confirmingLink = false;
    static bool confirmingLinkOff = false;
    static unsigned long lastLinkDiagnostic = 0;
    static unsigned long linkOnSince = 0;
    static unsigned long linkOffSince = 0;
    static unsigned long linkOffCandidateSince = 0;
    const auto linkStatus = Ethernet.linkStatus();
    const unsigned long now = millis();

    if (!warnedAboutLink) {
      if (linkStatus == LinkOFF) {
        if (!confirmingLinkOff) {
          confirmingLinkOff = true;
          linkOffCandidateSince = now;
        } else if (now - linkOffCandidateSince >= LINK_DOWN_DEBOUNCE_MS) {
          warnedAboutLink = true;
          confirmingLinkOff = false;
          confirmingLink = false;
          linkOffSince = linkOffCandidateSince;
          lastLinkDiagnostic = now;
          ethernetLinkLost = true;
          DIAG(F("Ethernet link OFF confirmed at %lu ms after %lu ms; IP %s, hardware %d"),
               now, now - linkOffSince, Ethernet.localIP().toString().c_str(),
               Ethernet.hardwareStatus());
        }
        return;
      }
      if (confirmingLinkOff) {
        DIAG(F("Ethernet transient link OFF ignored after %lu ms (status %d)"),
             now - linkOffCandidateSince, linkStatus);
        confirmingLinkOff = false;
      }
    }

    if (linkStatus == LinkOFF) {
      confirmingLink = false;
      if (now - lastLinkDiagnostic >= 10000) {
        DIAG(F("Ethernet link still OFF at %lu ms (status %d, hardware %d, IP %s)"),
             now, linkStatus, Ethernet.hardwareStatus(),
             Ethernet.localIP().toString().c_str());
        lastLinkDiagnostic = now;
      }
      return;
    }

    // link status must be ok here
    if (warnedAboutLink) {
      if (linkStatus == LinkON) {
        if (!confirmingLink) {
          confirmingLink = true;
          linkOnSince = now;
          DIAG(F("Ethernet link ON detected at %lu ms after %lu ms down; IP %s"),
               now, now - linkOffSince,
               Ethernet.localIP().toString().c_str());
        } else if (now - linkOnSince >= 1000) {
          DIAG(F("Ethernet link RESTORED at %lu ms; IP %s"),
               now, Ethernet.localIP().toString().c_str());
          warnedAboutLink = false;
          confirmingLink = false;
          ethernetLinkRestored = true;
          DIAG(F("Ethernet recovery event queued"));
        }
      } else {
        confirmingLink = false;
        DIAG(F("Ethernet link status unknown (%d)"), linkStatus);
        return;
      }
      if (warnedAboutLink) return;
    }

    // Always do this because we don't want traffic to intefere with being found!
    mdns.run();

    const int dhcpResult = Ethernet.maintain();
    if (dhcpResult != 0) {
      DIAG(F("Ethernet DHCP maintain result %d at %lu ms; IP %s"),
           dhcpResult, millis(), Ethernet.localIP().toString().c_str());
    }
    switch (dhcpResult) {
    case DHCP_CHECK_RENEW_FAIL:
        DIAG(F("Ethernet Error: renewed fail"));
        return;
    case DHCP_CHECK_REBIND_FAIL:
        DIAG(F("Ethernet Error: rebind fail"));
        return;
    default:
        break;
    }
}

#endif
