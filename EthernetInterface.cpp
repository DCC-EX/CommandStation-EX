/*
 *  © 2026 Mathew Winters
 *  © 2024 Morten "Doc" Nielsen
 *  © 2023-2024 Paul M. Antoine
 *  © 2022 Bruno Sanches
 *  © 2021 Fred Decker
 *  © 2020-2022 Harald Barth
 *  © 2020-2026 Chris Harlow
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

struct UdpListener {
  EthernetUDP udp;
  uint16_t port = 0;
};
static UdpListener udpListeners[2];

bool EthernetInterface::startUDPListener(const IPAddress &group, uint16_t port) {
  for (auto &listener : udpListeners) {
    if (listener.port) continue;
    if (!listener.udp.beginMulticast(group, port)) {
      listener.udp.stop();
      return false;
    }
    listener.port = port;
    return true;
  }
  DIAG(F("No free UDP listener for port %d"), port);
  return false;
}

void EthernetInterface::stopUDPListeners() {
  for (auto &listener : udpListeners) {
    listener.udp.stop();
    listener.port = 0;
  }
}

bool EthernetInterface::receiveUDPPacket(EXNetworkUdpPacket &packet) {
  for (auto &listener : udpListeners) {
    if (!listener.port) continue;
    while (listener.udp.parsePacket() > 0) {
      int packetSize = listener.udp.available();
      if (packetSize <= 2) {
        listener.udp.flush();
        continue;
      }
      int length = listener.udp.read(packet.data, EXNETWORK_UDP_MAX_SIZE);
      packet.remoteIP = listener.udp.remoteIP();
      if (length < packetSize) {
        DIAG(F("Short UDP packet read %d < %d from %s"), length, packetSize,
             packet.remoteIP.toString().c_str());
        listener.udp.flush();
        continue;
      }
      packet.length = length;
      packet.data[length] = 0;
      packet.localPort = listener.port;
      return true;
    }
  }
  return false;
}

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

  #ifndef ETHERNET_CS_PIN
  // The link-state handler enables IGMP before multicast listeners are started.
  Ethernet.schedule();
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
#ifdef ETHERNET_W5500_RX_DIAG_PORT
#include <utility/w5100.h>
#endif
#if __has_include(<esp_mac.h>)
#include <esp_mac.h>
#else
#include <esp_system.h>
#endif

#include "DIAG.h"
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

struct UdpListener {
  EthernetUDP udp;
  IPAddress group;
  uint16_t port = 0;
#ifdef ETHERNET_W5500_RX_DIAG_PORT
  uint8_t diagSocket = MAX_SOCK_NUM;
  unsigned long lastStallLog = 0;
#endif
};
static UdpListener udpListeners[2];

#ifdef ETHERNET_W5500_RX_DIAG_PORT
static uint8_t logW5500Socket(uint16_t port) {
  if (Ethernet.hardwareStatus() != EthernetW5500) {
    DIAG(F("W5500 RX diagnostic unavailable: controller is not a W5500"));
    return MAX_SOCK_NUM;
  }
  SPI.beginTransaction(SPI_ETHERNET_SETTINGS);
  for (uint8_t socket = 0; socket < MAX_SOCK_NUM; ++socket) {
    if (W5100.readSnPORT(socket) != port) continue;
    const unsigned int status = W5100.readSnSR(socket);
    if (status == SnSR::CLOSED) continue;
    const unsigned int mode = W5100.readSnMR(socket);
    uint8_t dst[4];
    W5100.readSnDIPR(socket, dst);
    const uint16_t dstPort = W5100.readSnDPORT(socket);
    const uint16_t queued = W5100.readSnRX_RSR(socket);
    SPI.endTransaction();
    DIAG(F("W5500 socket %d port %d: status 0x%x mode 0x%x dst %s:%d queued %d"),
         socket, port, status, mode, IPAddress(dst).toString().c_str(), dstPort, queued);
    return socket;
  }
  SPI.endTransaction();
  DIAG(F("W5500 RX diagnostic: no open socket for port %d"), port);
  return MAX_SOCK_NUM;
}

static int parseW5500Packet(UdpListener &listener) {
  const auto socket = listener.diagSocket;
  if (socket == MAX_SOCK_NUM) return listener.udp.parsePacket();

  SPI.beginTransaction(SPI_ETHERNET_SETTINGS);
  const uint16_t queued = W5100.readSnRX_RSR(socket);
  SPI.endTransaction();
  const int parsed = listener.udp.parsePacket();
  if (queued > 0 || parsed > 0) {
    const auto now = millis();
    if (parsed > 0 || now - listener.lastStallLog >= 1000) {
      if (parsed <= 0) listener.lastStallLog = now;
      DIAG(F("W5500 RX socket %d port %d: queued %d parsed %d"),
           socket, listener.port, queued, parsed);
    }
  }
  return parsed;
}
#endif

bool EthernetInterface::connected = false;
static bool ethernetLinkLost = false;
static bool ethernetLinkRestored = false;
bool EthernetInterface::isUp() {
  return connected;
}

bool EthernetInterface::startUDPListener(const IPAddress &group, uint16_t port) {
  for (auto &listener : udpListeners) {
    if (listener.port) continue;
    if (!listener.udp.beginMulticast(group, port)) {
      listener.udp.stop();
      return false;
    }
    listener.group = group;
    listener.port = port;
#ifdef ETHERNET_W5500_RX_DIAG_PORT
    listener.diagSocket = MAX_SOCK_NUM;
    if (port == ETHERNET_W5500_RX_DIAG_PORT) {
      listener.diagSocket = logW5500Socket(port);
      listener.lastStallLog = millis() - 1000UL;
    }
#endif
    return true;
  }
  DIAG(F("No free UDP listener for port %d"), port);
  return false;
}

void EthernetInterface::stopUDPListeners() {
  for (auto &listener : udpListeners) {
    listener.udp.stop();
    listener.port = 0;
  }
}

bool EthernetInterface::receiveUDPPacket(EXNetworkUdpPacket &packet) {
  for (auto &listener : udpListeners) {
    if (!listener.port) continue;
#ifdef ETHERNET_W5500_RX_DIAG_PORT
    while (parseW5500Packet(listener) > 0) {
#else
    while (listener.udp.parsePacket() > 0) {
#endif
      int packetSize = listener.udp.available();
      if (packetSize <= 2) {
        listener.udp.flush();
        continue;
      }
      int length = listener.udp.read(packet.data, EXNETWORK_UDP_MAX_SIZE);
      packet.remoteIP = listener.udp.remoteIP();
      if (length < packetSize) {
        DIAG(F("Short UDP packet read %d < %d from %s"), length, packetSize,
             packet.remoteIP.toString().c_str());
        listener.udp.flush();
        continue;
      }
      packet.length = length;
      packet.data[length] = 0;
      packet.localPort = listener.port;
      return true;
    }
  }
  return false;
}

bool EthernetInterface::sendUDPMulticast(const IPAddress &group, uint16_t port, const uint8_t *data, size_t len) {
  for (auto &listener : udpListeners) {
    if (!listener.port || listener.port != port || listener.group != group) continue;
    // Keep the W5500 socket's multicast destination and MAC paired.
    return listener.udp.beginPacket(group, port)
      && listener.udp.write(data, len) == len
      && listener.udp.endPacket();
  }
  DIAG(F("Ethernet: no multicast socket for %s:%d"), group.toString().c_str(), port);
  return false;
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
  const esp_err_t macResult = esp_read_mac(mac, ESP_MAC_ETH);
  if (macResult != ESP_OK) {
    DIAG(F("Ethernet: cannot read ESP32 Ethernet MAC (%d), falling back to WiFi"), macResult);
    return false;
  }

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
