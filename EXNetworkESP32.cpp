/*
 *  © 2024 Harald Barth
 *  © 2024 Paul M. Antoine
 *  © 2026 Chris Harlow (with Github copilot)
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

#include "EXNetworkESP32.h"

#if defined(ARDUINO_ARCH_ESP32)
#include "DIAG.h"
#include "EXNetworkPreferences.h"
#include "EthernetOTA.h"

bool exNetworkUsingEthernet = false;
static WiFiUDP udpTx;

bool EXNetworkShimESP32::setup() {
  exNetworkUsingEthernet = EthernetInterface::setup();
  if (!exNetworkUsingEthernet) {
    if (!EXNetworkPreferences::getEnabled()) {
      DIAG(F(" Ethernet unavailable and WiFi disabled in preferences"));
      return false;
    }
    DIAG(F("Ethernet unavailable; trying WiFi"));
    if (!WifiESP::setup()) {
      DIAG(F("Network setup failed"));
      return false;
    }
  }
  return true;
}

void EXNetworkShimESP32::loop() {
  if (exNetworkUsingEthernet) EthernetInterface::loop();
  else WifiESP::loop();
}

bool EXNetworkShimESP32::isUp() {
  return exNetworkUsingEthernet ? EthernetInterface::isUp() : WifiESP::isUp();
}

IPAddress EXNetworkShimESP32::getIPAddress() {
  return exNetworkUsingEthernet
    ? EthernetInterface::getIPAddress()
    : WifiESP::getIPAddress();
}

void EXNetworkShimESP32::setupMDNS() {
  if (exNetworkUsingEthernet) EthernetInterface::setupMDNS();
  else WifiESP::setupMDNS();
}

void EXNetworkShimESP32::addService(const char *name, const char *proto, uint16_t port) {
  if (exNetworkUsingEthernet) EthernetInterface::addService(name, proto, port);
  else WifiESP::addService(name, proto, port);
}

void EXNetworkShimESP32::addServiceTxt(
    const char *name, const char *proto, const char *key, const char *value) {
  if (exNetworkUsingEthernet) EthernetInterface::addServiceTxt(name, proto, key, value);
  else WifiESP::addServiceTxt(name, proto, key, value);
}

void EXNetworkShimESP32::teardown() {
  if (exNetworkUsingEthernet) {
    EthernetInterface::teardown();
    exNetworkUsingEthernet = false;
  } else {
    WifiESP::teardown();
  }
}

bool EXNetworkShimESP32::startUDPListener(const IPAddress &group, uint16_t port) {
  return exNetworkUsingEthernet
	  ? EthernetInterface::startUDPListener(group, port)
	  : WifiESP::startUDPListener(group, port);
}

void EXNetworkShimESP32::stopUDPListeners() {
  // Closing on link loss prevents W5500 IGMP leaves from following the recovery joins.
  if (exNetworkUsingEthernet) EthernetInterface::stopUDPListeners();
  else WifiESP::stopUDPListeners();
}

bool EXNetworkShimESP32::startUDPTx() {
  return exNetworkUsingEthernet ? EthernetInterface::startUDPTx() : udpTx.begin(0);
}

void EXNetworkShimESP32::stopUDPTx() {
  if (exNetworkUsingEthernet) EthernetInterface::stopUDPTx();
  else udpTx.stop();
}

bool EXNetworkShimESP32::transmitUDP(const IPAddress &ip, uint16_t port,
                                     const uint8_t *data, size_t len) {
  if (exNetworkUsingEthernet) return EthernetInterface::transmitUDP(ip, port, data, len);
  return udpTx.beginPacket(ip, port) && udpTx.write(data, len) == len && udpTx.endPacket();
}

bool EXNetworkShimESP32::takeLinkLost() {
  return exNetworkUsingEthernet && EthernetInterface::takeLinkLost();
}

bool EXNetworkShimESP32::takeLinkRestored() {
  return exNetworkUsingEthernet && EthernetInterface::takeLinkRestored();
}

void EXNetworkShimESP32::networkRestored() {
  if (exNetworkUsingEthernet) EthernetInterface::restartMDNS();
  EthernetOTA.networkRestored();
}

void EXNetworkShimESP32::startNetworkServices() {
  EthernetOTA.begin<EXNetworkShimESP32>();
}

void EXNetworkShimESP32::loopNetworkServices() {
  EthernetOTA.loop();
}

void EXNetworkShimESP32::stopNetworkServices() {
  EthernetOTA.end();
}

#endif
