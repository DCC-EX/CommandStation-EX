/*
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

#ifndef EXNetworkESP32_h
#define EXNetworkESP32_h

#include "defines.h"

#if defined(ARDUINO_ARCH_ESP32)
#include "WifiESP32.h"
#include "EthernetInterface.h"
#include <WiFiUdp.h>
#include <EthernetUdp.h>

extern bool exNetworkUsingEthernet;

// Runtime counterpart to the platform-specific network shims. Ethernet is
// attempted first; the selected interface handles setup and network services.
class EXNetworkShimESP32 {
public:
  static bool setup();
  static void loop();
  static bool isUp();
  static IPAddress getIPAddress();
  static void setupMDNS();
  static void addService(const char *name, const char *proto, uint16_t port);
  static void addServiceTxt(const char *name, const char *proto, const char *key, const char *value);
  static void teardown();
};

// Client facade that forwards the Client API to the active ESP32 transport.
// Accepted clients retain their transport so later interface selection cannot
// redirect operations on an existing connection.
class EXNetworkClient : public Client {
public:
  EXNetworkClient() : ethernet(exNetworkUsingEthernet) {}
  EXNetworkClient(const WiFiClient &client) : wifiClient(client), ethernet(false) {}
  EXNetworkClient(const EthernetClient &client) : ethernetClient(client), ethernet(true) {}

  int connect(IPAddress ip, uint16_t port) override {
    return ethernet ? ethernetClient.connect(ip, port) : wifiClient.connect(ip, port);
  }
  int connect(const char *host, uint16_t port) override {
    return ethernet ? ethernetClient.connect(host, port) : wifiClient.connect(host, port);
  }
  size_t write(uint8_t value) override {
    return ethernet ? ethernetClient.write(value) : wifiClient.write(value);
  }
  size_t write(const uint8_t *buffer, size_t size) override {
    return ethernet ? ethernetClient.write(buffer, size) : wifiClient.write(buffer, size);
  }
  int available() override {
    return ethernet ? ethernetClient.available() : wifiClient.available();
  }
  int read() override {
    return ethernet ? ethernetClient.read() : wifiClient.read();
  }
  int read(uint8_t *buffer, size_t size) override {
    return ethernet ? ethernetClient.read(buffer, size) : wifiClient.read(buffer, size);
  }
  int peek() override {
    return ethernet ? ethernetClient.peek() : wifiClient.peek();
  }
  void flush() override {
    if (ethernet) ethernetClient.flush();
    else wifiClient.flush();
  }
  void stop() override {
    if (ethernet) ethernetClient.stop();
    else wifiClient.stop();
  }
  uint8_t connected() override {
    return ethernet ? ethernetClient.connected() : wifiClient.connected();
  }
  operator bool() {
    return connected();
  }
  IPAddress remoteIP() {
    return ethernet ? ethernetClient.remoteIP() : wifiClient.remoteIP();
  }
  uint16_t remotePort() {
    return ethernet ? ethernetClient.remotePort() : wifiClient.remotePort();
  }
  bool operator==(const EXNetworkClient &other) {
    return ethernet == other.ethernet &&
      (ethernet ? ethernetClient == other.ethernetClient : wifiClient == other.wifiClient);
  }
  using Client::write;

private:
  WiFiClient wifiClient;
  EthernetClient ethernetClient;
  bool ethernet;
};

// Server facade that accepts clients from whichever interface setup selected.
class EXNetworkServer {
public:
  explicit EXNetworkServer(uint16_t port) : wifiServer(port), ethernetServer(port) {}
  void begin() {
    if (exNetworkUsingEthernet) ethernetServer.begin();
    else wifiServer.begin();
  }
  void end() {
    if (exNetworkUsingEthernet) ethernetServer.end();
    else wifiServer.end();
  }
  EXNetworkClient available() {
    return exNetworkUsingEthernet
      ? EXNetworkClient(ethernetServer.available())
      : EXNetworkClient(wifiServer.available());
  }

private:
  WiFiServer wifiServer;
  EthernetServerESP32 ethernetServer;
};

// UDP facade for receive, multicast, and transmit operations on the active
// ESP32 network interface.
class EXNetworkUDP : public Print {
public:
  bool begin(uint16_t port) {
    return exNetworkUsingEthernet ? ethernetUDP.begin(port) : wifiUDP.begin(port);
  }
  bool beginMulticast(IPAddress ip, uint16_t port) {
    return exNetworkUsingEthernet
      ? ethernetUDP.beginMulticast(ip, port) != 0
      : wifiUDP.beginMulticast(ip, port) != 0;
  }
  void stop() {
    if (exNetworkUsingEthernet) ethernetUDP.stop();
    else wifiUDP.stop();
  }
  int parsePacket() {
    return exNetworkUsingEthernet ? ethernetUDP.parsePacket() : wifiUDP.parsePacket();
  }
  int available() {
    return exNetworkUsingEthernet ? ethernetUDP.available() : wifiUDP.available();
  }
  int read(uint8_t *buffer, size_t size) {
    return exNetworkUsingEthernet
      ? ethernetUDP.read(buffer, size)
      : wifiUDP.read(buffer, size);
  }
  int read(char *buffer, size_t size) {
    return exNetworkUsingEthernet
      ? ethernetUDP.read(buffer, size)
      : wifiUDP.read(buffer, size);
  }
  void flush() {
    if (exNetworkUsingEthernet) ethernetUDP.flush();
    else wifiUDP.flush();
  }
  IPAddress remoteIP() {
    return exNetworkUsingEthernet ? ethernetUDP.remoteIP() : wifiUDP.remoteIP();
  }
  uint16_t remotePort() {
    return exNetworkUsingEthernet ? ethernetUDP.remotePort() : wifiUDP.remotePort();
  }
  size_t write(uint8_t value) override {
    return exNetworkUsingEthernet
      ? ethernetUDP.write(value)
      : wifiUDP.write(value);
  }
  int beginPacket(IPAddress ip, uint16_t port) {
    return exNetworkUsingEthernet
      ? ethernetUDP.beginPacket(ip, port)
      : wifiUDP.beginPacket(ip, port);
  }
  size_t write(const uint8_t *buffer, size_t size) {
    return exNetworkUsingEthernet
      ? ethernetUDP.write(buffer, size)
      : wifiUDP.write(buffer, size);
  }
  int endPacket() {
    return exNetworkUsingEthernet ? ethernetUDP.endPacket() : wifiUDP.endPacket();
  }
  using Print::write;

private:
  WiFiUDP wifiUDP;
  EthernetUDP ethernetUDP;
};

typedef EXNetworkUDP EXNetworkUDPRx;
typedef EXNetworkUDP EXNetworkUDPTx;
#define _SHIM_ EXNetworkShimESP32
#endif

#endif
