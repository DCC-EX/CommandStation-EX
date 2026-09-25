#include "EXNetwork.h"
#include "DIAG.h"
#include "CommandDistributor.h"
#include "NodeManager.h"
#include "RingStream.h"
#include <vector>
#include "WiThrottle.h"
#include "Websockets.h"

#ifdef ARDUINO_ARCH_ESP32
#include "EthernetOTA.h"
#endif

#ifndef NODE_GROUP
#define NODE_GROUP 254
#endif
#ifndef IP_PORT
#define IP_PORT 2560
#endif

// Common points of interaction for network interfaces
EXNetworkServer webServer(80);
EXNetworkServer throttleServer(IP_PORT);


// udp throtttle traffic 
EXNetworkUDPRx udpThrottleRx;
EXNetworkUDPRx udpNodeRx;
EXNetworkUDPTx udpTx;

constexpr uint16_t NODE_PORT = IP_PORT + 1;
const IPAddress nodeMulticastIP = {239, 255, 254, NODE_GROUP};
IPAddress throttleMulticastIP = {239, 255, 255, 0 /* will become WiFi.localIP()[3] */};

constexpr uint16_t UDP_COMMAND_MAX = 255;    // max inbound command payload (fits one DCC-EX command)
constexpr uint16_t UDP_RESPONSE_MAX  = 1472; // max outbound payload (Ethernet MTU 1500 - 20 IP - 8 UDP)
static RingStream *outboundRing = new RingStream(10240);

static std::vector<IPAddress> udpDiscoveryClients;

static void rememberUdpDiscoveryClient(const IPAddress &ip) {
  for (const auto &knownIp : udpDiscoveryClients) {
    if (knownIp == ip) return;
  }
  udpDiscoveryClients.push_back(ip);
}

// A network client must be maintained.
class exNetworkClient {
public:
  exNetworkClient(EXNetworkClient c) {
    client = c;
    inUse = true;
  }

  bool active(byte clientId) {
    if (!inUse) return false;
    if (!EXNetwork::isUp()) {
      DIAG(F("Remove client %d"), clientId);
      CommandDistributor::forget(clientId);
      client.stop();
      inUse = false;
      return false;
    }
    if (!client.connected()) {
      DIAG(F("Remove disconnected client %d"), clientId);
      CommandDistributor::forget(clientId);
      client.stop();
      inUse = false;
      return false;
    }
    return true;
  }
  bool recycle(EXNetworkClient c) {
    if (client == c) {
      // ESP32: caller will warn about duplicate clients/recycling
      // STM32: this is normal behavior due to .available() returning clients with unread data.
      inUse = true;
      return true;
    }
    if (!inUse) {
      client = c;
      inUse = true;
      return true;
    }
    return false;
  };

  EXNetworkClient client;

private:
  bool inUse;
};
static std::vector<exNetworkClient> throttleClients; // A list to hold all clients.

void EXNetwork::setup() {
  #ifdef ARDUINO_ARCH_STM32
  udpTx.stop();
  udpThrottleRx.stop();
  udpNodeRx.stop();
  #endif

  _SHIM_::setup();
  auto ipaddress = _SHIM_::getIPAddress();
  throttleMulticastIP[3] = ipaddress[3];

  #ifdef ARDUINO_ARCH_STM32
  // STM32Ethernet enables NETIF_FLAG_IGMP from its link-state handler.
  // Run it before beginMulticast(), otherwise igmp_joingroup() can fail.
  Ethernet.schedule();
  #endif

  // Socket server for old style throttle connections.
  throttleServer.begin(IP_PORT);
  DIAG(F("throttleServer started on port %d"), IP_PORT);

  // Web server for browser interface.
  webServer.begin(80);
  DIAG(F("webServer started on port 80"));

  if (!udpTx.begin(0)) {
    DIAG(F("udpTx failed to start"));
  }

  bool nodeFail = false;

  #ifdef ARDUINO_ARCH_STM32
  // listen for incoming throttle traffic via UDP.
    if (udpThrottleRx.beginMulticast(throttleMulticastIP, IP_PORT)) {
    DIAG(F("udpThrottleRx started on %s:%d"),
      throttleMulticastIP.toString().c_str(), IP_PORT);
  }
  
 
  // All node traffic via UDP multicast.
  if (udpNodeRx.beginMulticast(nodeMulticastIP, NODE_PORT)) {
    DIAG(F("udpNodeRx started on %s:%d"),
         nodeMulticastIP.toString().c_str(), NODE_PORT);
  } else {
    DIAG(F("udpNodeRx multicast join failed for %s:%d"),
         nodeMulticastIP.toString().c_str(), NODE_PORT);
    nodeFail = true;
  }
  #endif

  #ifdef ARDUINO_ARCH_ESP32
  // Listen for incoming throttle traffic via UDP.
    if (udpThrottleRx.beginMulticast(throttleMulticastIP, IP_PORT)) {
    DIAG(F("udpThrottleRx started on %s:%d"),
      throttleMulticastIP.toString().c_str(), IP_PORT);
  }
  // Receive node traffic via UDP multicast.
  if (udpNodeRx.beginMulticast(nodeMulticastIP, NODE_PORT)) {
    DIAG(F("udpNodeRx started on %s:%d"),
         nodeMulticastIP.toString().c_str(), NODE_PORT);
  } else {
    nodeFail = true;
  }
  #endif

  NodeManager::setup(!nodeFail);

  _SHIM_::setupMDNS();
  _SHIM_::addService("http", "tcp", 80);
  _SHIM_::addServiceTxt("http", "tcp", "path", "/");
  _SHIM_::addService("withrottle", "tcp", IP_PORT);
  _SHIM_::addService("dcc-ex", "tcp", IP_PORT);
  _SHIM_::addService("dcc-ex", "udp", IP_PORT);

  _SHIM_::addServiceTxt("dcc-ex", "udp", "multicast", "true");
  _SHIM_::addServiceTxt("dcc-ex", "udp", "group", throttleMulticastIP.toString().c_str());
  _SHIM_::addServiceTxt("dcc-ex", "udp", "port", String(IP_PORT).c_str());
  #ifdef ARDUINO_ARCH_ESP32
  EthernetOTA.begin<_SHIM_>();
  #endif
}

bool EXNetwork::isUp() { return _SHIM_::isUp(); }

void EXNetwork::udpMulticast(const char *buffer) {
  if (buffer == NULL || !isUp()) return;
  int count = strlen(buffer);
  if (count <= 0 || count > UDP_RESPONSE_MAX) {
    DIAG(F("udpMulticast: Invalid count %d, %s"), count, buffer);
    return;
  }

  // Regardless of the clientId, we can send it via UDP multicast.
  if (!sendUDP(throttleMulticastIP, IP_PORT, (const uint8_t *)buffer, count)) {
    DIAG(F("udpSend Multicast failed"));
  }

  // Also unicast to clients discovered via <#> probe packets.
  for (const auto &clientIp : udpDiscoveryClients) {
    if (!sendUDP(clientIp, IP_PORT, (const uint8_t *)buffer, count)) {
      DIAG(F("udpSend unicast failed to %s:%d"), clientIp.toString().c_str(), IP_PORT);
    }
  }
}

void EXNetwork::udpNodeMulticast(const char *buffer) {
  if (buffer == NULL || !isUp()) return;
  if (!sendUDP(nodeMulticastIP, NODE_PORT,
                       (const uint8_t *)buffer, strlen(buffer))) {
    DIAG(F("udpNodeMulticast failed"));
  }
}

EXNetworkClient EXNetwork::acceptWebInput() {
  return webServer.available();
}

void EXNetwork::processUdpPacket(EXNetworkUDPRx &udp, uint16_t localPort) {
  int packetSize = udp.available();
  if (packetSize <= 2) return;

  // Read the incoming UDP packet into a buffer.
  byte data[UDP_COMMAND_MAX + 1];
  int length = udp.read(data, UDP_COMMAND_MAX);
  if (length <= 2) return;
  data[length] = 0;

  // Pass node traffic to NodeManager
  if (localPort == NODE_PORT) {
    NodeManager::parse(data);
    return;
  }
 
  // detect UDP throttles that can't listen to the UDP broadcast and remember them for unicast responses
  IPAddress remoteIP = udp.remoteIP();
  if (length >= 3 && data[0] == '<' && data[1] == '#' && data[2] == '>') {
    rememberUdpDiscoveryClient(remoteIP);
  }
 
  // process command collecting results in a buffer
  StringBuffer response(UDP_RESPONSE_MAX);
  DCCEXParser::parse(&response, data);
  if (response.getLength() > 0) {
    sendUDP(remoteIP, IP_PORT,
            (const byte *)response.getString(), response.getLength());
  }
}

void EXNetwork::loop() {
  _SHIM_::loop(); // Wi-Fi/Ethernet continuous support.
  if (!isUp()) return;
  
  // Track new socket clients. 
  // STM32: .available() returns any client with data waiting.
  // ESP32: .available() returns only new clients.

  size_t clientId; // internal id to be used in ringstream operations.
  EXNetworkClient client;
  if (client = throttleServer.available()) {

    // Work out the clientId for this client and recycle or create a new throttleClients entry as needed.
    // ESP32: recycling a client happens only once per client connect.
    // STM32: recycling a client happens every incoming packet.
        
    for (clientId = 0; clientId < throttleClients.size(); clientId++) {
      if (throttleClients[clientId].recycle(client)) {
        #ifdef ARDUINO_ARCH_ESP32
        // ESP32: recycling a client that has already been accepted.
        DIAG(F("Recycle client %d %s:%d"), clientId,
             client.remoteIP().toString().c_str(), client.remotePort());
        #endif
        //STM32: recycling a client happens every incoming packet.
        break;
      }
    }
    if (clientId >= throttleClients.size()) {
      exNetworkClient networkClient(client);
      throttleClients.push_back(networkClient);
      DIAG(F("New client %d, %s:%d"), clientId,
           client.remoteIP().toString().c_str(), client.remotePort());
    }
  }

  // Loop over all connected clients. This removes inactive clients as a side effect.
  for (clientId = 0; clientId < throttleClients.size(); clientId++) {
    if (throttleClients[clientId].active(clientId)) {
      auto len = throttleClients[clientId].client.available();
      if (len > 0) {
        byte cmd[len + 1];
        for (int i = 0; i < len; i++) {
          cmd[i] = throttleClients[clientId].client.read();
        }
        cmd[len] = 0;
        CommandDistributor::parse(clientId, cmd, outboundRing);
      }
    }
  }

  // Poll and process UDP packets synchronously from the main loop.
  while (udpThrottleRx.parsePacket() > 0) {
    processUdpPacket(udpThrottleRx, IP_PORT);
  }
  
  while (udpNodeRx.parsePacket() > 0) {
    processUdpPacket(udpNodeRx, NODE_PORT);
  }

  WiThrottle::loop(outboundRing); // withrottle may need to broadcast changes 

  // Send the next queued outbound message.
  auto readValue = outboundRing->read();
  if (readValue >= 0) {
    clientId = readValue;
    bool useWebsocket = clientId & Websockets::WEBSOCK_CLIENT_MARKER;
    clientId &= ~Websockets::WEBSOCK_CLIENT_MARKER;

    int count = outboundRing->count();
    auto wsHeaderLen = useWebsocket ? Websockets::getOutboundHeaderSize(count) : 0;
    byte buffer[wsHeaderLen + count + 1];
    if (useWebsocket) Websockets::fillOutboundHeader(count, buffer);
    for (int i = 0; i < count; i++) {
      int c = outboundRing->read();
      if (!c) {
        DIAG(F("Ringread fail at %d"), i);
        break;
      }
      if (useWebsocket && c == '\n') c = '\r';
      buffer[i + wsHeaderLen] = (char)c;
    }
    buffer[wsHeaderLen + count] = '\0';

    if (clientId < throttleClients.size()) {
      if (throttleClients[clientId].active(clientId)) {
        if (Diag::WIFI) {
          DIAG(F("SEND%S %d:%s"), useWebsocket ? F("ws") : F(""), clientId,
               buffer + wsHeaderLen);
        }
        throttleClients[clientId].client.write(buffer, count + wsHeaderLen);
      } else {
        DIAG(F("Unsent(%d): %s"), clientId, buffer + wsHeaderLen);
      }
    } else {
      DIAG(F("Non existent client %d has message: %s"), clientId, buffer + wsHeaderLen);
    }
  }

  #ifdef ARDUINO_ARCH_ESP32
    EthernetOTA.loop();
  #endif
}


void EXNetwork::teardown() {
  // Stop all locos. This broadcasts speed 1 (estop) and sets all reminders to speed 1.
  DCC::setThrottle(0, 1, 1);

  // Terminate all client connections.
  while (!throttleClients.empty()) {
    // pop_back() invokes the destructor, which stops the underlying TCP connection.
    throttleClients.pop_back();
  }

  webServer.end();
  throttleServer.end();

  _SHIM_::teardown();
}

bool EXNetwork::sendUDP(const IPAddress &ip, uint16_t port, const uint8_t *data, size_t len) {
  return isUp() && udpTx.beginPacket(ip, port) && udpTx.write(data, len) && udpTx.endPacket();
}
