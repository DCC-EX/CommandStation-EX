#include "NetworkInterface.h"
#include "DIAG.h"
#include "CommandDistributor.h"
#include "NodeManager.h"
#include "RingStream.h"
#include <vector>
#include "WiThrottle.h"
#include "Websockets.h"


#ifndef NODE_GROUP
#define NODE_GROUP 254
#endif
#ifndef IP_PORT
#define IP_PORT 2560
#endif

// Common points of interaction for network interfaces
NetworkInterfaceServer webServer(80);
NetworkInterfaceServer throttleServer(IP_PORT);

NetworkInterfaceUDPRx udpThrottleRx;
NetworkInterfaceUDPRx udpNodeRx;
NetworkInterfaceUDPTx udpTx;

IPAddress udpBroadcastIP;
constexpr uint16_t UDP_COMMAND_MAX = 255;    // max inbound command payload (fits one DCC-EX command)
constexpr uint16_t UDP_RESPONSE_MAX  = 1472; // max outbound payload (Ethernet MTU 1500 - 20 IP - 8 UDP)
constexpr uint8_t UDP_COMMAND_QUEUE_DEPTH = 64;
constexpr uint16_t NODE_PORT = IP_PORT + 1;
const IPAddress nodeMulticastIP = {239, 255, 254, NODE_GROUP};
IPAddress udpMulticastIP = {239, 255, 255, 0 /* will become WiFi.localIP()[3] */};
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
  exNetworkClient(NetworkInterfaceClient c) {
    client = c;
    inUse = true;
  }

  bool active(byte clientId) {
    if (!inUse) return false;
    if (!NetworkInterface::isUp()) {
      DIAG(F("Remove client %d"), clientId);
      CommandDistributor::forget(clientId);
      client.stop();
      inUse = false;
      return false;
    }
    return true;
  }
  bool recycle(NetworkInterfaceClient c) {
    if (client == c) {
      if (inUse) DIAG(F("WARNING: Duplicate"));
      else DIAG(F("Returning"));
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

  NetworkInterfaceClient client;

private:
  bool inUse;
};
static std::vector<exNetworkClient> throttleClients; // A list to hold all clients.

struct UdpCommand {
  uint16_t len;
  uint32_t receivedMillis;
  IPAddress remoteIP;  // Originator IP.
  uint16_t localPort;  // Arrival port.
  byte data[UDP_COMMAND_MAX + 1]; // +1 for null terminator.
};
static UdpCommand udpCommandQueue[UDP_COMMAND_QUEUE_DEPTH];
static uint8_t udpCommandQueueHead = 0;
static uint8_t udpCommandQueueTail = 0;
static uint8_t udpCommandQueueCount = 0;
static uint32_t udpCommandDropCount = 0;


void NetworkInterface::setup() {
  // Initialize the UDP command queue for handling incoming UDP commands.
  udpCommandQueueHead = 0;
  udpCommandQueueTail = 0;
  udpCommandQueueCount = 0;
  udpCommandDropCount = 0;

  _SHIM_::setup();
  auto ipaddress = _SHIM_::getIPAddress();
  udpMulticastIP[3] = ipaddress[3];

  // Socket server for old style throttle connections.
  throttleServer.begin(IP_PORT);
  DIAG(F("throttleServer started on port %d"), IP_PORT);

  // Web server for browser interface.
  webServer.begin(80);
  DIAG(F("webServer started on port 80"));

  bool nodeFail = false;

  #ifdef ARDUINO_ARCH_STM32
  // Receive throttle traffic via UDP multicast.
  if (udpThrottleRx.beginMulticast(udpMulticastIP, IP_PORT)) {
    udpThrottleRx.onDataArrival(throttlePacketListener);
    DIAG(F("udpThrottleRx started on %s:%d"),
         udpMulticastIP.toString().c_str(), IP_PORT);
  }
  // Receive node traffic via UDP multicast.
  if (udpNodeRx.beginMulticast(nodeMulticastIP, NODE_PORT)) {
    udpNodeRx.onDataArrival(nodePacketListener);
    DIAG(F("udpNodeRx started on %s:%d"),
         nodeMulticastIP.toString().c_str(), NODE_PORT);
  } else {
    nodeFail = true;
  }
  #endif

  #ifdef ARDUINO_ARCH_ESP32
  // Receive throttle traffic via UDP multicast.
  if (udpThrottleRx.listenMulticast(udpMulticastIP, IP_PORT)) {
    udpThrottleRx.onPacket(esp32AsyncPacketListener);
    DIAG(F("udpThrottleRx started on %s:%d"),
         udpMulticastIP.toString().c_str(), IP_PORT);
  }
  // Receive node traffic via UDP multicast.
  if (udpNodeRx.listenMulticast(nodeMulticastIP, NODE_PORT)) {
    udpNodeRx.onPacket(esp32AsyncPacketListener);
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
  _SHIM_::addServiceTxt("dcc-ex", "udp", "group", udpMulticastIP.toString().c_str());
  _SHIM_::addServiceTxt("dcc-ex", "udp", "port", String(IP_PORT).c_str());
}

bool NetworkInterface::isUp() { return _SHIM_::isUp(); }

void NetworkInterface::udpMulticast(const char *buffer) {
  if (buffer == NULL || !isUp()) return;
  int count = strlen(buffer);
  if (count <= 0 || count > UDP_RESPONSE_MAX) {
    DIAG(F("udpMulticast: Invalid count %d, %s"), count, buffer);
    return;
  }

  // Regardless of the clientId, we can send it via UDP multicast.
  if (!sendUDP(udpMulticastIP, IP_PORT, (const uint8_t *)buffer, count)) {
    DIAG(F("udpSend Multicast failed"));
  }

  // Also unicast to clients discovered via <#> probe packets.
  for (const auto &clientIp : udpDiscoveryClients) {
    if (!sendUDP(clientIp, IP_PORT, (const uint8_t *)buffer, count)) {
      DIAG(F("udpSend unicast failed to %s:%d"), clientIp.toString().c_str(), IP_PORT);
    }
  }
}

void NetworkInterface::udpNodeMulticast(const char *buffer) {
  if (buffer == NULL || !isUp()) return;
  if (!sendUDP(nodeMulticastIP, NODE_PORT,
                       (const uint8_t *)buffer, strlen(buffer))) {
    DIAG(F("udpNodeMulticast failed"));
  }
}

NetworkInterfaceClient NetworkInterface::acceptWebInput() {
  return webServer.available();
}

void NetworkInterface::queueUdpInput(IPAddress remoteIP, int localPort,
                                     const uint8_t *buffer, int length) {
  if (buffer == NULL) return;
  if (length <= 2 || length > UDP_COMMAND_MAX) {
    DIAG(F("queueUdpInput: Invalid length %d, %s"), length, buffer);
    return;
  }
  if (udpCommandQueueCount >= UDP_COMMAND_QUEUE_DEPTH) {
    udpCommandDropCount++;
    if ((udpCommandDropCount & 0x3F) == 1) {
      DIAG(F("queueUdpInput: command queue full, dropped=%d"), udpCommandDropCount);
    }
    return;
  }

  UdpCommand cmd;
  cmd.len = length;
  cmd.receivedMillis = millis();
  cmd.remoteIP = remoteIP;
  cmd.localPort = localPort;
  memcpy(cmd.data, buffer, cmd.len);
  cmd.data[cmd.len] = 0;

  udpCommandQueue[udpCommandQueueTail] = cmd;
  udpCommandQueueTail = (udpCommandQueueTail + 1) % UDP_COMMAND_QUEUE_DEPTH;
  udpCommandQueueCount++;
}

#ifdef ARDUINO_ARCH_ESP32
// NOTE: This function is called asynchronously by the Wi-Fi code.
void NetworkInterface::esp32AsyncPacketListener(NetworkInterfaceUDPPacket &packet) {
  queueUdpInput(packet.remoteIP(), packet.localPort(), packet.data(), packet.length());
}
#else
void NetworkInterface::throttlePacketListener() {
  auto packetSize = udpThrottleRx.parsePacket();
  if (packetSize < 2) return;

  byte buffer[UDP_COMMAND_MAX];
  auto length = udpThrottleRx.read(buffer, sizeof(buffer));
  queueUdpInput(udpThrottleRx.remoteIP(), IP_PORT, buffer, length);
}

void NetworkInterface::nodePacketListener() {
  auto packetSize = udpNodeRx.parsePacket();
  if (packetSize < 2) return;

  byte buffer[UDP_COMMAND_MAX];
  auto length = udpNodeRx.read(buffer, sizeof(buffer));
  queueUdpInput(udpNodeRx.remoteIP(), NODE_PORT, buffer, length);
}
#endif

void NetworkInterface::loop() {
  _SHIM_::loop(); // Wi-Fi/Ethernet continuous support.
    if (!isUp()) return;
  size_t clientId; // Temporary loop variable.

  // Track new socket clients.
  NetworkInterfaceClient client;
  while (client = throttleServer.available()) {
    for (clientId = 0; clientId < throttleClients.size(); clientId++) {
      if (throttleClients[clientId].recycle(client)) {
        DIAG(F("Recycle client %d %s:%d"), clientId,
             client.remoteIP().toString().c_str(), client.remotePort());
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

  // Drain queued UDP commands collected by the async callback.
  if (udpCommandQueueCount > 0
#ifndef ARDUINO_ARCH_ESP32
      || udpThrottleRx.parsePacket() > 0 || udpNodeRx.parsePacket() > 0
#endif
  ) {
#ifndef ARDUINO_ARCH_ESP32
    do {
      throttlePacketListener();
    } while (udpThrottleRx.parsePacket() > 0);
    do {
      nodePacketListener();
    } while (udpNodeRx.parsePacket() > 0);
#endif

    UdpCommand cmd;
    while (udpCommandQueueCount > 0) {
      cmd = udpCommandQueue[udpCommandQueueHead];
      udpCommandQueueHead = (udpCommandQueueHead + 1) % UDP_COMMAND_QUEUE_DEPTH;
      udpCommandQueueCount--;

      if (cmd.localPort == NODE_PORT) {
        // This is a node multicast; no response is required.
        NodeManager::parse(cmd.data);
        return;
      }

      StringBuffer response(UDP_RESPONSE_MAX);
      if (cmd.len >= 3 && cmd.data[0] == '<' && cmd.data[1] == '#' && cmd.data[2] == '>') {
        rememberUdpDiscoveryClient(cmd.remoteIP);
      }
      DCCEXParser::parse(&response, cmd.data);
      if (Diag::WIFI) {
        DIAG(F("UDP Command: %s>, Response: %s"), cmd.data, response.getString());
      }
      if (response.getLength() > 0) {
        sendUDP(cmd.remoteIP, IP_PORT,
                        (const byte *)response.getString(), response.getLength());
      }
    }
  }

  WiThrottle::loop(outboundRing);

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
}


void NetworkInterface::teardown() {
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
  udpCommandQueueHead = 0;
  udpCommandQueueTail = 0;
  udpCommandQueueCount = 0;
}

bool NetworkInterface::sendUDP(const IPAddress &ip, uint16_t port, const uint8_t *data, size_t len) {
  if (!isUp()) return false;
  #ifdef ARDUINO_ARCH_STM32
    if (ip[0]==239)
      return udpTx.beginMulticast(ip, port) && udpTx.write(data, len) && udpTx.endPacket();
  #endif
  return udpTx.beginPacket(ip, port) && udpTx.write(data, len) && udpTx.endPacket();
}