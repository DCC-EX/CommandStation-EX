#ifndef EXNetwork_h
#define EXNetwork_h
#ifdef ARDUINO_ARCH_ESP32
  #include "WifiESP32.h"
  #include <WiFiUdp.h>
  typedef WiFiClient EXNetworkClient;
  typedef WiFiServer EXNetworkServer;
  typedef WiFiUDP EXNetworkUDPRx;
  typedef WiFiUDP EXNetworkUDPTx;
  #define _SHIM_ WifiESP
  #else 
  #include "EthernetInterface.h"
  typedef EthernetClient EXNetworkClient;
  typedef EthernetServer EXNetworkServer;
  typedef EthernetUDP  EXNetworkUDPRx;
  typedef EthernetUDP  EXNetworkUDPTx;
  #define _SHIM_ EthernetInterface
  #endif

class EXNetwork {
public:
  static void setup();
  static void teardown();
  static void loop();
  static bool isUp();
  static void udpMulticast(const char *buffer);
  static void udpNodeMulticast(const char *buffer);
  static EXNetworkClient acceptWebInput();
  
private:
  static bool sendUDP(const IPAddress &ip, uint16_t port, const uint8_t *data, size_t len);
  static void processUdpPacket(EXNetworkUDPRx &udp, uint16_t localPort);
};

#endif
