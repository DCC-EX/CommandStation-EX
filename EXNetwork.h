#ifndef EXNetwork_h
#define EXNetwork_h
#ifdef ARDUINO_ARCH_ESP32
  #include "WifiESP32.h"
  typedef WiFiClient EXNetworkClient;
  typedef WiFiServer EXNetworkServer;
  typedef AsyncUDP EXNetworkUDPRx;
  typedef WiFiUDP EXNetworkUDPTx;
  typedef AsyncUDPPacket EXNetworkUDPPacket;
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
  static void throttlePacketListener();
  static void nodePacketListener();
  #ifdef ARDUINO_ARCH_ESP32
  static void esp32AsyncPacketListener(EXNetworkUDPPacket &packet);
  #endif
  static void queueUdpInput(IPAddress remoteIP,int localPort,const uint8_t *buffer,int length);
};

#endif
