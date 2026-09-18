#ifndef NetworkInterface_h
#define NetworkInterface_h
#ifdef ARDUINO_ARCH_ESP32
  #include "WifiESP32.h"
  typedef WiFiClient NetworkInterfaceClient;
  typedef WiFiServer NetworkInterfaceServer;
  typedef AsyncUDP NetworkInterfaceUDPRx;
  typedef WiFiUDP NetworkInterfaceUDPTx;
  typedef AsyncUDPPacket NetworkInterfaceUDPPacket;
  #define _SHIM_ WifiESP
  #else 
  #include "EthernetInterface.h"
  typedef EthernetClient NetworkInterfaceClient;
  typedef EthernetServer NetworkInterfaceServer;
  typedef EthernetUDP  NetworkInterfaceUDPRx;
  typedef EthernetUDP  NetworkInterfaceUDPTx;
  #define _SHIM_ EthernetInterface
  #endif

class NetworkInterface {
public:
  static void setup();
  static void teardown();
  static void loop();
  static bool isUp();
  static void udpMulticast(const char *buffer);
  static void udpNodeMulticast(const char *buffer);
  static NetworkInterfaceClient acceptWebInput();
  
private:
  static bool sendUDP(const IPAddress &ip, uint16_t port, const uint8_t *data, size_t len);
  static void throttlePacketListener();
  static void nodePacketListener();
  #ifdef ARDUINO_ARCH_ESP32
  static void esp32AsyncPacketListener(NetworkInterfaceUDPPacket &packet);
  #endif
  static void queueUdpInput(IPAddress remoteIP,int localPort,const uint8_t *buffer,int length);
};

#endif
