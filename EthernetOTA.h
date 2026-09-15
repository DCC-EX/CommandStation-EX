#ifndef ETHERNET_OTA_H
#define ETHERNET_OTA_H

#include "defines.h"

#if defined(ARDUINO_ARCH_ESP32) && (WIFI_ON || ETHERNET_ON)

#include <Arduino.h>
#include <functional>
#if ETHERNET_ON
#include "EthernetInterface.h"
using OTAUDP = EthernetUDP;
using OTAClient = EthernetClient;
#else
#include <WiFi.h>
#include <WiFiUdp.h>
using OTAUDP = WiFiUDP;
using OTAClient = WiFiClient;
#endif

class EthernetOTAClass {
public:
  EthernetOTAClass();

  EthernetOTAClass& setHostname(const char* hostname);
  EthernetOTAClass& setPassword(const char* password);
  EthernetOTAClass& onStart(std::function<void(void)> callback);
  EthernetOTAClass& onEnd(std::function<void(void)> callback);
  EthernetOTAClass& onError(std::function<void(int)> callback);
  void begin();
  void handle();

private:
  enum State { IDLE, WAIT_AUTH, RUN_UPDATE };

  State state;
  bool initialized;
  int command;
  int updatePort;
  int updateSize;
  IPAddress remoteIP;
  uint16_t remotePort;
  char expectedMD5[33];
  String hostname;
  String password;
  String nonce;
  std::function<void(void)> startCallback;
  std::function<void(void)> endCallback;
  std::function<void(int)> errorCallback;
  OTAUDP udp;

  void receiveInvitation();
  void receiveAuthentication();
  void runUpdate();
  void sendResponse(const char* response);
};

extern EthernetOTAClass EthernetOTA;

#endif
#endif