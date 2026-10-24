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

#ifndef ETHERNET_OTA_H
#define ETHERNET_OTA_H

#include "defines.h"

#if defined(ARDUINO_ARCH_ESP32)

#include <Arduino.h>
#include <functional>
#include "EXNetwork.h"
using OTAUDP = EXNetworkUDP;
using OTAClient = EXNetworkClient;

class EthernetOTAClass {
public:
  EthernetOTAClass();

  EthernetOTAClass& setHostname(const char* hostname);
  EthernetOTAClass& setPassword(const char* password);
  EthernetOTAClass& onStart(std::function<void(void)> callback);
  EthernetOTAClass& onEnd(std::function<void(void)> callback);
  EthernetOTAClass& onError(std::function<void(int)> callback);
  template<class NetworkShim>
  void begin() {
    begin(NetworkShim::addService, NetworkShim::addServiceTxt);
  }
  void loop();
  void networkRestored();

private:
  using AddServiceCallback = void (*)(const char*, const char*, uint16_t);
  using AddServiceTxtCallback = void (*)(const char*, const char*, const char*, const char*);

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

  void configure();
  void begin(AddServiceCallback addService, AddServiceTxtCallback addServiceTxt);
  void receiveInvitation();
  void receiveAuthentication();
  void runUpdate();
  void sendResponse(const char* response);
};

extern EthernetOTAClass EthernetOTA;

#endif
#endif