#include "EthernetOTA.h"

#if defined(ARDUINO_ARCH_ESP32) && (WIFI_ON || ETHERNET_ON)

#include <MD5Builder.h>
#include <Update.h>
#if !ETHERNET_ON
#include <ESPmDNS.h>
#endif

static constexpr uint16_t ETHERNET_OTA_PORT = 3232;
static constexpr int OTA_AUTH_ERROR = 0;
static constexpr int OTA_BEGIN_ERROR = 1;
static constexpr int OTA_CONNECT_ERROR = 2;
static constexpr int OTA_RECEIVE_ERROR = 3;
static constexpr int OTA_END_ERROR = 4;

EthernetOTAClass EthernetOTA;

static String md5(const String& value) {
  MD5Builder builder;
  builder.begin();
  builder.add(value);
  builder.calculate();
  return builder.toString();
}

EthernetOTAClass::EthernetOTAClass()
  : state(IDLE), initialized(false), command(U_FLASH), updatePort(0),
    updateSize(0), remoteIP(), remotePort(0), expectedMD5{} {}

EthernetOTAClass& EthernetOTAClass::setHostname(const char* value) {
  if (!initialized && value) hostname = value;
  return *this;
}

EthernetOTAClass& EthernetOTAClass::setPassword(const char* value) {
  if (!initialized && value) password = value;
  return *this;
}

EthernetOTAClass& EthernetOTAClass::onStart(std::function<void(void)> callback) {
  startCallback = callback;
  return *this;
}

EthernetOTAClass& EthernetOTAClass::onEnd(std::function<void(void)> callback) {
  endCallback = callback;
  return *this;
}

EthernetOTAClass& EthernetOTAClass::onError(std::function<void(int)> callback) {
  errorCallback = callback;
  return *this;
}

void EthernetOTAClass::begin() {
  if (!initialized) {
    udp.begin(ETHERNET_OTA_PORT);
  #if !ETHERNET_ON
    MDNS.enableArduino(ETHERNET_OTA_PORT, password.length() > 0);
  #endif
    initialized = true;
    state = IDLE;
  }
}

void EthernetOTAClass::sendResponse(const char* response) {
  udp.beginPacket(remoteIP, remotePort);
  udp.print(response);
  udp.endPacket();
}

void EthernetOTAClass::receiveInvitation() {
  char packet[80];
  int length = udp.read(packet, sizeof(packet) - 1);
  if (length <= 0) return;
  packet[length] = '\0';

  char md5Value[33];
  if (sscanf(packet, "%d %d %d %32s", &command, &updatePort,
             &updateSize, md5Value) != 4 ||
      (command != U_FLASH && command != U_SPIFFS) || updateSize <= 0) {
    return;
  }

  remoteIP = udp.remoteIP();
  remotePort = udp.remotePort();
  strncpy(expectedMD5, md5Value, sizeof(expectedMD5));
  expectedMD5[sizeof(expectedMD5) - 1] = '\0';

  if (password.length()) {
    nonce = md5(String(micros()));
    String response = "AUTH " + nonce;
    sendResponse(response.c_str());
    state = WAIT_AUTH;
  } else {
    sendResponse("OK");
    state = RUN_UPDATE;
  }
}

void EthernetOTAClass::receiveAuthentication() {
  char packet[100];
  int length = udp.read(packet, sizeof(packet) - 1);
  if (length <= 0) return;
  packet[length] = '\0';

  int authCommand;
  char clientNonce[33];
  char response[33];
  if (sscanf(packet, "%d %32s %32s", &authCommand, clientNonce, response) != 3 ||
      authCommand != U_AUTH) {
    state = IDLE;
    return;
  }

  String passMD5 = md5(password);
  String challenge = passMD5 + ":" + nonce + ":" + clientNonce;
  if (md5(challenge) != response) {
    sendResponse("Authentication Failed");
    if (errorCallback) errorCallback(OTA_AUTH_ERROR);
    state = IDLE;
    return;
  }

  sendResponse("OK");
  state = RUN_UPDATE;
}

void EthernetOTAClass::runUpdate() {
  OTAClient client;
  if (!client.connect(remoteIP, updatePort)) {
    if (errorCallback) errorCallback(OTA_CONNECT_ERROR);
    state = IDLE;
    return;
  }

  if (!Update.begin(updateSize, command) || !Update.setMD5(expectedMD5)) {
    if (errorCallback) errorCallback(OTA_BEGIN_ERROR);
    client.stop();
    state = IDLE;
    return;
  }

  if (startCallback) startCallback();
  uint32_t written = 0;
  uint32_t lastData = millis();
  uint8_t buffer[1460];

  while (!Update.isFinished() && client.connected()) {
    if (!client.available()) {
      if (millis() - lastData > 10000) {
        Update.abort();
        if (errorCallback) errorCallback(OTA_RECEIVE_ERROR);
        client.stop();
        state = IDLE;
        return;
      }
      delay(1);
      continue;
    }

    lastData = millis();
    size_t available = client.available();
    if (available > sizeof(buffer)) available = sizeof(buffer);
    size_t received = client.read(buffer, available);
    if (!received) continue;
    size_t stored = Update.write(buffer, received);
    if (stored != received) {
      Update.abort();
      if (errorCallback) errorCallback(OTA_RECEIVE_ERROR);
      client.stop();
      state = IDLE;
      return;
    }
    written += stored;
    client.print(String(stored));
  }

  if (Update.end() && written == static_cast<uint32_t>(updateSize)) {
    client.print("OK");
    client.stop();
    if (endCallback) endCallback();
    delay(100);
    ESP.restart();
  } else {
    if (errorCallback) errorCallback(OTA_END_ERROR);
    Update.printError(client);
    client.stop();
    state = IDLE;
  }
}

void EthernetOTAClass::handle() {
  if (!initialized) return;

  if (state == RUN_UPDATE) {
    runUpdate();
    return;
  }

  if (udp.parsePacket()) {
    if (state == WAIT_AUTH) receiveAuthentication();
    else receiveInvitation();
  }
}

#endif