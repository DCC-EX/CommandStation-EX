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

#include "EthernetOTA.h"

#if defined(ARDUINO_ARCH_ESP32)

#include <esp_system.h>
#include <mbedtls/md.h>
#include <mbedtls/pkcs5.h>
#include <mbedtls/version.h>
#include <Update.h>
#include "TrackManager.h"
#include "Display_Implementation.h"
#include "DCC.h"
#include "CommandDistributor.h"
#include "DIAG.h"
#include "EXNetworkPreferences.h"

static constexpr uint16_t ETHERNET_OTA_PORT = 3232;
static constexpr int OTA_AUTH_ERROR = 0;
static constexpr int OTA_BEGIN_ERROR = 1;
static constexpr int OTA_CONNECT_ERROR = 2;
static constexpr int OTA_RECEIVE_ERROR = 3;
static constexpr int OTA_END_ERROR = 4;

#define ETHERNET_OTA_STR(tok) tok
#define ETHERNET_OTA_BOARD ETHERNET_OTA_STR(ARDUINO_VARIANT)

EthernetOTAClass EthernetOTA;

static String hexString(const uint8_t* bytes, size_t length) {
  static const char digits[] = "0123456789abcdef";
  String result;
  result.reserve(length * 2);
  for (size_t i = 0; i < length; ++i) {
    result += digits[bytes[i] >> 4];
    result += digits[bytes[i] & 0x0f];
  }
  return result;
}

static bool sha256(const String& value, String& result) {
  uint8_t digest[32];
  const mbedtls_md_info_t* info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
  if (!info) {
    DIAG(F("EthernetOTA SHA256 unavailable"));
    return false;
  }
  int error = mbedtls_md(info, reinterpret_cast<const uint8_t*>(value.c_str()),
                        value.length(), digest);
  if (error) {
    DIAG(F("EthernetOTA SHA256 failed: %d"), error);
    return false;
  }
  result = hexString(digest, sizeof(digest));
  return true;
}

static bool authenticationResponse(const String& password, const String& nonce,
                                   const String& clientNonce, String& response) {
  String passwordHash;
  if (!sha256(password, passwordHash)) return false;
  String salt = nonce + ":" + clientNonce;
  uint8_t key[32];
  // espota.py uses the hexadecimal SHA256 password hash as the PBKDF2 password.
#if MBEDTLS_VERSION_MAJOR >= 3
  int error = mbedtls_pkcs5_pbkdf2_hmac_ext(
    MBEDTLS_MD_SHA256,
    reinterpret_cast<const uint8_t*>(passwordHash.c_str()), passwordHash.length(),
    reinterpret_cast<const uint8_t*>(salt.c_str()), salt.length(), 10000, sizeof(key), key);
#else
  mbedtls_md_context_t context;
  mbedtls_md_init(&context);
  int error = mbedtls_md_setup(&context, mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), 1);
  if (!error) {
    error = mbedtls_pkcs5_pbkdf2_hmac(
      &context,
      reinterpret_cast<const uint8_t*>(passwordHash.c_str()), passwordHash.length(),
      reinterpret_cast<const uint8_t*>(salt.c_str()), salt.length(), 10000, sizeof(key), key);
  }
  mbedtls_md_free(&context);
#endif
  if (error) {
    DIAG(F("EthernetOTA PBKDF2 failed: %d"), error);
    return false;
  }
  return sha256(hexString(key, sizeof(key)) + ":" + nonce + ":" + clientNonce, response);
}

EthernetOTAClass::EthernetOTAClass()
  : state(IDLE), initialized(false), command(U_FLASH), updatePort(0),
    updateSize(0), remoteIP(), remotePort(0), expectedMD5{}, authStarted(0) {}

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

void EthernetOTAClass::configure() {
  setHostname(
    EXNetworkPreferences::getHostName()
  );
#ifdef OTA_AUTH
  setPassword(OTA_AUTH);
#endif // OTA_AUTH
}

void EthernetOTAClass::begin(AddServiceCallback addService, AddServiceTxtCallback addServiceTxt) {
  if (!initialized) {
    configure();

    addService("_arduino", "_tcp", ETHERNET_OTA_PORT);
    addServiceTxt("_arduino", "_tcp", "board", ETHERNET_OTA_BOARD);
    addServiceTxt("_arduino", "_tcp", "tcp_check", "no");
    addServiceTxt("_arduino", "_tcp", "ssh_upload", "no");
    addServiceTxt("_arduino", "_tcp", "auth_upload", password.length() ? "yes" : "no");

    const uint8_t beginResult = udp.begin(ETHERNET_OTA_PORT) ? 0 : 1;
    DIAG(F("EthernetOTA UDP begin port %d result %d"), ETHERNET_OTA_PORT, beginResult);
    initialized = true;
    state = IDLE;

  }

}

void EthernetOTAClass::networkRestored() {
  udp.stop();
  if (!udp.begin(ETHERNET_OTA_PORT)) {
    DIAG(F("EthernetOTA UDP restart failed on port %d"), ETHERNET_OTA_PORT);
  }
  state = IDLE;
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
    uint8_t randomBytes[32];
    esp_fill_random(randomBytes, sizeof(randomBytes));
    nonce = hexString(randomBytes, sizeof(randomBytes));
    String response = "AUTH " + nonce;
    sendResponse(response.c_str());
    authStarted = millis();
    state = WAIT_AUTH;
  } else {
    sendResponse("OK");
    state = RUN_UPDATE;
  }
}

void EthernetOTAClass::receiveAuthentication() {
  if (udp.remoteIP() != remoteIP) {
    udp.flush();
    DIAG(F("EthernetOTA ignored authentication from unexpected host"));
    return;
  }
  // espota.py opens a new UDP socket for authentication.
  remotePort = udp.remotePort();
  char packet[160];
  int length = udp.read(packet, sizeof(packet) - 1);
  if (length <= 0) return;
  packet[length] = '\0';

  int authCommand;
  char clientNonce[65];
  char response[65];
  char extra;
  if (sscanf(packet, "%d %64s %64s %c", &authCommand, clientNonce, response, &extra) != 3 ||
      authCommand != U_AUTH || strlen(clientNonce) != 64 || strlen(response) != 64 ||
      strspn(clientNonce, "0123456789abcdef") != 64 ||
      strspn(response, "0123456789abcdef") != 64) {
    DIAG(F("EthernetOTA invalid authentication packet"));
    sendResponse("Authentication Failed");
    if (errorCallback) errorCallback(OTA_AUTH_ERROR);
    state = IDLE;
    return;
  }

  String expectedResponse;
  if (!authenticationResponse(password, nonce, clientNonce, expectedResponse) ||
      expectedResponse != response) {
    DIAG(F("EthernetOTA authentication failed"));
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

void EthernetOTAClass::loop() {
  if (!initialized) return;

  static bool otaInitialised = false;
  if (!otaInitialised) {
    EthernetOTA.onStart([]() {
      DCC::setThrottle(0,1,1);
      TrackManager::setMainPower(POWERMODE::OFF);
      TrackManager::setProgPower(POWERMODE::OFF);
      CommandDistributor::broadcastPower();
      DISPLAY_START (
        LCD(0,F("OTA update"));
        LCD(1,F("In progress..."));
      );
    });
    EthernetOTA.onEnd([]() {
      DISPLAY_START (
        LCD(0,F("OTA update"));
        LCD(1,F("Complete"));
      );
    });
    EthernetOTA.onError([](int error) {
      DIAG(F("EthernetOTA update error: %d"), error);
      DISPLAY_START (
        LCD(0,F("OTA update"));
        LCD(1,F("Error: %d"), error);
      );
    });
    otaInitialised = true;
  }

  if (state == RUN_UPDATE) {
    runUpdate();
    return;
  }

  if (state == WAIT_AUTH && millis() - authStarted > 10000) {
    DIAG(F("EthernetOTA authentication timed out"));
    if (errorCallback) errorCallback(OTA_AUTH_ERROR);
    state = IDLE;
  }

  if (udp.parsePacket()) {
    if (state == WAIT_AUTH) receiveAuthentication();
    else receiveInvitation();
  }
}

#endif