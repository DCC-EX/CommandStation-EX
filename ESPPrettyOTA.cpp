
/*
 *  © 2025 Mathew Winters
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

#ifdef ARDUINO_ARCH_ESP32
#include "ESPPrettyOTA.h"

#include "DCC.h"
#include "DIAG.h"
#include "config.h"
#include "version.h"

#ifndef OTA_USERNAME
#define OTA_USERNAME ""
#endif
#ifndef OTA_PASSWORD
#define OTA_PASSWORD ""
#endif
#ifndef OTA_PASSWORD_IS_MD5_HASH
#define OTA_PASSWORD_IS_MD5_HASH false
#endif

// Custom stream to wrap output from PrettyOTA to DIAG
class CustomDiagStream : public Stream {
 public:
  CustomDiagStream() {}

  size_t write(uint8_t c) override {
    if (!_started) {
      _started = true;
    }

    if (c == '\n' || c == '\r') {
      _started = false;
      DIAG(buildOutput.c_str());
      buildOutput = "";
    } else {
      buildOutput += (char)c;
    }
    return 1;
  }

  int available() override { return 0; }

  int read() override { return 0; }

  int peek() override { return 0; }

  void flush() override {}

 private:
  bool _started = false;
  String buildOutput;
};

void ESPPrettyOTA::setup() {
  // Print IP address
  DIAG(F("PrettyOTA can be accessed at: http://%s/update"),
       WiFi.localIP().toString().c_str());

  // Initialize PrettyOTA
  OTAUpdates.SetSerialOutputStream(new CustomDiagStream());
  OTAUpdates.Begin(&webServer);

  if (OTA_USERNAME != "" && OTA_PASSWORD != "") {
    OTAUpdates.SetAuthenticationDetails(OTA_USERNAME, OTA_PASSWORD, OTA_PASSWORD_IS_MD5_HASH);
  }

  // Set firmware version to VERSION
  OTAUpdates.OverwriteAppVersion(VERSION);

  // Set current build time and date
  PRETTY_OTA_SET_CURRENT_BUILD_TIME_AND_DATE();

  // set custom callbacks
  OTAUpdates.OnStart([this](NSPrettyOTA::UPDATE_MODE updateMode) {
    this->OnOTAStart(updateMode);
  });
  OTAUpdates.OnProgress([this](uint32_t currentSize, uint32_t totalSize) {
    this->OnOTAProgress(currentSize, totalSize);
  });

  // Start web server
  webServer.begin();
}

// we wrap the default output to be compatible with
// the DIAG output and print much less.
void ESPPrettyOTA::OnOTAStart(NSPrettyOTA::UPDATE_MODE updateMode) {
  DIAG(F("OTA update started - stopping DCC-EX"));
  DCC::setThrottle(0, 1, 1);  // taken from ! Stop
}

// Gets called while update is running
// currentSize: Number of bytes already processed
// totalSize: Total size of new firmware in bytes
void ESPPrettyOTA::OnOTAProgress(uint32_t currentSize, uint32_t totalSize) {
  static float lastPercentage = 0.0f;
  const float percentage = 100.0f * static_cast<float>(currentSize) / static_cast<float>(totalSize);
  const uint8_t numBarsToShow = static_cast<uint8_t>(percentage / 3.3333f);

  if (percentage - lastPercentage >= 1.0f) {
    DIAG(F("OTA Progress: %02u%%"), static_cast<uint8_t>(percentage));
    lastPercentage = percentage;
  }
}

#endif
