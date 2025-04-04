
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

void ESPPrettyOTA::setup() {
  // Print IP address
  DIAG(F("PrettyOTA can be accessed at: http://%s/update\n"), WiFi.localIP().toString().c_str());

  // Initialize PrettyOTA
  OTAUpdates.Begin(&webServer);

  if (OTA_USERNAME != "" && OTA_PASSWORD != "") {
    OTAUpdates.SetAuthenticationDetails(OTA_USERNAME, OTA_PASSWORD, OTA_PASSWORD_IS_MD5_HASH);
  }

  // Set firmware version to 1.0.0
  OTAUpdates.OverwriteAppVersion(VERSION);

  // Set current build time and date
  PRETTY_OTA_SET_CURRENT_BUILD_TIME_AND_DATE();

  // set custom callbacks
  OTAUpdates.OnStart([this](NSPrettyOTA::UPDATE_MODE updateMode) { this->OnOTAStart(updateMode); });
  OTAUpdates.OnProgress([this](uint32_t currentSize, uint32_t totalSize) { this->OnOTAProgress(currentSize, totalSize); });
  OTAUpdates.OnEnd([this](bool successful) { this->OnOTAEnd(successful); });

  // Start web server
  webServer.begin();
}

void ESPPrettyOTA::OnOTAStart(NSPrettyOTA::UPDATE_MODE updateMode) {
  DIAG(F("OTA update started\n"));

  DCC::setThrottle(0, 1, 1);  // taken from ! Stop

  if (updateMode == NSPrettyOTA::UPDATE_MODE::FIRMWARE) {
    DIAG(F("Mode: Firmware\n"));
  } else if (updateMode == NSPrettyOTA::UPDATE_MODE::FILESYSTEM) {
    DIAG(F("Mode: Filesystem\n"));
  }
}

// Gets called while update is running
// currentSize: Number of bytes already processed
// totalSize: Total size of new firmware in bytes
void ESPPrettyOTA::OnOTAProgress(uint32_t currentSize, uint32_t totalSize) {
  DIAG(F("OTA Progress Current: %u bytes, Total: %u bytes\n"), currentSize, totalSize);
}

// Gets called when update finishes
void ESPPrettyOTA::OnOTAEnd(bool successful) {
  if (successful) {
    DIAG(F("OTA update finished successfully\n"));
  } else {
    DIAG(F("OTA update failed\n"));
  }
}

#endif
