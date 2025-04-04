
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

#ifndef PrettyOTA_h
#define PrettyOTA_h

// installs PrettyOTA https://github.com/LostInCompilation/PrettyOTA

#ifdef ARDUINO_ARCH_ESP32
#include <PrettyOTA.h>

class ESPPrettyOTA {
 private:
  PrettyOTA OTAUpdates;
  AsyncWebServer webServer;

 public:
  ESPPrettyOTA() : webServer(80) {}
  void setup();

  void OnOTAStart(NSPrettyOTA::UPDATE_MODE updateMode);
  void OnOTAProgress(uint32_t currentSize, uint32_t totalSize);
  void OnOTAEnd(bool successful);
};

#endif
#endif