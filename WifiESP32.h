/*
 *  © 2021 Harald Barth
 *  © 2023 Nathan Kellenicki
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

#if defined(ARDUINO_ARCH_ESP32)
#ifndef WifiESP32_h
#define WifiESP32_h

#include <WiFi.h>
#include "FSH.h"

class WifiESP
{
public:
  static bool setup();
  static void loop();
private:
    static bool setupFromPreferences();
		static bool setupFromConfig(const char *wifiESSID,
		    const char *wifiPassword,
		    const byte channel,
	 		  const bool forceAP);
  static void teardown();
  static bool ConnectSTA(const char * SSid, const char * password);
  static bool ConnectAP(const char * SSid, const char * password, byte channel);
  static bool wifiUp;
  static WiFiServer *server;
  static int16_t wifiLed;
};
#endif //WifiESP32_h
#endif //ESP32
