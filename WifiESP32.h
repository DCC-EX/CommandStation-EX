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

#include "FSH.h"

class WifiESP
{

public:
  static bool setup(const char *wifiESSID,
		    const char *wifiPassword,
		    const char *hostname,
		    const int port,
		    const byte channel,
			const bool forceAP);
  static void loop();
private:
};
#endif //WifiESP8266_h
#endif //ESP8266
