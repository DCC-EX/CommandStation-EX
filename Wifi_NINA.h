/*
 *  © 2023 Paul M. Antoine
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

#ifndef WifiNINA_h
#define WifiNINA_h
// #include "FSH.h"
#include <Arduino.h>
// #include <SPI.h>
// #include <WifiNINA.h>

class WifiNINA
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
#endif //WifiNINA_h
