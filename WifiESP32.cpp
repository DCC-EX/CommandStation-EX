/*
    © 2021, Harald Barth.

    This file is part of CommandStation-EX

    This is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    It is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with CommandStation.  If not, see <https://www.gnu.org/licenses/>.
*/

#include "defines.h"
#if defined(ARDUINO_ARCH_ESP32)
#include <WiFi.h>
#include "WifiESP32.h"
#include "DIAG.h"
#include "RingStream.h"
#include "CommandDistributor.h"

bool WifiESP::setup(const char *SSid,
                    const char *password,
                    const char *hostname,
                    int port,
                    const byte channel) {
  return false;
}

void WifiESP::loop() {

}
#endif //ESP32
