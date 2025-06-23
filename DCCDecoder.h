/*
 *  © 2025 Harald Barth
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
#include <Arduino.h>
#include "DCCPacket.h"

class DCCDecoder {
public:
  static bool parse(DCCPacket &p);
  static inline void onoff(bool on) {active = on;};
private:
  static bool active;
};
#endif // ARDUINO_ARCH_ESP32
