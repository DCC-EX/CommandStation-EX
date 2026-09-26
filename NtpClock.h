/*
 *  © 2026 Kristian Kalweit
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

/*
 * Feeds the DCC-EX fast clock from the ESP32 system clock, which is kept
 * accurate by SNTP over the existing WiFi connection.
 *
 * The ESP32 has an RTC peripheral, but without a backup battery and without
 * an external 32kHz crystal it drifts and is lost on every power cycle. NTP
 * fixes that: the time is fetched once the WiFi is up and then resynced
 * automatically by the SNTP client in the background.
 *
 * There is no setup() - the WiFi is not up when the sketch starts, so
 * configuration happens lazily on the first loop() calls that see a
 * connection. Enable with NTP_CLOCK in config.h, see config.example.h.
 */

#ifndef NtpClock_h
#define NtpClock_h

#include "defines.h"

#if defined(ARDUINO_ARCH_ESP32) && defined(NTP_CLOCK)

class NtpClock {
public:
  static void loop();

  // True once a valid time has been received from NTP. Until then the model
  // clock reads 0 (midnight) and must not be trusted. Lets EX-RAIL wait for
  // the clock before acting on it, e.g. to pick a scene at startup.
  static bool isSynced() { return synced; }

private:
  static bool configured;   // configTzTime() has been called
  static bool synced;       // a plausible time has been seen
  static unsigned long lastPoll;
};

#endif // ARDUINO_ARCH_ESP32 && NTP_CLOCK
#endif // NtpClock_h
