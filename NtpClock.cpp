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

#include "NtpClock.h"

#if defined(ARDUINO_ARCH_ESP32) && defined(NTP_CLOCK)

#include <WiFi.h>
#include <time.h>
#include "CommandDistributor.h"
#include "DIAG.h"

#ifndef NTP_CLOCK_SERVER
#define NTP_CLOCK_SERVER "pool.ntp.org"
#endif

// POSIX timezone string. The default is Central European Time including the
// EU daylight saving rules, so the model clock follows the wall clock.
#ifndef NTP_CLOCK_TZ
#define NTP_CLOCK_TZ "CET-1CEST,M3.5.0,M10.5.0/3"
#endif

// How many model minutes pass per real minute. 1 = model time is wall time.
#ifndef NTP_CLOCK_RATE
#define NTP_CLOCK_RATE 1
#endif

// Unix time of 2023-11-14. time() returns a value near 0 until SNTP has
// answered, so anything below this means "not synced yet".
static const time_t PLAUSIBLE_TIME = 1700000000;

bool NtpClock::configured = false;
bool NtpClock::synced = false;
unsigned long NtpClock::lastPoll = 0;

void NtpClock::loop() {
  // Only in station mode. In AP mode the CS is its own network and there is
  // no route to an NTP server, so there is nothing to wait for.
  if (WiFi.status() != WL_CONNECTED) return;

  // Nothing here needs to be timely, once a second is plenty.
  unsigned long now = millis();
  if (now - lastPoll < 1000) return;
  lastPoll = now;

  if (!configured) {
    // Starts the SNTP client, which then resyncs on its own in the
    // background (hourly by default).
    configTzTime(NTP_CLOCK_TZ, NTP_CLOCK_SERVER);
    configured = true;
    DIAG(F("NTP clock: asking %S for the time"), F(NTP_CLOCK_SERVER));
    return;
  }

  time_t rawTime = time(nullptr);
  if (rawTime < PLAUSIBLE_TIME) return; // SNTP has not answered yet

  struct tm local;
  if (!localtime_r(&rawTime, &local)) return;

  // Derive the model time from the wall clock rather than counting ticks,
  // so a resync or a missed loop cannot make the model clock drift.
  uint32_t secondsToday = local.tm_hour * 3600UL
                        + local.tm_min * 60UL
                        + local.tm_sec;
  int16_t modelMinutes = (secondsToday * (uint32_t)NTP_CLOCK_RATE / 60) % 1440;

  if (!synced) {
    synced = true;
    DIAG(F("NTP clock: synced to %d:%d, rate %d"),
         local.tm_hour, local.tm_min, (int)NTP_CLOCK_RATE);
  }

  // Ignores repeated values, so calling this every second costs nothing.
  CommandDistributor::setClockTime(modelMinutes, (int8_t)NTP_CLOCK_RATE);
}

#endif // ARDUINO_ARCH_ESP32 && NTP_CLOCK
