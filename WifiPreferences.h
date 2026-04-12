/*
 *  © 2026 Chris Harlow
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

#ifndef WifiPreferences_h
#define WifiPreferences_h
#include <Arduino.h>
class WifiPreferences {
public:
  static bool load();
  static void saveSTA(const char *_ssid, const char *_password, bool sticky);
  static void saveAP(const char *_ssid, const char *_password, byte _channel);
  static void saveHostName(const char *_hostname);
  static void clear();
  static void enable(bool enable);
  static bool getEnabled();
  static const char *getSsidSTA();
  static const char *getPasswordSTA();
  static const char *getSsidAP();
  static const char *getPasswordAP();
  static const char *getHostName();
  static byte getChannelAP();
  static void dump(Print * stream);
private:
  static bool enabled;
  static char ssidAP[32];
  static char passwordAP[32];
  static byte channelAP;
  static char ssidSTA[32];
  static char passwordSTA[32];
  static char hostName[32];
};
#endif //WifiPreferences_h
