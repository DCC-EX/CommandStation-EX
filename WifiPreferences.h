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
  static void load();
  static void save(const char *_ssid, const char *_password, byte _channel, bool _forceAP);
  static void clear();
  static const char *getSSID();
  static const char *getPassword();
  static byte getChannel();
  static bool getForceAP();
private:
  static char ssid[32];
  static char password[32];
  static byte channel;
  static bool forceAP;
};
#endif //WifiPreferences_h
