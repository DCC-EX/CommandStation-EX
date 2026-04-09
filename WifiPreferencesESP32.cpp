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

#ifdef ARDUINO_ARCH_ESP32
#include "WifiPreferences.h"
#include <Preferences.h>
Preferences preferences;

void WifiPreferences::load() {
  preferences.begin("DCCEX-WIFI", true); // read only
  preferences.getString("ssid", ssid, sizeof(ssid));
  preferences.getString("password", password, sizeof(password));
  channel = preferences.getUChar("channel", 0);
  forceAP = preferences.getBool("forceAP", false);
  preferences.end();
}
void WifiPreferences::save(const char *_ssid, const char *_password,  byte _channel, bool _forceAP) {
  preferences.begin("DCCEX-WIFI", false); // read/write
  preferences.putString("ssid", _ssid);
  preferences.putString("password", _password);
  preferences.putUChar("channel", _channel);
  preferences.putBool("forceAP", _forceAP);
  preferences.end();
  load(); // reload to update static variables
}
void WifiPreferences::clear() {
  preferences.begin("DCCEX-WIFI", false); // read/write
  preferences.clear();
  preferences.end();
  load(); // reload to update static variables
}
const char *WifiPreferences::getSSID() {
  return ssid;
}
const char *WifiPreferences::getPassword() {
  return password;
}
byte WifiPreferences::getChannel() {
  return channel;
}
bool WifiPreferences::getForceAP() {
  return forceAP    ;
}
char WifiPreferences::ssid[32] ="";   
char WifiPreferences::password[32] ="";
byte WifiPreferences::channel = 0;
bool WifiPreferences::forceAP = false;
#endif //ARDUINO_ARCH_ESP32
