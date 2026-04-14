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
#include <WiFi.h>

Preferences preferences;
WifiPreferences::SavedState WifiPreferences::state;


bool WifiPreferences::load() {

  // create default AP mode SSID and password based on MAC address
  String strMac = WiFi.macAddress();
  strMac.remove(0,9);
  strMac.replace(":","");
  strMac.replace(":","");
  // convert mac addr hex chars to lower case to be compatible with AT software
  std::transform(strMac.begin(), strMac.end(), strMac.begin(),
      [](char c){
          if (c <= 'Z' && c >= 'A') c = c - ('Z' - 'z');
          return c;
      });
  
  preferences.begin(EEPROM_FLAG, true);
  /* experiments with preferences.isKey("ssid") have proved problematic */
  
  state.ssidSTA[0]=0;
  preferences.getString("ssidSTA", state.ssidSTA, sizeof(state.ssidSTA));
  
  state.passwordSTA[0]=0;
  preferences.getString("passwordSTA", state.passwordSTA, sizeof(state.passwordSTA));
  
  state.ssidAP[0]=0; 
  preferences.getString("ssidAP", state.ssidAP, sizeof(state.ssidAP));
  if (!state.ssidAP[0]) {
    // default to AP mode with SSID and password based on MAC address if not set in preferences
    strncpy(state.ssidAP,"DCCEX_",sizeof(state.ssidAP));
    strncat(state.ssidAP, strMac.c_str(), sizeof(state.ssidAP) - strlen(state.ssidAP) - 1);
  }
  
  state.passwordAP[0]=0;
  state.hideAPPassword=true; // default to hiding AP password if set in preferences;
  preferences.getString("passwordAP", state.passwordAP, sizeof(state.passwordAP));
  if (!state.passwordAP[0]) {
    strncpy(state.passwordAP,"PASS_",sizeof(state.passwordAP));
    strncat(state.passwordAP, strMac.c_str(), sizeof(state.passwordAP) - strlen(state.passwordAP) - 1);
    state.hideAPPassword= false; 
  }

  strncpy(state.hostName, "DCC-EX",sizeof(state.hostName));
  preferences.getString("hostName", state.hostName, sizeof(state.hostName));
  
  state.enabled = preferences.getBool("enabled", true);
  state.channelAP = preferences.getUChar("channelAP", 11);
  state.hiddenAP = preferences.getBool("hiddenAP", false);
  preferences.end();
  return true;
}

bool WifiPreferences::saveSTA(const char *_ssid, const char *_password,  bool sticky) {
  if (strlen(_ssid)>=sizeof(state.ssidSTA)) return false; // SSID too long
  if (strlen(_password)>=sizeof(state.passwordSTA)) return false; // Password too long
  strncpy(state.ssidSTA, _ssid, sizeof(state.ssidSTA));
  strncpy(state.passwordSTA, _password, sizeof(state.passwordSTA));
  if (!sticky) return true; // do not save to preferences if not sticky
  preferences.begin(EEPROM_FLAG, false); // read/write
  preferences.putString("ssidSTA", state.ssidSTA);
  preferences.putString("passwordSTA", state.passwordSTA);  
  preferences.end();
  return true;
}

bool WifiPreferences::saveAP(const char *_ssid, const char *_password,  byte _channel, bool _hidden) {
  if (strlen(_ssid)>=sizeof(state.ssidAP)) return false; // SSID too long
  if (strlen(_password)>=sizeof(state.passwordAP)) return false; // Password too long
  strncpy(state.ssidAP, _ssid, sizeof(state.ssidAP));
  strncpy(state.passwordAP, _password, sizeof(state.passwordAP));
  state.channelAP=_channel;
  state.hiddenAP=_hidden;
  preferences.begin(EEPROM_FLAG, false); // read/write
  preferences.putString("ssidAP", state.ssidAP);
  preferences.putString("passwordAP", state.passwordAP);
  preferences.putUChar("channelAP",state.channelAP);
  preferences.putBool("hiddenAP",state.hiddenAP);
  preferences.end();
  return true;
}

bool WifiPreferences::saveHostName(const char *_hostname) {
  if (strlen(_hostname)>=sizeof(state.hostName)) return false; // hostname too long
  strncpy(state.hostName, _hostname, sizeof(state.hostName));
  preferences.begin(EEPROM_FLAG, false); // read/write
  preferences.putString("hostName", state.hostName);
  preferences.end();
  return true;
}

void WifiPreferences::clear() {
  preferences.begin(EEPROM_FLAG, false); // read/write
  preferences.clear();
  preferences.end();
  load(); // reload to update static variables and defaults
}

void WifiPreferences::enable(bool enable) {
  if (enable==state.enabled) return; 
  state.enabled=enable;
  preferences.begin(EEPROM_FLAG, false); // read/write
  preferences.putBool("enabled", state.enabled);
  preferences.end();
}

#endif //ARDUINO_ARCH_ESP32
