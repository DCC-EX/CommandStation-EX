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
#include "StringFormatter.h"

Preferences preferences;

bool WifiPreferences::load() {
  preferences.begin("DCCEX-WIFI", true);
  /* experiments with preferences.isKey("ssid") have proved problematic */
  ssidSTA[0]=0;
  preferences.getString("ssidSTA", ssidSTA, sizeof(ssidSTA));
  passwordSTA[0]=0;
  preferences.getString("passwordSTA", passwordSTA, sizeof(passwordSTA));
  ssidAP[0]=0;
  preferences.getString("ssidAP", ssidAP, sizeof(ssidAP));
  passwordAP[0]=0;
  preferences.getString("passwordAP", passwordAP, sizeof(passwordAP));
  hostName[0]=0;
  preferences.getString("hostName", hostName, sizeof(hostName));
  if (hostName[0]==0) {
    // default host name if not set in preferences
    strncpy(hostName, "DCC-EX",sizeof(hostName));
  }
  enabled = preferences.getBool("enabled", true);
  preferences.end();
  return true;
}

void WifiPreferences::saveSTA(const char *_ssid, const char *_password,  bool sticky) {
  strncpy(ssidSTA, _ssid, sizeof(ssidSTA));
  strncpy(passwordSTA, _password, sizeof(passwordSTA));
  if (!sticky) return; // do not save to preferences if not sticky
  preferences.begin("DCCEX-WIFI", false); // read/write
  preferences.putString("ssidSTA", ssidSTA);
  preferences.putString("passwordSTA", passwordSTA);  
  preferences.end();
}

void WifiPreferences::saveAP(const char *_ssid, const char *_password,  byte _channel) {
  strncpy(ssidAP, _ssid, sizeof(ssidAP));
  strncpy(passwordAP, _password, sizeof(passwordAP));
  channelAP=_channel;
  preferences.begin("DCCEX-WIFI", false); // read/write
  preferences.putString("ssidAP", ssidAP);
  preferences.putString("passwordAP", passwordAP);
  preferences.putUChar("channelAP",channelAP);
  preferences.end();
}

void WifiPreferences::saveHostName(const char *_hostname) {
  strncpy(hostName, _hostname, sizeof(hostName));
  preferences.begin("DCCEX-WIFI", false); // read/write
  preferences.putString("hostName", hostName);
  preferences.end();
}

void WifiPreferences::clear() {
  preferences.begin("DCCEX-WIFI", false); // read/write
  preferences.clear();
  preferences.end();
  load(); // reload to update static variables and defaults
}

void WifiPreferences::enable(bool enable) {
  if (enable==enabled) return; 
  enabled=enable;
  preferences.begin("DCCEX-WIFI", false); // read/write
  preferences.putBool("enabled", enabled);
  preferences.end();
}

// getters
bool WifiPreferences::getEnabled() { return enabled;}
const char *WifiPreferences::getSsidSTA() {return ssidSTA;}
const char *WifiPreferences::getPasswordSTA() {return passwordSTA;}
const char *WifiPreferences::getSsidAP() {return ssidAP;}
const char *WifiPreferences::getPasswordAP() {return passwordAP;}
const char *WifiPreferences::getHostName() {return hostName;}
byte WifiPreferences::getChannelAP() {return channelAP;}


void WifiPreferences::dump(Print* stream) {
  StringFormatter::send(stream, 
                 F("<* C WIFI %S *>\n"), enabled?F("ON"):F("OFF"));
  if (ssidAP[0]) StringFormatter::send(stream, 
                 F("<* C WIFI AP \"%s\" \"%s\" %d *>\n"), ssidAP, passwordAP, channelAP);
  if (ssidSTA[0]) StringFormatter::send(stream, 
                 F("<* C WIFI \"%s\" \"********\" *>\n"), ssidSTA);
  StringFormatter::send(stream, 
                 F("<* C WIFI HOSTNAME \"%s\" *>\n"), hostName);
}

char WifiPreferences::ssidSTA[32] ="";   
char WifiPreferences::passwordSTA[32] ="";
char WifiPreferences::ssidAP[32] ="";
char WifiPreferences::passwordAP[32] ="";
byte WifiPreferences::channelAP = 0;
bool WifiPreferences::enabled  = true;
char WifiPreferences::hostName[32] ="";
#endif //ARDUINO_ARCH_ESP32
