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
#ifdef  still_todo
#ifndef ARDUINO_ARCH_ESP32
#include "WifiPreferences.h"
#include <EEPROM.h>
#include "WifiInterface.h"
#include "DIAG.h"

#define SHOVE(field) EEPROM.put(savedEEPROMAddress+offsetof(SavedState, field), state.field)

WifiPreferences::SavedState WifiPreferences::state;
int WifiPreferences::savedEEPROMAddress=EEPROM.length() - sizeof(state);

bool WifiPreferences::load() {
  // create default AP mode SSID and password based on MAC address
  char strMac[8];
  WifiInterface::getMac(strMac,sizeof(strMac));


  if (savedEEPROMAddress<0) {
    DIAG(F("EEPROM too small for WifiPreferences "), sizeof(state));
    return false;
  }
 
  EEPROM.get(savedEEPROMAddress, state);
  if (memcmp(state.eepromFlag, EEPROM_FLAG, sizeof(EEPROM_FLAG))!=0) {
    // no valid preferences in EEPROM, so initialize defaults
    strncpy(state.eepromFlag, EEPROM_FLAG, sizeof(state.eepromFlag));
    state.enabled=true;
    state.channelAP=11;
    state.hiddenAP=false;
    state.hideAPPassword=true;
    strncpy(state.ssidAP,"DCCEX_",sizeof(state.ssidAP));
    strncat(state.ssidAP, strMac, sizeof(state.ssidAP) - strlen(state.ssidAP) - 1);
    strncpy(state.passwordAP,"PASS_",sizeof(state.passwordAP));
    strncat(state.passwordAP, strMac, sizeof(state.passwordAP) - strlen(state.passwordAP) - 1);
    state.ssidSTA[0]=0;
    state.passwordSTA[0]=0;
    strncpy(state.hostName, "DCC-EX",sizeof(state.hostName));
    EEPROM.put(savedEEPROMAddress, state);
  }
  return true;
}

bool WifiPreferences::saveSTA(const char *_ssid, const char *_password,  bool sticky) {
  if (strlen(_ssid)>=sizeof(state.ssidSTA)) return false; // SSID too long
  if (strlen(_password)>=sizeof(state.passwordSTA)) return false; // Password too long
  strncpy(state.ssidSTA, _ssid, sizeof(state.ssidSTA));
  strncpy(state.passwordSTA, _password, sizeof(state.passwordSTA));
  if (!sticky) return true; // do not save to preferences if not sticky
  SHOVE(ssidSTA);
  SHOVE(passwordSTA);

  WifiInterface::setup(); // restart wifi to apply new credentials
  return true;
}

bool WifiPreferences::saveAP(const char *_ssid, const char *_password,  byte _channel, bool _hidden) {
  if (strlen(_ssid)>=sizeof(state.ssidAP)) return false; // SSID too long
  if (strlen(_password)>=sizeof(state.passwordAP)) return false; // Password too long
  strncpy(state.ssidAP, _ssid, sizeof(state.ssidAP));
  strncpy(state.passwordAP, _password, sizeof(state.passwordAP));
  state.channelAP=_channel;
  state.hiddenAP=_hidden;
  SHOVE(ssidAP);
  SHOVE(passwordAP);
  SHOVE(channelAP);
  SHOVE(hiddenAP);
  WifiInterface::setup(); // restart wifi to apply new credentials
  return true;
}

bool WifiPreferences::saveHostName(const char *_hostname) {
  if (strlen(_hostname)>=sizeof(state.hostName)) return false; // hostname too long
  strncpy(state.hostName, _hostname, sizeof(state.hostName));
  SHOVE(hostName);
  WifiInterface::setup(); // restart wifi to apply new hostname
  return true;
}

void WifiPreferences::clear() {
  strncpy(state.eepromFlag,"DEAD",sizeof(state.eepromFlag));
  SHOVE(eepromFlag);
  load(); // reload to update static variables and defaults
  WifiInterface::setup(); // restart wifi to apply cleared preferences
}

void WifiPreferences::enable(bool enable) {
  if (enable==state.enabled) return; 
  state.enabled=enable;
  SHOVE(enabled);
  WifiInterface::setup(); // restart wifi to apply new enabled state
}

#endif //ARDUINO_ARCH_ESP32
#endif