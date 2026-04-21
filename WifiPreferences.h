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
#include "StringFormatter.h"

const char EEPROM_FLAG[]="DCCEX-WIFI";

// Note... functions that are not cpu-specific are implemented here in the header file to reduce duplication.
//         CPU specific functions are implemented in the corresponding WifiPreferencesXXX.cpp file.
class WifiPreferences {
public:
  static bool load();
  static bool saveSTA(const char *_ssid, const char *_password, bool sticky);
  static bool saveAP(const char *_ssid, const char *_password, byte _channel, bool _hidden);
  static bool saveHostName(const char *_hostname);
  static void clear();
  static void enable(bool enable);
  static bool getEnabled() {return state.enabled;};
  static const char *getSsidSTA() {return state.ssidSTA;};
  static const char *getPasswordSTA() {return state.passwordSTA;};
  static const char *getSsidAP() {return state.ssidAP;};
  static const char *getPasswordAP() {return state.passwordAP;};
  static const char *getHostName() {return state.hostName;};
  static byte getChannelAP() {return state.channelAP;};
  static bool getHiddenAP() {return state.hiddenAP;};
  static bool getHideAPPassword() {return state.hideAPPassword;};
  static void dump(Print * stream) {
      StringFormatter::send(stream, 
                 F("<* C WIFI %S *>\n"), state.enabled?F("ON"):F("OFF"));
  if (state.ssidAP[0]) StringFormatter::send(stream, 
                 F("<* C WIFI %S \"%s\" \"%s\" %d *>\n"),
                 state.hiddenAP?F("HIDDENAP"):F("AP"), state.ssidAP, state.passwordAP, state.channelAP);
  if (state.ssidSTA[0]) StringFormatter::send(stream, 
                 F("<* C WIFI \"%s\" \"********\" *>\n"), state.ssidSTA);
  StringFormatter::send(stream, 
                 F("<* C WIFI HOSTNAME \"%s\" *>\n"), state.  hostName);

  };

// STA limits are chosen to meet standards for external routers, despite being stupidly large.
static const byte MAX_STA_SSID_LENGTH=32;
static const byte MAX_STA_PASSWORD_LENGTH=64;
// AP limits are smaller to fit in the typical small memory of an embedded AP implementation and because a short SSID is more appropriate for an AP. 
static const byte MAX_HOSTNAME_LENGTH=16;
static const byte MAX_AP_SSID_LENGTH=16;
static const byte MAX_AP_PASSWORD_LENGTH=16;

private:
  struct SavedState {
    char eepromFlag[sizeof(EEPROM_FLAG)]; // Reserved for AVR to detect if preferences have been saved before
    bool enabled;
    byte channelAP;
    bool hiddenAP;
    bool hideAPPassword;
    char ssidAP[MAX_AP_SSID_LENGTH+1];
    char passwordAP[MAX_AP_PASSWORD_LENGTH+1];
    char ssidSTA[MAX_STA_SSID_LENGTH+1];
    char passwordSTA[MAX_STA_PASSWORD_LENGTH+1];
    char hostName[MAX_HOSTNAME_LENGTH+1];
  };
  static SavedState state;
  static int savedEEPROMAddress;
};
#endif //WifiPreferences_h
