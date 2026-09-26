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

#ifndef EXNetworkPreferences_h
#define EXNetworkPreferences_h
#include <Arduino.h>
enum SystemPreferences : int16_t {
  // Starting value for system preferences in NVS
  // change the order of this enum and mess will happen
  _FIRST=32000,
  ssidSTA, 
  passwordSTA, 
  ssidAP,
  passwordAP, 
  hostName, 
  enabled, 
  channelAP, 
  hiddenAP,
  throttleNode,
  // any additional preferences can only be added here
  _LAST
};

class EXNetworkPreferences {
public:
  static bool load();
  static void saveSTA(const char *_ssid, const char *_password, bool sticky);
  static void saveAP(const char *_ssid, const char *_password, byte _channel, bool _hidden);
  static void saveHostName(const char *_hostname);
  static void saveThrottleNode(bool throttleNode);
  static void clear();
  static void enable(bool enable);
  static bool getThrottleNode();
  static bool getEnabled();
  static const char *getSsidSTA();
  static const char *getPasswordSTA();
  static const char *getSsidAP();
  static const char *getPasswordAP();
  static const char *getHostName();
  static byte getChannelAP();
  static bool getHiddenAP();
  static void dump(Print * stream);
private:
  static char tempssidSTA[32];
  static char tempPasswordSTA[64];
};
#endif //EXNetworkPreferences_h
