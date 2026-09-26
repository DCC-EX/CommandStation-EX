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


// The wifi preferences implementation for ESP32 using NVSTable, but includes
// this file contains the implementation of the EXNetworkPreferences class for ESP32 using NVSTable.
// The hostnamem enabled and node flags are also used by ethernet on stm32 platforms.

#include "EXNetworkPreferences.h"
#include "StringFormatter.h"
#include "NVSTable.h"

bool EXNetworkPreferences::load() {
  auto hostName=NVSTable::getTextNVS(SystemPreferences::hostName);
  if (hostName[0]==0) {
    // We have not been here before, set all non ""/0 defaults 
    NVSTable::setNVS(SystemPreferences::hostName, "DCC-EX",false);
    NVSTable::setNVS(SystemPreferences::enabled, true,false);
  #ifdef ARDUINO_ARCH_ESP32
    NVSTable::setNVS(SystemPreferences::channelAP, 11,false);
    NVSTable::setNVS(SystemPreferences::hiddenAP, false,false);
    NVSTable::setNVS(SystemPreferences::throttleNode, true,false);
  #endif
    NVSTable::save();
  }

  return true;
}

#ifdef ARDUINO_ARCH_ESP32

// Wifi stuff is ESP32 only 
char EXNetworkPreferences::tempssidSTA[32];
char EXNetworkPreferences::tempPasswordSTA[64];

void EXNetworkPreferences::saveSTA(const char *_ssid, const char *_password,  bool sticky) {
  if (sticky) {
    NVSTable::setNVS(SystemPreferences::ssidSTA, _ssid, false);
    NVSTable::setNVS(SystemPreferences::passwordSTA, _password, false);
    NVSTable::save();
    tempssidSTA[0]='\0';
    tempPasswordSTA[0]='\0';
    return; 
  }

  strncpy(tempssidSTA, _ssid, sizeof(tempssidSTA));
  strncpy(tempPasswordSTA, _password, sizeof(tempPasswordSTA));
}


void EXNetworkPreferences::saveAP(const char *_ssid, const char *_password,  byte _channel, bool _hidden) {
  NVSTable::setNVS(SystemPreferences::ssidAP, _ssid,false);
  NVSTable::setNVS(SystemPreferences::passwordAP, _password,false);
  NVSTable::setNVS(SystemPreferences::channelAP, _channel,false);
  NVSTable::setNVS(SystemPreferences::hiddenAP, _hidden,false);
  NVSTable::save();
}

const char *EXNetworkPreferences::getSsidSTA() {
  if (tempssidSTA[0] != '\0') return tempssidSTA;
  return NVSTable::getTextNVS(SystemPreferences::ssidSTA);
}

const char *EXNetworkPreferences::getPasswordSTA() {
  if (tempPasswordSTA[0] != '\0') return tempPasswordSTA;
  return NVSTable::getTextNVS(SystemPreferences::passwordSTA);
}
const char *EXNetworkPreferences::getSsidAP() {
  return NVSTable::getTextNVS(SystemPreferences::ssidAP);
}
const char *EXNetworkPreferences::getPasswordAP() {
  return NVSTable::getTextNVS(SystemPreferences::passwordAP);
}

byte EXNetworkPreferences::getChannelAP() {
  return NVSTable::getNVS(SystemPreferences::channelAP,true);
}

bool EXNetworkPreferences::getHiddenAP() {
  return NVSTable::getNVS(SystemPreferences::hiddenAP,true);
}
#endif


void EXNetworkPreferences::saveHostName(const char *_hostname) {
  NVSTable::setNVS(SystemPreferences::hostName, _hostname,true);
}

void EXNetworkPreferences::clear() {
  // clear all network preferences from NVS
  for (uint16_t i = SystemPreferences::_FIRST; i < SystemPreferences::_LAST; i++) {
    NVSTable::setNVS(i,(int16_t)0,false);
  }
  NVSTable::save();
  load(); // reload to update static variables and defaults
}

void EXNetworkPreferences::enable(bool enable) {
  NVSTable::setNVS(SystemPreferences::enabled, enable, true);
}

void EXNetworkPreferences::saveThrottleNode(bool _throttleNode) {
  NVSTable::setNVS(SystemPreferences::throttleNode, _throttleNode, true);
}

// getters
bool EXNetworkPreferences::getEnabled() {
   return NVSTable::getNVS(SystemPreferences::enabled,true);
}

const char *EXNetworkPreferences::getHostName() {
  return NVSTable::getTextNVS(SystemPreferences::hostName);
}
bool EXNetworkPreferences::getThrottleNode() {
  return NVSTable::getNVS(SystemPreferences::throttleNode,true);
}

void EXNetworkPreferences::dump(Print* stream) {
  StringFormatter::send(stream, 
      F("<* C HOSTNAME \"%s\" *>\n"), NVSTable::getTextNVS(SystemPreferences::hostName));
  #ifdef ARDUINO_ARCH_ESP32
  StringFormatter::send(stream, 
      F("<* C WIFI %S%S *>\n"), 
        NVSTable::getNVS(SystemPreferences::enabled)?F("ON"):F("OFF"),
        NVSTable::getNVS(SystemPreferences::throttleNode,true)?F(""):F(" [NODE]"));
  
  if (NVSTable::getTextNVS(SystemPreferences::ssidAP)[0]) StringFormatter::send(stream, 
      F("<* C WIFI %S \"%s\" \"%s\" %d *>\n"),
        NVSTable::getNVS(SystemPreferences::hiddenAP,true)?F("HIDDENAP"):F("AP"),
        NVSTable::getTextNVS(SystemPreferences::ssidAP), NVSTable::getTextNVS(SystemPreferences::passwordAP), NVSTable::getNVS(SystemPreferences::channelAP,true));
  
    if (NVSTable::getTextNVS(SystemPreferences::ssidSTA)[0]) StringFormatter::send(stream, 
      F("<* C WIFI \"%s\" \"********\" *>\n"),
        NVSTable::getTextNVS(SystemPreferences::ssidSTA));
#else
  StringFormatter::send(stream, 
      F("<* C ETHERNET %S%S *>\n"), 
        NVSTable::getNVS(SystemPreferences::enabled)?F("ON"):F("OFF"),
        NVSTable::getNVS(SystemPreferences::throttleNode,true)?F(""):F(" [NODE]"));
  #endif
}
