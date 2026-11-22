/*
 *  © 2026 Chris Harlow
 *  © 2026 Mathew Winters
 *  © 2026 Herb Morton
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
#include "SystemPreferences.h"
#include "StringFormatter.h"
#include "NVSTable.h"
#include "EXNetwork.h"
#if defined(ARDUINO_ARCH_ESP32)
#include "EXNetworkESP32.h"
#endif
#include "version.h"
#include "GITHUB_SHA.h"
#include "DCC.h"


// allow defined DCCHOSTNAME (eg in platformio.ini) to override the hostname
bool EXNetworkPreferences::load() {
  auto hostName=NVSTable::getTextNVS(SystemPreferences::hostName);
  if (hostName[0]==0) {
#ifdef DCCHOSTNAME
    NVSTable::setNVS(SystemPreferences::hostName, DCCHOSTNAME,false);
#else
    NVSTable::setNVS(SystemPreferences::hostName, "DCC-EX",false);
#endif
    // We have not been here before, set all non ""/0 defaults
    NVSTable::setNVS(SystemPreferences::enabled, true,false);
  #ifdef ARDUINO_ARCH_ESP32
    NVSTable::setNVS(SystemPreferences::channelAP, 11,false);
    NVSTable::setNVS(SystemPreferences::hiddenAP, false,false);
    NVSTable::setNVS(SystemPreferences::throttleNode, true,false);
    NVSTable::setNVS(SystemPreferences::ethernetEnabled, false, false);
  #else
    NVSTable::setNVS(SystemPreferences::ethernetEnabled, true, false);
  #endif
    NVSTable::setNVS(SystemPreferences::nodeGroup, 254,false); // default node group
    NVSTable::save();
  }

#ifdef DCCHOSTNAME
  NVSTable::setNVS(SystemPreferences::hostName, DCCHOSTNAME,false);
  NVSTable::save();
#endif

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
  for (auto pref   = SystemPreferences::_FIRST_NETWORK_PREF; pref < SystemPreferences::_LAST_NETWORK_PREF; pref++) {
    NVSTable::setNVS(pref,0,false);
  }
  NVSTable::save();
  load(); // reload to update static variables and defaults
}

void EXNetworkPreferences::enable(bool enable) {
  NVSTable::setNVS(SystemPreferences::enabled, enable, true);
}

void EXNetworkPreferences::enableEthernet(bool enable) {
  NVSTable::setNVS(SystemPreferences::ethernetEnabled, enable, true);
}

void EXNetworkPreferences::saveThrottleNode(bool _throttleNode) {
  NVSTable::setNVS(SystemPreferences::throttleNode, _throttleNode, true);
}

// getters
bool EXNetworkPreferences::getEnabled() {
   return NVSTable::getNVS(SystemPreferences::enabled,true);
}

bool EXNetworkPreferences::getEthernetEnabled() {
  return NVSTable::getNVS(SystemPreferences::ethernetEnabled, true);
}

const char *EXNetworkPreferences::getHostName() {
  return NVSTable::getTextNVS(SystemPreferences::hostName);
}
bool EXNetworkPreferences::getThrottleNode() {
  return NVSTable::getNVS(SystemPreferences::throttleNode,true);
}
byte EXNetworkPreferences::getNodeGroup() {
  auto value = NVSTable::getNVS(SystemPreferences::nodeGroup,true);
  return value?value:254; // return the value if set, otherwise default to 254
}
void EXNetworkPreferences::saveNodeGroup(byte nodeGroup) {
  NVSTable::setNVS(SystemPreferences::nodeGroup, nodeGroup, true);
}



void EXNetworkPreferences::dump(Print* stream) {
  StringFormatter::send(stream, 
      F("<* C HOSTNAME \"%s\" *>\n"), NVSTable::getTextNVS(SystemPreferences::hostName));
  StringFormatter::send(stream, 
      F("<* C NODEGROUP %d *>\n"), NVSTable::getNVS(SystemPreferences::nodeGroup,true));
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

  StringFormatter::send(stream,
    F("<* WIFI IP %s *>\n"),
    (!exNetworkUsingEthernet && WifiESP::isUp())
      ? WifiESP::getIPAddress().toString().c_str()
      : "no IP");
#endif
#if !defined(ARDUINO_ARCH_ESP32) || defined(ETHERNET_CS_PIN)
  bool ethEnabled = NVSTable::getNVS(SystemPreferences::ethernetEnabled, true);
  StringFormatter::send(stream, 
      F("<* C ETHERNET %S%S *>\n"), 
        ethEnabled?F("ON"):F("OFF"),
        NVSTable::getNVS(SystemPreferences::throttleNode,true)?F(""):F(" [NODE]"));
#ifdef HAS_ETHERNET_HW
  if (ethEnabled) {
    if (EthernetInterface::isUp()) {
      StringFormatter::send(stream,
        F("<* ETHERNET IP %s *>\n"),
        EthernetInterface::getIPAddress().toString().c_str());
    } else {
	#if defined(ETHERNET_CS_PIN)
				StringFormatter::send(stream,
					F("<* ETHERNET %S *>\n"),
					(Ethernet.linkStatus() == LinkOFF) ? F("no cable") : F("no IP"));
	#else
				StringFormatter::send(stream, F("<* C ETHERNET no IP *>\n"));
	#endif
			}
		}
#else
  StringFormatter::send(stream, F("<* ETHERNET_PINS: No hardware/pins compiled *>\n"));
#endif
	#if defined(ETHERNET_PNAME)
		StringFormatter::send(stream, F("<* ETHERNET_PINS %s *>\n"), ETHERNET_PNAME);
	#endif
	#if defined(ETHERNET_CS_PIN)
  #if defined(ARDUINO_ARCH_ESP32)
		StringFormatter::send(stream,
			F("<* SPI Pins: CS = %d, SCK = %d, MISO = %d, MOSI = %d, RST = %d *>\n"),
			ETHERNET_CS_PIN, ETHERNET_SCK_PIN, ETHERNET_MISO_PIN, ETHERNET_MOSI_PIN, ETHERNET_RST_PIN);
  #else
		StringFormatter::send(stream,
			F("<* SPI Pins: CS = %d, SCK = %d, MISO = %d, MOSI = %d, RST = NRST *>\n"),
			ETHERNET_CS_PIN, ETHERNET_SCK_PIN, ETHERNET_MISO_PIN, ETHERNET_MOSI_PIN);
  #endif
  #endif
  #endif
	StringFormatter::send(stream,
		F("<iDCC-EX V-%S / %S / %S G-%S >\n"), 
		F(VERSION), F(ARDUINO_TYPE), DCC::getMotorShieldName(), F(GITHUB_SHA));


//  Additiona line available with date and time add two lines in environment build_flags
//    -DPIO_ENV_NAME=${PIOENV}
//    !python -c "import subprocess; b=subprocess.check_output(['git','rev-parse','--abbrev-ref','HEAD']).decode().strip().replace('/','_'); print('-DGIT_BRANCH=' + b)"

#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

#ifdef PIO_ENV_NAME
  #define BUILD_ENV_STR STR(PIO_ENV_NAME)

#ifdef GIT_BRANCH
  #define BUILD_BRANCH_STR STR(GIT_BRANCH)
#else
  #define BUILD_BRANCH_STR "DCC-EX"
#endif

  StringFormatter::send(stream,
      F("<* %S  %S  %S %S *>\n"),
      F(BUILD_ENV_STR), F(BUILD_BRANCH_STR), F(__DATE__), F(__TIME__));
#endif

}
