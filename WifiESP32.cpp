/*
    © 2023, 2026 Paul M. Antoine
    © 2021 Harald Barth
    © 2023 Nathan Kellenicki
    © 2025, 2026 Chris Harlow

    This file is part of CommandStation-EX

    This is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    It is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with CommandStation.  If not, see <https://www.gnu.org/licenses/>.
*/

#if defined(ARDUINO_ARCH_ESP32)
#include <vector>
#include "defines.h"
#include "ESPmDNS.h"
#include "esp_wifi.h"
#include "WifiESP32.h"
#include "DIAG.h"
#include "WifiPreferences.h"

#if __has_include ( "soc/rtc_wdt.h")
#include <rtc_wdt.h>
#endif

#include "esp_task_wdt.h"
#include "esp_idf_version.h"
#include "freertos/task.h"






#if __has_include(<esp_mac.h>)
  #include <esp_mac.h>
#else
  #include <esp_system.h>
#endif

#if defined(ESP_IDF_VERSION)
namespace {
  bool sTaskWdtRegistered = false;
  bool sTaskWdtInitAttempted = false;

  void ensureTaskWdtRegistered() {
    if (sTaskWdtInitAttempted) return;
    sTaskWdtInitAttempted = true;

    esp_err_t err = esp_task_wdt_add(nullptr);
    if (err == ESP_OK) {
      sTaskWdtRegistered = true;
    } else if (err != ESP_ERR_INVALID_STATE) {
      DIAG(F("Task WDT add failed: %d"), err);
    }
  }
}

void feedTheDog0(){
  if (!sTaskWdtInitAttempted) {
    ensureTaskWdtRegistered();
  }

  if (sTaskWdtRegistered) {
    esp_err_t err = esp_task_wdt_reset();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
      DIAG(F("Task WDT reset failed: %d"), err);
    }
  }
}
#else
void feedTheDog0(){
  // No IDF version information available.
}
#endif

static bool APmode = false;
bool WifiESP::wifiUp = false;

#ifdef WIFI_LED
int16_t WifiESP::wifiLed = WIFI_LED;
#else
int16_t WifiESP::wifiLed = 0;
#endif


void WifiESP::teardown() {
  // terminate MDNS anouncement
  mdns_service_remove_all();
  mdns_free();
  // stop WiFi
  WiFi.disconnect(true);
  wifiUp = false;
}


bool WifiESP::setup() {
  if (wifiUp) teardown();
  if (wifiLed) {
    pinMode(wifiLed, OUTPUT);
    digitalWrite(wifiLed, 0);
  }
  wifiUp=setupFromPreferences();
  if (wifiLed) digitalWrite(wifiLed, wifiUp);

  if (!wifiUp) return false;

  return true;
}

void WifiESP::setupMDNS() {
  if (!MDNS.begin(WifiPreferences::getHostName())) {
    DIAG(F("Wifi setup failed to start mDNS"));
  }
}

void WifiESP::addService(const char *name, const char *proto, uint16_t port) {
  if (!MDNS.addService(name, proto, port)) {
    DIAG(F("addService failed %s %s %d"), name, proto, port);
  }
}

void WifiESP::addServiceTxt(const char *name, const char *proto, const char *key, const char *value) {
  MDNS.addServiceTxt(name, proto, key, value);
}

bool WifiESP::setupFromPreferences() {
  WifiPreferences::load();
  if (!WifiPreferences::getEnabled()) {
    LCD(5,F("WIFI OFF"));
    LCD(6,F(""));
    LCD(7,F(""));
    return false;
  }

  // if we have been given an STA connection, try that first
  auto ssidptr=WifiPreferences::getSsidSTA();
  if (ssidptr[0] && ConnectSTA(ssidptr, WifiPreferences::getPasswordSTA())) return true;
    
  // Try for a defined AP mode. ConnectAP will fill missing values from mac.
  if ( ConnectAP(WifiPreferences::getSsidAP(), WifiPreferences::getPasswordAP(), WifiPreferences::getChannelAP()) ) return true;
  
  // all a bit of a mystery 
  return false;
}

const char *wlerror[] = {
  "WL_IDLE_STATUS",
  "WL_NO_SSID_AVAIL",
  "WL_SCAN_COMPLETED",
  "WL_CONNECTED",
  "WL_CONNECT_FAILED",
  "WL_CONNECTION_LOST",
  "WL_DISCONNECTED"
};

static void setStaProtocolsBestEffort() {
  // Prefer higher-throughput modes when available, but never crash if a target/core
  // rejects a protocol bitmap. Fall back to legacy b/g/n which is broadly supported.
  esp_err_t err = ESP_OK;

#if CONFIG_SOC_WIFI_SUPPORT_5G
  wifi_protocols_t proto = {
      .ghz_2g = WIFI_PROTOCOL_11AX,
      .ghz_5g = WIFI_PROTOCOL_11AX,
  };
  err = esp_wifi_set_protocols(WIFI_IF_STA, &proto);
#elif CONFIG_SOC_WIFI_HE_SUPPORT
  err = esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11N | WIFI_PROTOCOL_11AX);
#else
  err = esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11N);
#endif

  if (err != ESP_OK) {
    DIAG(F("esp_wifi_set_protocol failed (%d), falling back to 11b/g/n"), err);
    err = esp_wifi_set_protocol(
        WIFI_IF_STA,
        WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
    if (err != ESP_OK) {
      DIAG(F("Fallback esp_wifi_set_protocol failed (%d)"), err);
    }
  }
}

bool WifiESP::ConnectSTA(const char * SSid, const char * password) {
  WiFi.setHostname(WifiPreferences::getHostName());
  WiFi.mode(WIFI_STA);
  // Optimize Wi-Fi for multicast send performance!!
  // Only advertise the higher bandwidth modes if the ESP32 supports them.
  // Some targets/cores reject narrow protocol bitmaps; use safe fallback instead of aborting.
  setStaProtocolsBestEffort();

#if !CONFIG_SOC_WIFI_SUPPORT_5G
  esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT20);
#endif

#ifdef SERIAL_BT_COMMANDS
  WiFi.setSleep(true);
#else
  WiFi.setSleep(false);
#endif
  WiFi.setAutoReconnect(true);
  // Scan all channels, and select the AP with the strongest signal.
  WiFi.setScanMethod(WIFI_ALL_CHANNEL_SCAN);
  WiFi.setSortMethod(WIFI_CONNECT_AP_BY_SIGNAL);
  
  WiFi.begin(SSid, password);

  uint8_t tries = 40;
  while (WiFi.status() != WL_CONNECTED && tries) {
    USB_SERIAL.print('.');
    tries--;
    delay(500);
  }
  if (WiFi.status() == WL_CONNECTED) {
    DIAG(F("Wifi in STA mode"));
    LCD(5,F(""));
    LCD(6,F(""));
    LCD(7, F("IP: %s"), WiFi.localIP().toString().c_str());
    return true;
  }
  DIAG(F("Could not connect to Wifi SSID %s"),SSid);
  return false;
}

bool WifiESP::ConnectAP(const char * SSid, const char * password,  byte channel) {
// prepare all strings
  bool password_secret=true;
  String strSSID; // retain scope in function for c_str() to be valid
  String strPass;

  // Grab the MAC address of the ESP32 and use the last 3 bytes to create a unique SSID and password if not provided.
  if (!SSid || SSid[0] == 0) {
      uint8_t byteMac[6];
      esp_read_mac(byteMac, ESP_MAC_WIFI_STA);

      char suffix[7];
      snprintf(suffix, sizeof(suffix), "%02x%02x%02x",
              byteMac[3], byteMac[4], byteMac[5]);

      strSSID = "DCCEX_";
      strSSID += suffix;
      SSid = strSSID.c_str();

      strPass = "PASS_";
      strPass += suffix;
      password = strPass.c_str();

      password_secret = false;
  }
  
  WiFi.mode(WIFI_AP);
  // Optimize Wi-Fi for multicast send performance!!
  // Only advertise the higher bandwidth modes if the ESP32 supports them.  Some older ESP32s only support 802.11b/g/n, newer ones support 802.11ax.
  // NB: the ESP32-C5 has a single WiFi radio... so can be configured for **either** 2.4GHz or 5GHz, but not both at the same time.
  // We'll keep the ESP32-C5 in 2.4GHz mode for now, with future 5GHz support. The 5GHz config here is left for reference.
#if CONFIG_SOC_WIFI_SUPPORT_5G
    wifi_protocols_t proto = {
        .ghz_2g = WIFI_PROTOCOL_11AX,
        .ghz_5g = WIFI_PROTOCOL_11AX,
    };
    esp_wifi_set_protocols(WIFI_IF_AP, &proto);
// For the ESP32-C6, and anything similar which supports 802.11ax, but only 2.4GHz, we can use the same 11ax protocol.
#elif CONFIG_SOC_WIFI_HE_SUPPORT
    esp_wifi_set_protocol(
            WIFI_IF_AP,
            WIFI_PROTOCOL_11N |
            WIFI_PROTOCOL_11AX);
// Legacy ESP32s (ESP32, ESP32-S3, ESP32-C3) which support only 2.4GHz, and not 5GHz, can use the 11n protocol.
#else
    esp_wifi_set_protocol(
        WIFI_IF_AP,
        WIFI_PROTOCOL_11N);
#endif

#ifdef SERIAL_BT_COMMANDS
  WiFi.setSleep(true);
#else
  WiFi.setSleep(false);
#endif

// For now, we will force the ESP32-C5 to operate in 2.4GHz mode only, as the 5GHz support is not yet implemented.
#if CONFIG_IDF_TARGET_ESP32C5
    esp_wifi_set_band_mode(WIFI_BAND_MODE_2G_ONLY);
#endif

  const bool hiddenAP = WifiPreferences::getHiddenAP();
  
  if (WiFi.softAP(SSid,password, channel, hiddenAP, 8)) {
    DIAG(F("Wifi in AP mode"));
    LCD(5, F("WIFI: %s"), SSid);
    if (password_secret) LCD(6,F("")); 	
    else LCD(6, F("PASS: %s"),password);
    LCD(7, F("IP: %s"),WiFi.softAPIP().toString().c_str());
    APmode = true;
    return true;
  }
  DIAG(F("Could not set up AP with Wifi SSID %s"),SSid);
  return false;
}

void WifiESP::loop() {
  
  auto wlStatus=WiFi.status();

  if (!APmode && wlStatus != WL_CONNECTED) { // in STA mode but not connected any more
    // kick it again THIS IS CRAZY BECAUSE THE DELAYS WILL CAUSE ISSUES
    if (wlStatus <= 6) {
      DIAG(F("Wifi aborted with error %s. Kicking Wifi!"), wlerror[wlStatus]);
      esp_wifi_start();
      esp_wifi_connect();
      for (uint8_t tries=40; WiFi.status() != WL_CONNECTED && tries; tries--) {
	      Serial.print('.');
	      delay(500);
      }
  }
}

  // when loop() is running on core0 we must
  // feed the core0 wdt ourselves as yield()
  // is not necessarily yielding to a low
  // prio task. On core1 this is not a problem
  // as there the wdt is disabled by the
  // arduio IDE startup routines.
  if (xPortGetCoreID() == 0) {
    feedTheDog0();
    yield();
  }
}

#endif //ESP32
