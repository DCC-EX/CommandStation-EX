/*
    © 2023 Paul M. Antoine
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
#include "RingStream.h"
#include "CommandDistributor.h"
#include "WiThrottle.h"
#include "DCC.h"
#include "Websockets.h"
/*
#include "soc/rtc_wdt.h"
#include "esp_task_wdt.h"
*/

#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"
void feedTheDog0(){
  // feed dog 0
  TIMERG0.wdt_wprotect=TIMG_WDT_WKEY_VALUE; // write enable
  TIMERG0.wdt_feed=1;                       // feed dog
  TIMERG0.wdt_wprotect=0;                   // write protect
  // feed dog 1
  //TIMERG1.wdt_wprotect=TIMG_WDT_WKEY_VALUE; // write enable
  //TIMERG1.wdt_feed=1;                       // feed dog
  //TIMERG1.wdt_wprotect=0;                   // write protect
}

/*
void enableCoreWDT(byte core){
  TaskHandle_t idle = xTaskGetIdleTaskHandleForCPU(core);
  if(idle == NULL){
    DIAG(F("Get idle rask on core %d failed"),core);
  } else {
    if(esp_task_wdt_add(idle) != ESP_OK){
      DIAG(F("Failed to add Core %d IDLE task to WDT"),core);
    } else {
      DIAG(F("Added Core %d IDLE task to WDT"),core);
    }
  }
}

void disableCoreWDT(byte core){
    TaskHandle_t idle = xTaskGetIdleTaskHandleForCPU(core);
    if(idle == NULL || esp_task_wdt_delete(idle) != ESP_OK){
      DIAG(F("Failed to remove Core %d IDLE task from WDT"),core);
    }
}
*/

class NetworkClient {
public:
  NetworkClient(WiFiClient c) {
    wifi = c;
    inUse = true;
  };
  bool active(byte clientId) {
    if (!inUse)
      return false;
    if(!wifi.connected()) {
      DIAG(F("Remove client %d"), clientId);
      CommandDistributor::forget(clientId);
      wifi.stop();
      inUse = false;
      return false;
    }
    return true;
  }
  bool recycle(WiFiClient c) {
    if (wifi == c) {
      if (inUse == true)
	DIAG(F("WARNING: Duplicate"));
      else
	DIAG(F("Returning"));
      inUse = true;
      return true;
    }
    if (inUse == false) {
      wifi = c;
      inUse = true;
      return true;
    }
    return false;
  };
  WiFiClient wifi;
private:
  bool inUse;
};

// file scope variables
static std::vector<NetworkClient> clients; // a list to hold all clients
static RingStream *outboundRing = new RingStream(10240);
static bool APmode = false;
// init of static class scope variables
bool WifiESP::wifiUp = false;

#ifdef WIFI_LED
int16_t WifiESP::wifiLed = WIFI_LED;
#else
int16_t WifiESP::wifiLed = 0;
#endif

WiFiServer *WifiESP::server = NULL;

#ifdef WIFI_TASK_ON_CORE0
void wifiLoop(void *){
  for(;;){
    WifiESP::loop();
  }
}
#endif

char asciitolower(char in) {
  if (in <= 'Z' && in >= 'A')
    return in - ('Z' - 'z');
  return in;
}

void WifiESP::teardown() {
  // stop all locos
  DCC::setThrottle(0,1,1); // this broadcasts speed 1(estop) and sets all reminders to speed 1.
  // terminate all clients connections
  while (!clients.empty()) {
    // pop_back() should invoke destructor which does stop()
    // on the underlying TCP connction
    clients.pop_back();
  }
  // stop server
  if (server != NULL) {
    server->stop();
    server->close();
    server->end();
    DIAG(F("server stop, close, end"));
  }
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

  // Try the setup from preferences first, then from config, and if both fail, give up
  wifiUp=setupFromPreferences();
  if (!wifiUp) wifiUp=setupFromConfig(WIFI_SSID, WIFI_PASSWORD, WIFI_CHANNEL, WIFI_FORCE_AP);
  if (wifiLed) digitalWrite(wifiLed, wifiUp);

  if (!wifiUp) return false;

  // Now Wifi is up, register the mDNS service
  if(!MDNS.begin(WIFI_HOSTNAME)) {
    DIAG(F("Wifi setup failed to start mDNS"));
  }

  server = new WiFiServer(IP_PORT); // start listening on tcp port
  if (!server) return false;
  server->begin();
  // server started here
  if(!MDNS.addService("withrottle", "tcp", IP_PORT)) {
    DIAG(F("Wifi setup failed to add withrottle service to mDNS"));
  }
  DIAG(F("Server has started on port %d"),IP_PORT);
  return true;
}

#include "WifiPreferences.h"
bool WifiESP::setupFromPreferences() {
  if (!WifiPreferences::load()) return false; // load failed, likely no preferences stored
  auto ssidptr=WifiPreferences::getSSID();
  if (ssidptr == nullptr || ssidptr[0] == 0) return false;
  if (WifiPreferences::getForceAP()) {
    return ConnectAP(WifiPreferences::getSSID(), WifiPreferences::getPassword(),  WifiPreferences::getChannel());
  }
  return ConnectSTA(WifiPreferences::getSSID(), WifiPreferences::getPassword());
}

bool WifiESP::setupFromConfig(const char *SSid,
			      const char *password,
			      const byte channel,
			      const bool forceAP) {

  // parameters on entry are from config.h

  if (strcmp("OFF", SSid) == 0) {
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    return false; // debatable if that is true (success) or false (no network)
  }


  // clean start
  WiFi.disconnect(true);
  // differnet settings that did not improve for haba
  // WiFi.useStaticBuffers(true);
  // WiFi.setScanMethod(WIFI_ALL_CHANNEL_SCAN);
  // WiFi.setSortMethod(WIFI_CONNECT_AP_BY_SECURITY);

  const char *yourNetwork = "Your network ";
  if (strncmp(yourNetwork, SSid, 13) == 0  || SSid[0]==0) SSid=nullptr;
  if (strncmp(yourNetwork, password, 13) == 0 || password[0]==0) password=nullptr;

  if (SSid && password && !forceAP) {

    wifiUp=ConnectSTA(SSid, password);
    if (!wifiUp) { 
      DIAG(F("Forcing one more Wifi restart"));
      esp_wifi_start(); // should this be in the connect function? esp_wifi_stop() is called by WiFi.disconnect(true) above, but that does not seem to be enough to recover from some failure modes. Calling esp_wifi_start() here seems to help with some of those failure modes, but it is not clear why.
      esp_wifi_connect();
      wifiUp=ConnectSTA(SSid, password);
      if (!wifiUp) DIAG(F("Wifi STA mode FAIL. Will revert to AP mode"));    
    }
  }
  if (!wifiUp || forceAP) {
    wifiUp=ConnectAP(SSid, password, channel);
  }

  if (!wifiUp) {
    DIAG(F("Wifi setup all fail (STA and AP mode)"));
    // no idea to go on
    return false;
  }

  return true;
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

bool WifiESP::ConnectSTA(const char * SSid, const char * password) {
  WiFi.setHostname(WIFI_HOSTNAME);
  WiFi.mode(WIFI_STA);

#ifdef SERIAL_BT_COMMANDS
  WiFi.setSleep(true);
#else
  WiFi.setSleep(false);
#endif
  WiFi.setAutoReconnect(true);
  WiFi.begin(SSid, password);
  uint8_t tries = 40;
  while (WiFi.status() != WL_CONNECTED && tries) {
    USB_SERIAL.print('.');
    tries--;
    delay(500);
  }
  if (WiFi.status() == WL_CONNECTED) {
    DIAG(F("Wifi in STA mode"));
    LCD(7, F("IP: %s"), WiFi.localIP().toString().c_str());
    return true;
  }
  DIAG(F("Could not connect to Wifi SSID %s"),SSid);
  return false;
}

bool WifiESP::ConnectAP(const char * SSid, const char * password,  byte channel) {
// prepare all strings
  String strMac;
  if (!SSid || !password) {
    strMac = WiFi.macAddress();
    strMac.remove(0,9);
    strMac.replace(":","");
    strMac.replace(":","");
    // convert mac addr hex chars to lower case to be compatible with AT software
    std::transform(strMac.begin(), strMac.end(), strMac.begin(), asciitolower);
  }
  String strSSID;
  if (!SSid) {
    strSSID.concat("DCCEX_");
    strSSID.concat(strMac);
  } else {
    strSSID.concat(SSid);
  }
  String strPass;
  if (!password) {
    strPass.concat("PASS_");
    strPass.concat(strMac);
  } else {
    strPass.concat(password);
  }

  WiFi.setScanMethod(WIFI_ALL_CHANNEL_SCAN); // Scan all channels so we find strongest
  WiFi.mode(WIFI_AP);
#ifdef SERIAL_BT_COMMANDS
  WiFi.setSleep(true);
#else
  WiFi.setSleep(false);
#endif

#ifdef WIFI_HIDE_SSID
  const bool hiddenAP = true;
#else
  const bool hiddenAP = false;
#endif

  if (WiFi.softAP(strSSID.c_str(),strPass.c_str(), channel, hiddenAP, 8)) {
    DIAG(F("Wifi in AP mode"));
    LCD(5, F("Wifi: %s"), strSSID.c_str());
    if (!password) 	LCD(6, F("PASS: %s"),strPass.c_str());
    LCD(7, F("IP: %s"),WiFi.softAPIP().toString().c_str());
    APmode = true;
    return true;
  }
  DIAG(F("Could not set up AP with Wifi SSID %s"),strSSID.c_str());
  return false;
}

void WifiESP::loop() {
  int clientId; //tmp loop var

  // really no good way to check for LISTEN especially in AP mode?
  wl_status_t wlStatus;
  if (APmode || (wlStatus = WiFi.status()) == WL_CONNECTED) {
    if (server->hasClient()) {
      WiFiClient client;
      while (client = server->available()) {
	for (clientId=0; clientId<clients.size(); clientId++){
	  if (clients[clientId].recycle(client)) {
	    DIAG(F("Recycle client %d %s:%d"), clientId, client.remoteIP().toString().c_str(),client.remotePort());
	    break;
	  }
	}
	if (clientId>=clients.size()) {
	  NetworkClient nc(client);
	  clients.push_back(nc);
	  DIAG(F("New client %d, %s:%d"), clientId, client.remoteIP().toString().c_str(),client.remotePort());
	}
      }
    }
    // loop over all connected clients
    // this removes as a side effect inactive clients when checking ::active()
    for (clientId=0; clientId<clients.size(); clientId++){
      if(clients[clientId].active(clientId)) {
	int len;
	if ((len = clients[clientId].wifi.available()) > 0) {
	  // read data from client
	  byte cmd[len+1];
	  for(int i=0; i<len; i++) {
	    cmd[i]=clients[clientId].wifi.read();
	  }
	  cmd[len]=0;
	  CommandDistributor::parse(clientId,cmd,outboundRing);
	}
      }
    } // all clients

    WiThrottle::loop(outboundRing);

    // something to write out?
    clientId=outboundRing->read();
    bool useWebsocket=clientId & Websockets::WEBSOCK_CLIENT_MARKER;
    clientId &= ~ Websockets::WEBSOCK_CLIENT_MARKER;
    if (clientId >= 0) {
      // We have data to send in outboundRing
      // and we have a valid clientId.
      // First read it out to buffer
      // and then look if it can be sent because
      // we can not leave it in the ring for ever
      int count=outboundRing->count();
      auto wsHeaderLen=useWebsocket? Websockets::getOutboundHeaderSize(count) : 0;
      {
        byte buffer[wsHeaderLen + count + 1];  // one extra for '\0'
        if (useWebsocket) Websockets::fillOutboundHeader(count, buffer);
        for (int i = 0; i < count; i++) {
          int c = outboundRing->read();
          if (!c) {
            DIAG(F("Ringread fail at %d"), i);
            break;
          }
          // websocket implementations at browser end can barf at \n
          if (useWebsocket && (c == '\n')) c = '\r';
          buffer[i + wsHeaderLen] = (char)c;
        }
        // buffer filled, end with '\0' so we can use it as C string
	buffer[wsHeaderLen+count]='\0';
	if((unsigned int)clientId <= clients.size()) {
	  if (clients[clientId].active(clientId)) {
	    if (Diag::WIFI)
	      DIAG(F("SEND%S %d:%s"), useWebsocket?F("ws"):F(""),clientId, buffer+wsHeaderLen);
	    clients[clientId].wifi.write(buffer,count+wsHeaderLen);
	  } else {
	    // existed but not active
	    DIAG(F("Unsent(%d): %s"), clientId, buffer+wsHeaderLen);
	  }
	} else {
	  DIAG(F("Non existent client %d has message: %s"), clientId, buffer+wsHeaderLen);
	}
      }
    }
  } else if (!APmode) { // in STA mode but not connected any more
    // kick it again
    if (wlStatus <= 6) {
      DIAG(F("Wifi aborted with error %s. Kicking Wifi!"), wlerror[wlStatus]);
      esp_wifi_start();
      esp_wifi_connect();
      uint8_t tries=40;
      while (WiFi.status() != WL_CONNECTED && tries) {
	Serial.print('.');
	tries--;
	delay(500);
      }
    } else {
      // all well, probably
      //DIAG(F("Running BT"));
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
