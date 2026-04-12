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
#include "WifiPreferences.h"  
#include "soc/rtc_wdt.h"
#include "esp_task_wdt.h"


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
  wifiUp=setupFromPreferences();
  if (wifiLed) digitalWrite(wifiLed, wifiUp);

  if (!wifiUp) return false;

  // Now Wifi is up, register the mDNS service
  if(!MDNS.begin(WifiPreferences::getHostName())) {
    DIAG(F("Wifi setup failed to start mDNS"));
  }

  server = new WiFiServer(IP_PORT); // start listening on tcp port
  if (!server) return false;
  // server started here
  server->begin();
  if(!MDNS.addService("withrottle", "tcp", IP_PORT)) {
    DIAG(F("Wifi setup failed to add withrottle service to mDNS"));
  }
  DIAG(F("Server has started on port %d"),IP_PORT);
  return true;
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

bool WifiESP::ConnectSTA(const char * SSid, const char * password) {
  WiFi.setHostname(WifiPreferences::getHostName());
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
  
  if (!SSid || SSid[0]==0) {
    String strMac;
    strMac = WiFi.macAddress();
    strMac.remove(0,9);
    strMac.replace(":","");
    strMac.replace(":","");
    // convert mac addr hex chars to lower case to be compatible with AT software
    std::transform(strMac.begin(), strMac.end(), strMac.begin(), asciitolower);
    strSSID.concat("DCCEX_");
    strSSID.concat(strMac);
    SSid=strSSID.c_str();
    strPass.concat("PASS_");
    strPass.concat(strMac);
    password=strPass.c_str();
    password_secret=false;
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
  int clientId; //tmp loop var
  if (!wifiUp) return;

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
