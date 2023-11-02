/*
    © 2023 Paul M. Antoine
    © 2021 Harald Barth
    © 2023 Nathan Kellenicki

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
#include "defines.h"

#ifdef WIFI_NINA
#include <vector>
#include <SPI.h>
#ifndef ARDUINO_GIGA
#include <WifiNINA.h>
#else
#include <WiFi.h>
#endif
#include "Wifi_NINA.h"
// #include "ESPmDNS.h"
// #include <WiFi.h>
// #include "esp_wifi.h"
// #include "WifiESP32.h"
// #include <SPI.h>
#include "DIAG.h"
#include "RingStream.h"
#include "CommandDistributor.h"
#include "WiThrottle.h"

// Configure the pins used for the ESP32 connection
#if !defined(ARDUINO_GIGA) && defined(ARDUINO_ARCH_STM32) // Here my STM32 configuration
  #define SPIWIFI       SPI  // The SPI port
  #define SPIWIFI_SS    PA4   // Chip select pin
  #define ESP32_RESETN  PA10   // Reset pin
  #define SPIWIFI_ACK   PB3   // a.k.a BUSY or READY pin
  #define ESP32_GPIO0   -1
#else
#warning "WiFiNINA has no SPI port or pin allocations for this archiecture yet!"
#endif

class NetworkClient {
public:
  NetworkClient(WiFiClient c) {
    wifi = c;
  };
 bool ok() {
    return (inUse && wifi.connected());
  };
  bool recycle(WiFiClient c) {

    if (inUse == true) return false;

    // return false here until we have
    // implemented a LRU timer
    // if (LRU too recent) return false;
    //return false;

    wifi = c;
    inUse = true;
    return true;
  };
 WiFiClient wifi;
  bool inUse = true;
};

static std::vector<NetworkClient> clients; // a list to hold all clients
static WiFiServer *server = NULL;
static RingStream *outboundRing = new RingStream(10240);
static bool APmode = false;
static IPAddress ip;

// #ifdef WIFI_TASK_ON_CORE0
// void wifiLoop(void *){
//   for(;;){
//     WifiNINA::loop();
//   }
// }
// #endif

char asciitolower(char in) {
  if (in <= 'Z' && in >= 'A')
    return in - ('Z' - 'z');
  return in;
}

bool WifiNINA::setup(const char *SSid,
                    const char *password,
                    const char *hostname,
                    int port,
                    const byte channel,
                    const bool forceAP) {
  bool havePassword = true;
  bool haveSSID = true;
  bool wifiUp = false;
  uint8_t tries = 40;

  // Set up the pins!
#ifndef ARDUINO_GIGA
  WiFi.setPins(SPIWIFI_SS, SPIWIFI_ACK, ESP32_RESETN, ESP32_GPIO0, &SPIWIFI);
#endif
  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    DIAG(F("Communication with WiFi module failed!"));
    // don't continue for now!
    while (true);
  }

  // Print firmware version on the module
  String fv = WiFi.firmwareVersion();
  DIAG(F("WifiNINA Firmware version found:%s"), fv.c_str());

  // clean start
  // WiFi.mode(WIFI_STA);
  // WiFi.disconnect(true);
  // differnet settings that did not improve for haba
  // WiFi.useStaticBuffers(true);
  // WiFi.setScanMethod(WIFI_ALL_CHANNEL_SCAN);
  // WiFi.setSortMethod(WIFI_CONNECT_AP_BY_SECURITY);

  const char *yourNetwork = "Your network ";
  if (strncmp(yourNetwork, SSid, 13) == 0 || strncmp("", SSid, 13) == 0)
    haveSSID = false;
  if (strncmp(yourNetwork, password, 13) == 0 || strncmp("", password, 13) == 0)
    havePassword = false;

  if (haveSSID && havePassword && !forceAP) {
#ifndef ARDUINO_GIGA
    WiFi.setHostname(hostname); // Strangely does not work unless we do it HERE!
#endif
    // WiFi.mode(WIFI_STA);
    // WiFi.setAutoReconnect(true);
    WiFi.begin(SSid, password);
    while (WiFi.status() != WL_CONNECTED && tries) {
      Serial.print('.');
      tries--;
      delay(500);
    }
    if (WiFi.status() == WL_CONNECTED) {
      // String ip_str = sprintf("%xl", WiFi.localIP());
      DIAG(F("Wifi STA IP %d.%d.%d.%d"), WiFi.localIP()[0], WiFi.localIP()[1],WiFi.localIP()[2],WiFi.localIP()[3],WiFi.localIP()[4],WiFi.localIP()[5]);
      wifiUp = true;
    } else {
      DIAG(F("Could not connect to Wifi SSID %s"),SSid);
      DIAG(F("Forcing one more Wifi restart"));
      // esp_wifi_start();
      // esp_wifi_connect();
      tries=40;
      while (WiFi.status() != WL_CONNECTED && tries) {
	Serial.print('.');
	tries--;
	delay(500);
      }
      if (WiFi.status() == WL_CONNECTED) {
  ip = WiFi.localIP();
  DIAG(F("Wifi STA IP 2nd try %s"), ip);
  wifiUp = true;
      } else {
	DIAG(F("Wifi STA mode FAIL. Will revert to AP mode"));
	haveSSID=false;
      }
    }
  }
  if (!haveSSID || forceAP) {
    // prepare all strings
    String strSSID(forceAP ? SSid : "DCCEX_");
    String strPass(forceAP ? password : "PASS_");
    if (!forceAP) {
      byte mac[6];
      WiFi.macAddress(mac);
      String strMac;
      for (int i = 0; i++; i < 6) {
        strMac += String(mac[i], HEX);
      }

      DIAG(F("MAC address: %x:%x:%x:%x:%X;%x"), mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

      strMac.remove(0,9);
      strMac.replace(":","");
      strMac.replace(":","");
      // convert mac addr hex chars to lower case to be compatible with AT software
      std::transform(strMac.begin(), strMac.end(), strMac.begin(), asciitolower);
      strSSID.concat(strMac);
      strPass.concat(strMac);
    }

    if (WiFi.beginAP(strSSID.c_str(),
		    havePassword ? password : strPass.c_str(),
		    channel) == WL_AP_LISTENING) {
      DIAG(F("Wifi AP SSID %s PASS %s"),strSSID.c_str(),havePassword ? password : strPass.c_str());
      ip = WiFi.localIP();
      DIAG(F("Wifi AP IP %s"),ip);
      wifiUp = true;
      APmode = true;
    } else {
      DIAG(F("Could not set up AP with Wifi SSID %s"),strSSID.c_str());
    }
  }


  if (!wifiUp) {
    DIAG(F("Wifi setup all fail (STA and AP mode)"));
    // no idea to go on
    return false;
  }

  // TODO: we need to run the MDNS_Generic server I suspect
  // // Now Wifi is up, register the mDNS service
  // if(!MDNS.begin(hostname)) {
  //   DIAG(F("Wifi setup failed to start mDNS"));
  // }
  // if(!MDNS.addService("withrottle", "tcp", 2560)) {
  //   DIAG(F("Wifi setup failed to add withrottle service to mDNS"));
  // }

  server = new WiFiServer(port); // start listening on tcp port
  server->begin();
  // server started here

// #ifdef WIFI_TASK_ON_CORE0
//   //start loop task
//   if (pdPASS != xTaskCreatePinnedToCore(
// 	wifiLoop, /* Task function. */
// 	"wifiLoop",/* name of task.  */
// 	10000,     /* Stack size of task */
// 	NULL,      /* parameter of the task */
// 	1,         /* priority of the task */
// 	NULL,      /* Task handle to keep track of created task */
// 	0)) {      /* pin task to core 0 */
//     DIAG(F("Could not create wifiLoop task"));
//     return false;
//   }

//   // report server started after wifiLoop creation
//   // when everything looks good
//   DIAG(F("Server starting (core 0) port %d"),port);
// #else
  DIAG(F("Server will be started on port %d"),port);
// #endif
  ip = WiFi.localIP();
  LCD(4,F("IP: %d.%d.%d.%d"), ip[0], ip[1], ip[2], ip[3]);
  LCD(5,F("Port:%d"), port);
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

void WifiNINA::loop() {
  int clientId; //tmp loop var

  // really no good way to check for LISTEN especially in AP mode?
  wl_status_t wlStatus;
  if (APmode || (wlStatus = (wl_status_t)WiFi.status()) == WL_CONNECTED) {
    // loop over all clients and remove inactive
    for (clientId=0; clientId<clients.size(); clientId++){
      // check if client is there and alive
      if(clients[clientId].inUse && !clients[clientId].wifi.connected()) {
        DIAG(F("Remove client %d"), clientId);
        CommandDistributor::forget(clientId);
        clients[clientId].wifi.stop();
        clients[clientId].inUse = false;
        
        //Do NOT clients.erase(clients.begin()+clientId) as
        //that would mix up clientIds for later.
      }
    }
    WiFiClient client = server->available();
    if (client == true) {
      ///while (client.available() == true) {
        for (clientId=0; clientId<clients.size(); clientId++){
          if (clients[clientId].recycle(client)) {
            ip = client.remoteIP();
            DIAG(F("Recycle client %d %d.%d.%d.%d"), clientId, ip[0], ip[1], ip[2], ip[3]);
            break;
          }
        }
        if (clientId>=clients.size()) {
          NetworkClient* nc=new NetworkClient(client);
          clients.push_back(*nc);
          //delete nc;
          ip = client.remoteIP();
          DIAG(F("New client %d, %d.%d.%d.%d"), clientId, ip[0], ip[1], ip[2], ip[3]);
        }
      ///}
    }
    // loop over all connected clients
    for (clientId=0; clientId<clients.size(); clientId++){
      if(clients[clientId].ok()) {
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
    if (clientId >= 0) {
      // We have data to send in outboundRing
      // and we have a valid clientId.
      // First read it out to buffer
      // and then look if it can be sent because
      // we can not leave it in the ring for ever
      int count=outboundRing->count();
      {
	char buffer[count+1]; // one extra for '\0'
	for(int i=0;i<count;i++) {
	  int c = outboundRing->read();
	  if (c >= 0) // Panic check, should never be false
	    buffer[i] = (char)c;
	  else {
	    DIAG(F("Ringread fail at %d"),i);
	    break;
	  }
	}
	// buffer filled, end with '\0' so we can use it as C string
	buffer[count]='\0';
	if((unsigned int)clientId <= clients.size() && clients[clientId].ok()) {
	  if (Diag::CMD || Diag::WITHROTTLE)
	    DIAG(F("SEND %d:%s"), clientId, buffer);
	  clients[clientId].wifi.write(buffer,count);
	} else {
	  DIAG(F("Unsent(%d): %s"), clientId, buffer);
	}
      }
    }
  } else if (!APmode) { // in STA mode but not connected any more
    // kick it again
    if (wlStatus <= 6) {
      DIAG(F("Wifi aborted with error %s. Kicking Wifi!"), wlerror[wlStatus]);
      // esp_wifi_start();
      // esp_wifi_connect();
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
}
#endif // WIFI_NINA