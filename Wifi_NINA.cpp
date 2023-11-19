/*
    © 2023 Paul M. Antoine
    © 2021 Harald Barth
    © 2023 Nathan Kellenicki
    © 2023 Travis Farmer
    © 2023 Chris Harlow
    
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

#ifdef WIFI_NINA || GIGA_WIFI
//#include <vector>
#include <SPI.h>
#ifndef ARDUINO_GIGA
#include <WifiNINA.h>
#else
#if defined(GIGA_WIFI)
#include <WiFi.h>
#else
#include <WiFiNINA.h>
#endif
#endif
#include "Wifi_NINA.h"
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
#elif defined(ARDUINO_GIGA)
  #define SPIWIFI       SPI
  #define SPIWIFI_SS    10   // Chip select pin
  #define SPIWIFI_ACK    7   // a.k.a BUSY or READY pin
  #define ESP32_RESETN   5   // Reset pin
  #define ESP32_GPIO0   -1   // Not connected
#else
#warning "WiFiNINA has no SPI port or pin allocations for this archiecture yet!"
#endif
#define MAX_CLIENTS 10

static WiFiServer *server = NULL;
static RingStream *outboundRing = new RingStream(10240);
static bool APmode = false;
static IPAddress ip;

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
#if !defined(GIGA_WIFI)
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
      IPAddress ip = WiFi.localIP();
      DIAG(F("Wifi STA IP %d.%d.%d.%d"), ip[0], ip[1], ip[2], ip[3]);
      wifiUp = true;
    } else {
      DIAG(F("Could not connect to Wifi SSID %s"),SSid);
      DIAG(F("Forcing one more Wifi restart"));
      // esp_wifi_start();
      // esp_wifi_connect();
      WiFi.end();
      WiFi.begin(SSid, password);
      tries=40;
      while (WiFi.status() != WL_CONNECTED && tries) {
        Serial.print('.');
        tries--;
        delay(500);
      }
      if (WiFi.status() == WL_CONNECTED) {
        ip = WiFi.localIP();
        DIAG(F("Wifi STA IP 2nd try %d.%d.%d.%d"), ip[0], ip[1], ip[2], ip[3]);
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

      DIAG(F("MAC address: %x:%x:%x:%x:%x:%x"), mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

      strMac.remove(0,9);
      strMac.replace(":","");
      strMac.replace(":","");
      // convert mac addr hex chars to lower case to be compatible with AT software
      //std::transform(strMac.begin(), strMac.end(), strMac.begin(), asciitolower); ///TJF: why does this fail compile with WiFiNINA, but not giga WiFi???
      strSSID.concat(strMac);
      strPass.concat(strMac);
    }

    if (WiFi.beginAP(strSSID.c_str(),
		    havePassword ? password : strPass.c_str(),
		    channel) == WL_AP_LISTENING) {
      DIAG(F("Wifi AP SSID %s PASS %s"),strSSID.c_str(),havePassword ? password : strPass.c_str());
      ip = WiFi.localIP();
      DIAG(F("Wifi AP IP %d.%d.%d.%d"),ip[0], ip[1], ip[2], ip[3]);
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
  DIAG(F("Server will be started on port %d"),port);

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

WiFiClient * clients[MAX_CLIENTS];  // nulled in setup

void WifiNINA::checkForNewClient() {
  auto newClient=server->available();
  if (!newClient) return;
  for (byte clientId=0; clientId<MAX_CLIENTS; clientId++){
    if (!clients[clientId]) {
      clients[clientId]= new WiFiClient(newClient); // use this slot
      //DIAG(F("New client connected to slot %d"),clientId); //TJF: brought in for debugging.
      return;
    }
  }
}

void WifiNINA::checkForLostClients() {
  for (byte clientId=0; clientId<MAX_CLIENTS; clientId++){
    auto c=clients[clientId];
    if(c && !c->connected()) {
      clients[clientId]->stop();
      //DIAG(F("Remove client %d"), clientId);
      CommandDistributor::forget(clientId);
      clients[clientId]=nullptr;
    }
  }
}

void WifiNINA::checkForClientInput() {
  // Find a client providing input
    for (byte clientId=0; clientId<MAX_CLIENTS; clientId++){
      auto c=clients[clientId];
      if(c) {
        auto len=c->available();
        if (len) {
          // read data from client
          byte cmd[len+1];
          for(int i=0; i<len; i++) cmd[i]=c->read();
          cmd[len]=0x00;
          CommandDistributor::parse(clientId,cmd,outboundRing);
        }
      }
    }
}

void WifiNINA::checkForClientOutput() {
  // something to write out?
  auto clientId=outboundRing->read();
  if (clientId < 0) return;
  auto replySize=outboundRing->count();
  if (replySize==0) return; // nothing to send
  auto c=clients[clientId];
  if (!c) {
    // client is gone, throw away msg
    for (int i=0;i<replySize;i++) outboundRing->read();
    //DIAG(F("gone, drop message.")); //TJF: only for diag
    return;
  }
  // emit data to the client object
  for (int i=0;i<replySize;i++) c->write(outboundRing->read());
}

void WifiNINA::loop() {
  checkForLostClients(); // ***
  checkForNewClient();
  checkForClientInput(); // ***
  WiThrottle::loop(outboundRing); // allow withrottle to broadcast if needed
  checkForClientOutput();
}

#endif // WIFI_NINA