/*
    © 2021, Harald Barth.

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
#if defined(ARDUINO_ARCH_ESP32)
#include <WiFi.h>
#include "WifiESP32.h"
#include "DIAG.h"
#include "RingStream.h"
#include "CommandDistributor.h"

static std::vector<WiFiClient> clients; // a list to hold all clients
static WiFiServer *server = NULL;
static RingStream *outboundRing = new RingStream(2048);

bool WifiESP::setup(const char *SSid,
                    const char *password,
                    const char *hostname,
                    int port,
                    const byte channel) {
  bool havePassword = true;
  bool haveSSID = true;
  bool wifiUp = false;

  const char *yourNetwork = "Your network ";
  if (strncmp(yourNetwork, SSid, 13) == 0 || strncmp("", SSid, 13) == 0)
    haveSSID = false;
  if (strncmp(yourNetwork, password, 13) == 0 || strncmp("", password, 13) == 0)
    havePassword = false;

  if (haveSSID && havePassword) {
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.begin(SSid, password);
    while (WiFi.status() != WL_CONNECTED) {
      Serial.print('.');
      delay(500);
    }
    if (WiFi.status() == WL_CONNECTED) {
      DIAG(F("Wifi STA IP %s"),WiFi.localIP().toString().c_str());
      wifiUp = true;
    }
  }
  if (!haveSSID) {
    // prepare all strings
    String strSSID("DCC_");
    String strPass("PASS_");
    String strMac = WiFi.macAddress();
    strMac.remove(0,9);
    strMac.replace(":","");
    strMac.replace(":","");
    strSSID.concat(strMac);
    strPass.concat(strMac);

    WiFi.mode(WIFI_AP);
    if (WiFi.softAP(strSSID.c_str(),
		    havePassword ? password : strPass.c_str(),
		    channel, false, 8)) {
      DIAG(F("Wifi AP SSID %s PASS %s"),strSSID.c_str(),havePassword ? password : strPass.c_str());
      DIAG(F("Wifi AP IP %s"),WiFi.softAPIP().toString().c_str());
      wifiUp = true;
    }
  }


  if (!wifiUp) {
    DIAG(F("Wifi all fail"));
    // no idea to go on
    return false;
  }
  server = new WiFiServer(port); // start listening on tcp port
  server->begin();
  DIAG(F("Server up port %d"),port);

  return true;
  return false;
}

void WifiESP::loop() {
  int clientId; //tmp loop var

  if (WiFi.status() == WL_CONNECTED /* || what for AP? */) {
    if (server->hasClient()) {
      // loop over all clients and remove inactive
      for (clientId=0; clientId<clients.size(); clientId++){
	// check if client is there and alive
	if(!clients[clientId].connected()) {
	  clients[clientId].stop();
	  clients.erase(clients.begin()+clientId);
	}
      }
      WiFiClient client;
      while (client = server->available()) {
	clients.push_back(client);
	DIAG(F("New client %s"), client.remoteIP().toString().c_str());
      }
    }
    // loop over all connected clients
    for (clientId=0; clientId<clients.size(); clientId++){
      if(clients[clientId].connected()) {
	int len;
	if ((len = clients[clientId].available()) > 0) {
	  // read data from client
	  byte cmd[len+1];
	  for(int i=0; i<len; i++) {
	    cmd[i]=clients[clientId].read();
	  }
	  cmd[len]=0;
	  outboundRing->mark(clientId);
	  CommandDistributor::parse(clientId,cmd,outboundRing);
	  outboundRing->commit();
	}
      }
    } // all clients

    // something to write out?
    clientId=outboundRing->peek();
    if (clientId >= 0) {
      if ((unsigned int)clientId > clients.size()) {
	// something is wrong with the ringbuffer position
	outboundRing->info();
      } else {
	// we have data to send in outboundRing
	if(clients[clientId].connected()) {
	  outboundRing->read(); // read over peek()
	  int count=outboundRing->count();
	  {
	    char buffer[count+1];
	    for(int i=0;i<count;i++) {
	      int c = outboundRing->read();
	      if (c >= 0)
		buffer[i] = (char)c;
	      else {
		DIAG(F("Ringread fail at %d"),i);
		break;
	      }
	    }
	    buffer[count]=0;
	    clients[clientId].write(buffer,count);
	  }
	}
      }
    }
  } //connected
}
#endif //ESP32