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
#if defined(ARDUINO_ARCH_ESP8266)
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <vector>
#include <string>

#include "WifiESP8266.h"
#include "DIAG.h"
#include "RingStream.h"
#include "CommandDistributor.h"
#include <string.h>

static std::vector<AsyncClient*> clients; // a list to hold all clients
static AsyncServer *server;

static RingStream *outboundRing = new RingStream(2048);

static void handleError(void* arg, AsyncClient* client, int8_t error) {
  (void)arg;
  DIAG(F("connection error %s from client %s"), client->errorToString(error), client->remoteIP().toString().c_str());
}

static void handleData(void* arg, AsyncClient* client, void *data, size_t len) {
  (void)arg;
  //DIAG(F("data received from client %s"), client->remoteIP().toString().c_str());
  uint8_t clientId;
  for (clientId=0; clientId<clients.size(); clientId++){
    if (clients[clientId] == client) break;
  }
  if (clientId < clients.size()) {
    byte cmd[len+1];
    memcpy(cmd,data,len);
    cmd[len]=0;
    outboundRing->mark(clientId);
    CommandDistributor::parse(clientId,cmd,outboundRing);
    outboundRing->commit();
  }
}

//static AsyncClient *debugclient = NULL;

bool sendData(AsyncClient *client, char* data, size_t count) {
  size_t willsend = 0;

  // reply to client
  if (client->canSend()) {
    while (count > 0) {
      if (client->connected())
	willsend = client->add(data, count); // add checks for space()
      else
	willsend = 0;
      if (willsend < count) {
	DIAG(F("Willsend %d of count %d"), willsend, count);
      }
      if (client->connected() && client->send()) {
	count = count - willsend;
	data = data + willsend;
      } else {
	DIAG(F("Could not send promised %d"), count);
	return false;
      }
    }
    // Did send all bytes we wanted
    return true;
  }
  DIAG(F("Aborting: Busy or space=0"));
  return false;
}

static void deleteClient(AsyncClient* client) {
  uint8_t clientId;
  for (clientId=0; clientId<clients.size(); clientId++){
    if (clients[clientId] == client) break;
  }
  if (clientId < clients.size()) {
    clients[clientId] = NULL;
  }
}
static void handleDisconnect(void* arg, AsyncClient* client) {
  (void)arg;
  DIAG(F("Client disconnected"));
  deleteClient(client);
}

static void handleTimeOut(void* arg, AsyncClient* client, uint32_t time) {
  (void)arg;
  (void)time;
  DIAG(F("client ACK timeout ip: %s"), client->remoteIP().toString().c_str());
  deleteClient(client);
}


static void handleNewClient(void* arg, AsyncClient* client) {
  (void)arg;
  DIAG(F("New client %s"), client->remoteIP().toString().c_str());

  // add to list
  clients.push_back(client);

  // register events
  client->onData(&handleData, NULL);
  client->onError(&handleError, NULL);
  client->onDisconnect(&handleDisconnect, NULL);
  client->onTimeout(&handleTimeOut, NULL);

}

/*  Things one _might_ want to do:
   Disable soft watchdog: ESP.wdtDisable()
   Enable  soft watchdog: ESP.wdtEnable(X) ignores the value of X and enables it for fixed
                          time at least in version 3.0.2 of the esp8266 package.

Internet says:

I manage to complety disable the hardware watchdog on ESP8266 in order to run the benchmark CoreMark. 

void hw_wdt_disable(){
  *((volatile uint32_t*) 0x60000900) &= ~(1); // Hardware WDT OFF
}

void hw_wdt_enable(){
  *((volatile uint32_t*) 0x60000900) |= 1; // Hardware WDT ON
}

*/

bool WifiESP::setup(const char *SSid,
                    const char *password,
                    const char *hostname,
                    int port,
                    const byte channel) {
  bool havePassword = true;
  bool haveSSID = true;
  bool wifiUp = false;

  // We are server and should not sleep
  wifi_set_sleep_type(NONE_SLEEP_T);
  // connects to access point

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

  server = new AsyncServer(port); // start listening on tcp port

  server->onClient(&handleNewClient, server);
  server->begin();
  DIAG(F("Server up port %d"),port);

  return true;
}

void WifiESP::loop() {
  AsyncClient *client = NULL;
  // Do something with outboundRing
  // call sendData
  int clientId=outboundRing->peek();
  if (clientId >= 0) {
    if ((unsigned int)clientId > clients.size()) {
      // something is wrong with the ringbuffer position
      outboundRing->info();
      client = NULL;
    } else {
      client = clients[clientId];
    }
//    if (client != debugclient) {
//      DIAG(F("new client pointer = %x from id %d"), client, clientId);
//      debugclient = client;
//    }
  } else {
    client = NULL;
  }
  if (clientId>=0 && client && client->connected() && client->canSend()) {
    outboundRing->read();
    int count=outboundRing->count();
    //DIAG(F("Wifi reply client=%d, count=%d"), clientId,count);
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
      //DIAG(F("SEND:%s COUNT:%d"),buffer,count);
      uint8_t tries = 3;
      while (! sendData(client, buffer, count)) {
	DIAG(F("senData fail"));
	yield();
	if (tries == 0) break;
      }
    }
  }
#ifdef ESP_DEBUG
  static unsigned long last = 0;
  if (millis() - last > 60000) {
    last = millis();
    DIAG(F("+"));
  }
#endif
  ESP.wdtFeed();
}
#endif //ESP_FAMILY
