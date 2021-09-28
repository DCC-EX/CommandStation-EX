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

#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <vector>

#include "WifiESP.h"
#include "DIAG.h"
#include "RingStream.h"
#include "CommandDistributor.h"
#include <string.h>

static std::vector<AsyncClient*> clients; // a list to hold all clients
static AsyncServer *server;

static RingStream *outboundRing = new RingStream(2048);

static void handleError(void* arg, AsyncClient* client, int8_t error) {
  DIAG(F("connection error %s from client %s"), client->errorToString(error), client->remoteIP().toString().c_str());
}

static void handleData(void* arg, AsyncClient* client, void *data, size_t len) {
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

bool sendData(uint8_t clientId, char* data, int count) {
  AsyncClient *client = clients[clientId];
  size_t willsend = 0;

  // reply to client
  if (client->canSend()) {
    while (count > 0) {
      willsend = client->add(data, count); // add checks for space()
      if (willsend < count) {
	DIAG(F("Willsend %d of count %d"), willsend, count);
      }
      if (client->send()) {
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

static void handleDisconnect(void* arg, AsyncClient* client) {
  DIAG(F("client %s disconnected"), client->remoteIP().toString().c_str());
}

static void handleTimeOut(void* arg, AsyncClient* client, uint32_t time) {
  DIAG(F("client ACK timeout ip: %s"), client->remoteIP().toString().c_str());
}


static void handleNewClient(void* arg, AsyncClient* client) {
  DIAG(F("New client has been connected to server, ip: %s"), client->remoteIP().toString().c_str());

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

bool WifiESP::setup(const char *wifiESSID,
                    const char *wifiPassword,
                    const char *hostname,
                    int port,
                    const byte channel) {
  DIAG(F("START"));
  // connects to access point
  wifi_set_sleep_type(NONE_SLEEP_T);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  DIAG(F("BEGIN"));
  WiFi.begin(wifiESSID, wifiPassword);
  DIAG(F("STATUS"));
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(500);
  }

  DIAG(F("SERVER"));

  server = new AsyncServer(port); // start listening on tcp port

  DIAG(F("CLIENT"));
  server->onClient(&handleNewClient, server);
  DIAG(F("SBEGIN"));
  
  server->begin();

  DIAG(F("ENDSETUP"));

  return true;
}

void WifiESP::loop() {

  // Do something with outboundRing
  // call sendData
  int clientId=outboundRing->read();
  if (clientId>=0) {
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
      while (! sendData(clientId, buffer, count)) {
	DIAG(F("senData fail"));
	yield();
      }
    }
  }

  static unsigned long last = 0;
  if (millis() - last > 60000) {
    last = millis();
    DIAG(F("+"));
  }
  ESP.wdtFeed();
}
