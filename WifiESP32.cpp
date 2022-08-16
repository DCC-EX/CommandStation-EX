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

#if defined(ARDUINO_ARCH_ESP32)
#include <vector>
#include "defines.h"
#include <WiFi.h>
#include "esp_wifi.h"
#include "WifiESP32.h"
#include "DIAG.h"
#include "RingStream.h"
#include "CommandDistributor.h"
#include "WiThrottle.h"
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
  };
  bool ok() {
    return (inUse && wifi.connected());
  };
  bool recycle(WiFiClient c) {

    if (inUse == true) return false;

    // return false here until we have
    // implemented a LRU timer
    // if (LRU too recent) return false;
    return false;

    wifi = c;
    inUse = true;
    return true;
  };
  WiFiClient wifi;
  bool inUse = true;
};

static std::vector<NetworkClient> clients; // a list to hold all clients
static WiFiServer *server = NULL;
static RingStream *outboundRing = new RingStream(2048);
//static RingStream *eventRing = new RingStream(2048);
static bool APmode = false;

void wifiLoop(void *){
  for(;;){
    WifiESP::loop();
  }
}

bool WifiESP::setup(const char *SSid,
                    const char *password,
                    const char *hostname,
                    int port,
                    const byte channel) {
  bool havePassword = true;
  bool haveSSID = true;
  bool wifiUp = false;
  uint8_t tries = 40;

  // tests
  //  enableCoreWDT(1);
  //  disableCoreWDT(0);

  // clean start
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);
  //WiFi.useStaticBuffers(true); // does not help either

  const char *yourNetwork = "Your network ";
  if (strncmp(yourNetwork, SSid, 13) == 0 || strncmp("", SSid, 13) == 0)
    haveSSID = false;
  if (strncmp(yourNetwork, password, 13) == 0 || strncmp("", password, 13) == 0)
    havePassword = false;

  if (haveSSID && havePassword) {
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.setAutoReconnect(true);
    WiFi.begin(SSid, password);
    while (WiFi.status() != WL_CONNECTED && tries) {
      Serial.print('.');
      tries--;
      delay(500);
    }
    if (WiFi.status() == WL_CONNECTED) {
      DIAG(F("Wifi STA IP %s"),WiFi.localIP().toString().c_str());
      wifiUp = true;
    } else {
      DIAG(F("Could not connect to Wifi SSID %s"),SSid);
      DIAG(F("Forcing one more Wifi restart"));
      esp_wifi_start();
      esp_wifi_connect();
      tries=40;
      while (WiFi.status() != WL_CONNECTED && tries) {
	Serial.print('.');
	tries--;
	delay(500);
      }
      if (WiFi.status() == WL_CONNECTED) {
	DIAG(F("Wifi STA IP 2nd try %s"),WiFi.localIP().toString().c_str());
	wifiUp = true;
      } else {
	DIAG(F("Fail 2nd try"));
      }
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
    WiFi.setSleep(false);
    if (WiFi.softAP(strSSID.c_str(),
		    havePassword ? password : strPass.c_str(),
		    channel, false, 8)) {
      DIAG(F("Wifi AP SSID %s PASS %s"),strSSID.c_str(),havePassword ? password : strPass.c_str());
      DIAG(F("Wifi AP IP %s"),WiFi.softAPIP().toString().c_str());
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
  server = new WiFiServer(port); // start listening on tcp port
  server->begin();
  // server started here

  //start loop task
  if (pdPASS != xTaskCreatePinnedToCore(
	wifiLoop, /* Task function. */
	"wifiLoop",/* name of task.  */
	10000,     /* Stack size of task */
	NULL,      /* parameter of the task */
	1,         /* priority of the task */
	NULL,      /* Task handle to keep track of created task */
	0)) {      /* pin task to core 0 */
    DIAG(F("Could not create wifiLoop task"));
    return false;
  }

  // report server started after wifiLoop creation
  // when everything looks good
  DIAG(F("Server up port %d"),port);
  return true;
}

void WifiESP::loop() {
  int clientId; //tmp loop var

  // really no good way to check for LISTEN especially in AP mode?
  if (APmode || WiFi.status() == WL_CONNECTED) {
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
    if (server->hasClient()) {
      WiFiClient client;
      while (client = server->available()) {
	for (clientId=0; clientId<clients.size(); clientId++){
	  if (clients[clientId].recycle(client)) {
	    DIAG(F("Recycle client %d %s"), clientId, client.remoteIP().toString().c_str());
	    break;
	  }
	}
	if (clientId>=clients.size()) {
	  NetworkClient nc(client);
	  clients.push_back(nc);
	  DIAG(F("New client %d, %s"), clientId, client.remoteIP().toString().c_str());
	}
      }
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
	  outboundRing->mark(clientId);
	  CommandDistributor::parse(clientId,cmd,outboundRing);
	  if (outboundRing->peekTargetMark()!=255) //XXX fix 255 later
	    outboundRing->commit();
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
	  clients[clientId].wifi.write(buffer,count);
	} else {
	  DIAG(F("Unsent(%d): %s"), clientId, buffer);
	}
      }
    }
  } //connected

  // when loop() is running on core0 we must
  // feed the core0 wdt ourselves as yield()
  // is not necessarily yielding to a low
  // prio task. On core1 this is not a problem
  // as there the wdt is disabled by the
  // arduio IDE startup routines.
  if (xPortGetCoreID() == 0)
    feedTheDog0();
  yield();
}
#endif //ESP32
