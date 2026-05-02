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
#include "RingStream.h"
#include "CommandDistributor.h"
#include "WiThrottle.h"
#include "DCC.h"
#include "Websockets.h"
#include "WifiPreferences.h"  
#include "soc/rtc_wdt.h"
#include "esp_task_wdt.h"

// Async UDP transport
#include "AsyncUDP.h"
AsyncUDP udpSend;
AsyncUDP udpReceive;
IPAddress udpBroadcastIP;
const IPAddress udpMulticastIP = { 239, 255, 255, 250 }; // Multicast address for DCC-EX Native Protocol
volatile bool udpCommandReceived = false;
byte udpCommandBuffer[256]; // Buffer for UDP command data - TODO: COMMAND_BUFFER_SIZE
uint32_t udpCommandMillis = 0; // Counter for UDP commands

/* IRAM_ATTR */ void packet_listener(AsyncUDPPacket &packet);


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

  // Start UDP server for DCC-EX Native Protocol
  // Transmit via UDP multicast
  if (!udpSend.connect(udpMulticastIP, IP_PORT)) {
    DIAG(F("Failed to start UDP transmitter for DCC-EX Native Protocol"));
    // return false;
  } 
  else {
    DIAG(F("UDP Multicast transmitter for DCC-EX Native Protocol started on %s:%d"), udpMulticastIP.toString().c_str(), IP_PORT);
    // set the broadcast address for UDP
  }
  
  // Receive via UDP unicast
  // TODO: not yet handling individual clients!!
  if (!udpReceive.listen(IP_PORT)) {
    DIAG(F("Failed to start UDP receiver for DCC-EX Native Protocol"));
    // return false;
  } else {
    udpReceive.onPacket(packet_listener);
    DIAG(F("UDP Multicast receiver for DCC-EX Native Protocol started on port %d"), IP_PORT);
  }

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
  // Optimize Wi-Fi for multicast send performance!!
  // Prefer n only (no slow old 11b/11g) and HT20, as HT40 can cause issues
  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11N);
  esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT20);


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
  // Optimize Wi-Fi for multicast send performance!!
  // Prefer n only (no slow old 11b/11g) and HT20, as HT40 can cause issues
  esp_wifi_set_protocol(WIFI_IF_AP, WIFI_PROTOCOL_11N);
  esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT20);

#ifdef SERIAL_BT_COMMANDS
  WiFi.setSleep(true);
#else
  WiFi.setSleep(false);
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

    // Check UDP
    if (udpCommandReceived) {
      // DIAG(F("UDP Command processed after %d ms"), millis() - udpCommandMillis);
      // We have a command in the udpCommandBuffer
      // Parse it
      DCCEXParser::parse(&USB_SERIAL, udpCommandBuffer, NULL);
      udpCommandReceived = false; // Reset the flag
      udpCommandMillis = 0; // Reset the timer for UDP commands
    }


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

bool WifiESP::udpMulticast(const char *buffer, const int count) {
  bool ok = false;
  if (count <= 0 || count > 1540  // 1540 is the max UDP packet size
      || buffer == NULL) {
    DIAG(F("UDP Multicast: Invalid count %d or buffer %p"), count, buffer);
    return false;
  } 
  // Regardless of the clientId, we can send it via UDP Multicast
  if (udpSend.connected()) {
    // DIAG(F("UDP Multicast send %d bytes: %s"), count, buffer);
    ok = udpSend.print(buffer);
    if (!ok)
      DIAG(F("UDP Multicast send failed"));
  } 
  return ok;
}

// We'll receive UDP packets asynchronously, then send the on to be parsed
void packet_listener(AsyncUDPPacket &packet) {
  // DIAG(F("UDP Packet received: %s:%d -> %s:%d, Length: %d"),
  //      packet.remoteIP().toString().c_str(), packet.remotePort(),
  //      packet.localIP().toString().c_str(), packet.localPort(),
  //      packet.length());
  // DIAG(F("UDP Packet received: %s"), packet.data());
  if (!packet.isBroadcast() && !packet.isMulticast()) {
    if (packet.length() < 2) {
      DIAG(F("UDP Rcv: Packet too short %d bytes"), packet.length());
      return; // not enough data
    }
    if ((packet.data()[0] == '<') && packet.data()[packet.length() - 1] == '\0' && (packet.data()[packet.length() - 2] == '>'|| (packet.data()[packet.length() - 2] == '\n'))) {
      // Valid DCC-EX packet format
      // We found the closing '>', terminate the string
      memcpy(udpCommandBuffer, &packet.data()[0], packet.length());
      udpCommandReceived = true; // Set the flag to indicate a command is ready
      udpCommandMillis = millis(); // Reset the timer for UDP commands

      // for (int i = 0; i < packet.length(); i++) {
      //   if (udpCommandBuffer[i] == '\0')
      //     DIAG(F("UDP Rcv: Null character found in command buffer at position %d"), i);
      //   else
      //     DIAG(F("UDP Rcv: Command buffer[%d] = %c"), i, udpCommandBuffer[i]);
      // }
      // DCCEXParser::parse(&USB_SERIAL, udpCommandBuffer, NULL);
      // DCCEXParser::parse(&USB_SERIAL, packet.data(), NULL);
      // if (Diag::CMD || Diag::WITHROTTLE) {
        // DIAG(F("UDP Rcv %d bytes: %s"), packet.length(), (const char *)packet.data());
        // DIAG(F("UDP Rcv copied %d bytes: %s"), packet.length() - 3, (const char *)udpCommandBuffer);
      // }
    } else {
      // No closing '>', so we have a partial command, just print it
      // for (int i = 0; i < packet.length(); i++) {
      //   if (udpCommandBuffer[i] == '\0')
      //     DIAG(F("UDP Rcv: Null character found in command buffer at position %d"), i);
      //   else
      //     DIAG(F("UDP Rcv: Command buffer[%d] = %c"), i, udpCommandBuffer[i]);
      // }
      packet.data()[packet.length() - 1] = '\0';
      DIAG(F("UDP Rcv partial command: %s"), (const char *)packet.data());
    }
  } else {
      // This is a multicast or broadcast packet
      // Print the packet details
      DIAG(F("UDP Packet: %s"), packet.isBroadcast() ? "Broadcast" : packet.isMulticast() ? "Multicast" : "Unicast");
      DIAG(F("From: %s:%d, To: %s:%d, Length: %d, Data: "), 
           packet.remoteIP().toString().c_str(), packet.remotePort(),
           packet.localIP().toString().c_str(), packet.localPort(),
           packet.length());
  }
}
#endif //ESP32
