#include "WifiInterface.h"
#include "Config.h"
#include "DIAG.h"


WiFiEspServer WifiInterface::server(WIFI_PORT);
WiFiEspClient WifiInterface::client;
bool WifiInterface::haveClient=false;
bool WifiInterface::connected=false;

void WifiInterface::setup()
{
  Serial1.begin(WIFI_BAUD_RATE);    // initialize serial for ESP module
  WiFi.init(&Serial1);    // initialize ESP module

  // check for the presence of the shield
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    return;
  }

  // attempt to connect to WiFi network
  int status = WiFi.begin(WIFI_SSID, WIFI_PASS);
  

  if (status==WL_CONNECTED) {
       connected=true;
  
    // start the server on port WIFI_PORT 
    server.begin();
  
   // print the SSID of the network you're attached to
   DIAG(F("\nWifi Connected SSID: %s  IP=%d.%d.%d.%d port %d\n "), 
   WiFi.SSID(), WiFi.localIP()[0],WiFi.localIP()[1],WiFi.localIP()[2],WiFi.localIP()[3],WIFI_PORT);
  }
}



void WifiInterface::loop(DCCEXParser & parser) {
   if (!connected) return;
   
   WiFiEspClient xclient= server.available();  // listen for incoming clients
   if (xclient.connected()) {
     DIAG(F("\nNew Wifi Client connected\n"));
     parser.loop(xclient);
     xclient.stop();
     }
   }
