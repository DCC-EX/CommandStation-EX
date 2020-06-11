//
//#include "WifiInterface.h"
//#include "Config.h"
//#include "DIAG.h"
//
//
//WiFiEspServer WifiInterface::server(WIFI_PORT);
//
//bool WifiInterface::connected=false;
//
//void WifiInterface::setup()
//{
//  Serial1.begin(WIFI_BAUD_RATE);    // initialize serial for ESP module
//  WiFi.init(&Serial1);    // initialize ESP module
//
//  // check for the presence of the shield
//  if (WiFi.status() == WL_NO_SHIELD) {
//    Serial.println("WiFi shield not present");
//    return;
//  }
//
//  // attempt to connect to WiFi network
//  int status = WL_IDLE_STATUS;
//  for (int retries=0;status != WL_CONNECTED && retries<WIFI_CONNECT_RETRIES; retries++) {
//    DIAG(F("\nAttempting to connect to WPA SSID: %s\n"),WIFI_SSID);
//    delay(100);
//    status = WiFi.begin(WIFI_SSID, WIFI_PASS);
//  }
//
//  if (status==WL_CONNECTED) {
//       connected=true;
//  
//  
//    // start the web server on port WIFI_PORT 
//    server.begin();
//  
//   // print the SSID of the network you're attached to
//   DIAG(F("SSID: %s  IP=%s "), WiFi.SSID(), WiFi.localIP());
//  }
//}
//
//
//void WifiInterface::loop(DCCEXParser * parser) {
//  if (!connected) return existing;
//   if (!client) return existing;
//   DIAG(F("\nnew Wifi Client connected %s \n"),client.remoteIP());
//   if (existing) delete existing;  
//   return new DCCEXParser(client);
//}
