/*
 *  © 2021, Chris Harlow. All rights reserved.
 *  
 *  This file is part of DCC-EX/CommandStation-EX
 *
 *  This is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  It is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with CommandStation.  If not, see <https://www.gnu.org/licenses/>.
 * 
 */
#ifdef ARDUINO_AVR_UNO_WIFI_REV2
// This code is ONLY compiled on a unoWifiRev2 processor which uses a different architecture 

#include "WifiInterfaceRev2.h"
#include "DIAG.h"
#include "CommandDistributor.h"
#include <SPI.h>
#include <WiFiNINA.h>


WiFiServer WifiInterface::server(2560);
bool WifiInterface::connected=false;
/**
 * @brief Setup Wifi Connection
 * 
 */
 
 bool WifiInterface::setup(long serial_link_speed, 
                          const FSH *wifiESSID,
                          const FSH *wifiPassword,
                          const FSH *hostname,
                          const int port) {
  (void)serial_link_speed;                    
  (void)port;  // obsolete  
  (void)hostname;  // To be implemented  
  
  if (WiFi.status() == WL_NO_MODULE) {
     DIAG(F("Wifi- hardware failed\n"));
     return false; 
  }
  DIAG(F("Wifi Firmware=%s  expected=%S"),WiFi.firmwareVersion(),F(WIFI_FIRMWARE_LATEST_VERSION));


  int status = WL_IDLE_STATUS;
  int attempts = 4;
  while (status != WL_CONNECTED) {
    if (attempts-- <= 0) {
      DIAG(F("\nFAILED - No Wifi\n"));
      return false;   
    }
    DIAG(F("\nAttempting to connect to %s\n"),wifiESSID);
    status = WiFi.begin(wifiESSID, wifiPassword);
    // wait 10 seconds for connection:
    delay(10000);
  }

  server.begin();  // start the server on port 2560                 

  IPAddress ip = WiFi.localIP();
  LCD(4,F("IP: %d.%d.%d.%d"), ip[0], ip[1], ip[2], ip[3]);
  LCD(5,F("Port:2560"));
  outboundRing=new RingStream(OUTBOUND_RING_SIZE);     
}

/**
 * @brief Main loop for the WifiInterfaceRev2
 * 
 */
void WifiInterface::loop()
{
  WiFiClient client = server.available();   // listen for incoming clients
  if (client)
  {  
     // read bytes from a client
     byte buffer[MAX_NINA_BUFFER];
     int count = client.read(buffer, MAX_NINA_BUFFER-1);
     buffer[count] = '\0'; // terminate the string properly
     if (Diag::WIFI) DIAG(F("WIFI:%e\n"), buffer);
     // TEMPORARY - Assume all clients are client 1, this will confuse WiThrottle! 
     outboundRing->mark(1);
     // TEMPORARY - Assume all clients are client 1, this will confuse WiThrottle! 
     CommandDistributor::parse(1,buffer,outboundRing);
     outboundRing->commit();
     int socketOut=outboundRing->read();
     if (socketOut>=0) {
        int count=outboundRing->count();
        if (Diag::WIFI) DIAG(F("Wifi Reply count=:%d\n"), count);
        for(;count>0;count--)  client.write(outboundRing->read());
        client.flush(); //maybe 
     }
  }
}

#endif
