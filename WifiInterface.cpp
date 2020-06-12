#include "WifiInterface.h"
#include "Config.h"
#include "DIAG.h"
#include "StringFormatter.h"

const char READY_SEARCH[]="\r\nready\r\n";
const char OK_SEARCH[]="\r\nOK\r\n";

bool WifiInterface::connected=false;

void WifiInterface::setup() {
  DIAG(F("\n++++++ Wifi Setup In Progress ++++++++\n"));
  connected=setup2();
  DIAG(F("\n++++++ Wifi Setup %s ++++++++\n"), connected?"OK":"FAILED");
}

bool WifiInterface::setup2()
{
  Serial1.begin(WIFI_BAUD_RATE);    // initialize serial for ESP module
  
  delay(1000);

  StringFormatter::send(Serial1,F("AT+RST\r\n")); // reset module
  if (!checkForOK(10000,READY_SEARCH)) return false; 
  
  StringFormatter::send(Serial1,F("AT+CWMODE=1\r\n")); // configure as access point
  if (!checkForOK(10000,OK_SEARCH)) return false;
 
  StringFormatter::send(Serial1,F("AT+CWJAP=\"%s\",\"%s\"\r\n"),WIFI_SSID,WIFI_PASS);
  if (!checkForOK(20000,OK_SEARCH)) return false;
  
  StringFormatter::send(Serial1,F("AT+CIFSR\r\n")); // get ip address //192.168.4.1
  if (!checkForOK(10000,OK_SEARCH)) return false;
  
  StringFormatter::send(Serial1,F("AT+CIPMUX=1\r\n")); // configure for multiple connections
  if (!checkForOK(10000,OK_SEARCH)) return false;
  
  StringFormatter::send(Serial1,F("AT+CIPSERVER=1,%d\r\n"),WIFI_PORT); // turn on server on port 80
  if (!checkForOK(10000,OK_SEARCH)) return false;

  return true;
}

bool WifiInterface::checkForOK( const int timeout,char * search) {
  long int time = millis()+timeout;
  byte locator=0;
  DIAG(F("\nWifi setup Check:"),search);
  while( time > millis()) {
    while(Serial1.available()) {
      int ch=Serial1.read();
      Serial.write(ch);
      if (ch!=search[locator]) locator=0;
      if (ch==search[locator]){
        locator++;
        if (!search[locator]) {
          DIAG(F("\nOK after %dms\n"),millis()-time+timeout);
          return true;
        }
      }
    }
  }
  DIAG(F("\nTIMEOUT after %dms\n"),timeout);
  return false;
}


void WifiInterface::loop(DCCEXParser & parser) {
    if (!connected) return; 
   while(Serial1.available()) Serial.write(Serial1.read());
   // TODO Read incoming...  
   //       if starts +IPD 
   //          get session id
   //          fill buffer
   //          set session id into PrintAT
   //          call parser
   //          implement parser flush          
   
   }
