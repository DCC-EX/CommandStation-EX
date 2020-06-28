#include "WifiInterface.h"
#include "Config.h"
#include "DIAG.h"
#include "StringFormatter.h"
#include "WiThrottle.h"
 
const char  PROGMEM READY_SEARCH[]  ="\r\nready\r\n";
const char  PROGMEM OK_SEARCH[] ="\r\nOK\r\n";
const char  PROGMEM END_DETAIL_SEARCH[] ="@ 1000";
const char  PROGMEM PROMPT_SEARCH[] =">";

bool WifiInterface::connected=false;
DCCEXParser  WifiInterface::parser;
byte WifiInterface::loopstate=0;
int WifiInterface::datalength=0;
int WifiInterface::connectionId;
byte WifiInterface::buffer[MAX_WIFI_BUFFER];
MemStream  WifiInterface::streamer(buffer,sizeof(buffer));

void WifiInterface::setup(Stream & wifiStream,  const __FlashStringHelper* SSid, const __FlashStringHelper* password, int port) {
  
  DIAG(F("\n++++++ Wifi Setup In Progress ++++++++\n"));
  connected=setup2(wifiStream, SSid, password,port);
  // TODO calloc the buffer and streamer and parser etc 
  DIAG(F("\n++++++ Wifi Setup %S ++++++++\n"), connected?F("OK"):F("FAILED"));
}

bool WifiInterface::setup2(Stream & wifiStream, const __FlashStringHelper* SSid, const __FlashStringHelper* password, int port)
{
  
  delay(1000);

  StringFormatter::send(wifiStream,F("AT+RST\r\n")); // reset module
  checkForOK(wifiStream,5000,END_DETAIL_SEARCH,true);  // Show startup but ignore unreadable upto ready
  if (!checkForOK(wifiStream,5000,READY_SEARCH,false)) return false; 
  
  StringFormatter::send(wifiStream,F("AT+CWMODE=1\r\n")); // configure as access point
  if (!checkForOK(wifiStream,10000,OK_SEARCH,true)) return false;
 
  StringFormatter::send(wifiStream,F("AT+CWJAP=\"%S\",\"%S\"\r\n"),SSid,password);
  if (!checkForOK(wifiStream,20000,OK_SEARCH,true)) return false;
  
  StringFormatter::send(wifiStream,F("AT+CIFSR\r\n")); // get ip address //192.168.4.1
  if (!checkForOK(wifiStream,10000,OK_SEARCH,true)) return false;
  
  StringFormatter::send(wifiStream,F("AT+CIPMUX=1\r\n")); // configure for multiple connections
  if (!checkForOK(wifiStream,10000,OK_SEARCH,true)) return false;
  
  StringFormatter::send(wifiStream,F("AT+CIPSERVER=1,%d\r\n"),port); // turn on server on port 80
  if (!checkForOK(wifiStream,10000,OK_SEARCH,true)) return false;

  return true;
}

bool WifiInterface::checkForOK(Stream & wifiStream, const int timeout, const char * waitfor, bool echo) {
  long int startTime = millis();
  char *locator=waitfor;
  DIAG(F("\nWifi setup Check: %S\n"),waitfor);
  while( millis()-startTime < timeout) {
    while(wifiStream.available()) {
      int ch=wifiStream.read();
      if (echo) Serial.write(ch);
      if (ch!=pgm_read_byte_near(locator)) locator=waitfor;
      if (ch==pgm_read_byte_near(locator)) {
        locator++;
        if (!pgm_read_byte_near(locator)) {
          DIAG(F("\nOK after %dms\n"),millis()-startTime);
          return true;
        }
      }
    }
  }
  DIAG(F("\nTIMEOUT after %dms\n"),timeout);
  return false;
}

 
void WifiInterface::loop(Stream & wifiStream) {
    if (!connected) return; 
    
    WiThrottle::loop();  // check heartbeats 
    
    // read anything into a buffer, collecting info on the way  
    while (loopstate!=99 && wifiStream.available()) { 
      int ch=wifiStream.read();
      switch (loopstate) {
           case 0:  // looking for +
                connectionId=0;
                streamer.flush();
                if (ch=='+') loopstate=1;
                break;
           case 1:  // Looking for I     
                loopstate= (ch=='I')?2:0;
                break; 
           case 2:  // Looking for P     
                loopstate= (ch=='P')?3:0;
                break; 
           case 3:  // Looking for D     
                loopstate= (ch=='D')?4:0;
                break; 
           case 4:  // Looking for ,     
                loopstate= (ch==',')?5:0;
                break; 
           case 5:  // reading connection id
                if (ch==',') loopstate=6;
                else connectionId=10*connectionId+(ch-'0');
                break;
           case 6: // reading for length
                if (ch==':') loopstate=(datalength==0)?99:7;  // 99 is getout without reading next char
                else datalength=datalength*10 + (ch-'0');
                break;
           case 7: // reading data 
                streamer.write(ch);
                datalength--;
                if (datalength==0) loopstate=99;
                break;
        } // switch 
    } // while
    if (loopstate!=99) return; 
    streamer.write('\0');

    DIAG(F("\nWifiRead:%d:%s\n"),connectionId,buffer);
    streamer.setBufferContentPosition(0,0);  // reset write position to start of buffer
    // SIDE EFFECT WARNING::: 
    //  We know that parser will read the entire buffer before starting to write to it.
    //  Otherwise we would have to copy the buffer elsewhere and RAM is in short supply.

    // TODO ... tell JMRI parser that callbacks are diallowed because we dont want to handle the async 
    
    if (buffer[0]=='<')  parser.parse(streamer,buffer);
    else WiThrottle::getThrottle(streamer, connectionId)->parse(streamer, buffer);

       
    if (streamer.available()) { // there is a reply to send 
        DIAG(F("WiFiInterface Responding (%d) %s\n"),connectionId,buffer);
        
        StringFormatter::send(wifiStream,F("AT+CIPSEND=%d,%d\r\n"),connectionId,streamer.available());
        streamer.write('\0');
        if (checkForOK(wifiStream,1000,PROMPT_SEARCH,true))  wifiStream.print((char *) buffer);
    }
    loopstate=0;  // go back to looking for +IPD 
    }
