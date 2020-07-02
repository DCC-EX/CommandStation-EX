#include "WifiInterface.h"
#include "Config.h"
#include "DIAG.h"
#include "StringFormatter.h"
#include "WiThrottle.h"
#include "HTTPParser.h" 
const char  PROGMEM READY_SEARCH[]  ="\r\nready\r\n";
const char  PROGMEM OK_SEARCH[] ="\r\nOK\r\n";
const char  PROGMEM END_DETAIL_SEARCH[] ="@ 1000";
const char  PROGMEM PROMPT_SEARCH[] =">";
const char  PROGMEM SEND_OK_SEARCH[] ="\r\nSEND OK\r\n";

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

bool WifiInterface::checkForOK(Stream & wifiStream, const unsigned int timeout, const char * waitfor, bool echo) {
  unsigned long  startTime = millis();
   char  const *locator=waitfor;
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

bool WifiInterface::isHTML() {
  
  // POST GET PUT PATCH DELETE
  // You may think a simple strstr() is better... but not when ram & time is in short supply  
  switch (buffer[0]) {
    case 'P':
         if (buffer[1]=='U' && buffer[2]=='T' && buffer[3]==' ' ) return true; 
         if (buffer[1]=='O' && buffer[2]=='S' && buffer[3]=='T' && buffer[4]==' ') return true; 
         if (buffer[1]=='A' && buffer[2]=='T' && buffer[3]=='C' && buffer[4]=='H' && buffer[5]==' ') return true;
         return false; 
    case 'G':
         if (buffer[1]=='E' && buffer[2]=='T' && buffer[3]==' ' ) return true; 
         return false;
    case 'D':
         if (buffer[1]=='E' && buffer[2]=='L' && buffer[3]=='E' && buffer[4]=='T' && buffer[5]=='E' && buffer[6]==' ') return true; 
         return false;
    default:
       return false;
  } 
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
                streamer.flush();
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

   bool closeAfter=false;
   // Intercept HTTP requests 
    if (isHTML()) {
      HTTPParser::parse(streamer,buffer);
      closeAfter=true;
    }
    else if (buffer[0]=='<')  parser.parse(streamer,buffer, true);    // tell JMRI parser that callbacks are diallowed because we dont want to handle the async 
 
    else WiThrottle::getThrottle(streamer, connectionId)->parse(streamer, buffer);

       
    if (streamer.available()) { // there is a reply to send 
        streamer.write('\0');
        DIAG(F("WiFiInterface Responding client (%d) l(%d) %s\n"),connectionId,streamer.available()-1,buffer);
        
        StringFormatter::send(wifiStream,F("AT+CIPSEND=%d,%d\r\n"),connectionId,streamer.available()-1);
        if (checkForOK(wifiStream,1000,PROMPT_SEARCH,true))  wifiStream.print((char *) buffer);
        checkForOK(wifiStream,3000,SEND_OK_SEARCH,true);
    }
    if (closeAfter) {
      StringFormatter::send(wifiStream,F("AT+CIPCLOSE=%d\r\n"),connectionId);
      checkForOK(wifiStream,2000,OK_SEARCH,true);
    }
      
    loopstate=0;  // go back to looking for +IPD 
    }
