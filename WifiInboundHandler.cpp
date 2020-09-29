#include <Arduino.h>
#include "WifiInboundHandler.h"
#include "WiThrottle.h"
#include "DIAG.h"

WifiInboundHandler * WifiInboundHandler::singleton;

void WifiInboundHandler::setup(Stream * ESStream) {
  singleton=new WifiInboundHandler(ESStream);
}

void WifiInboundHandler::loop() {
  WiThrottle::loop();  // check heartbeats
  singleton->loop1();
}


WifiInboundHandler::WifiInboundHandler(Stream * ESStream) {
  wifiStream=ESStream;
  for (int clientId=0;clientId<MAX_CLIENTS;clientId++) {
      clientStatus[clientId]=UNUSED;
      // Note buffer is 1 byte longer than MemStream is told 
      // so that we can always inject a '\0' at stream->available()
      clientBuffer[clientId]=new byte[MAX_WIFI_BUFFER+1];
      clientStream[clientId]=new MemStream(clientBuffer[clientId], MAX_WIFI_BUFFER);      
  }
  clientPendingCIPSEND=-1;
  parser=new DCCEXParser();
} 


// Handle any inbound transmission
// +IPD,x,lll:data is stored in streamer[x]
// Other input returns  
void WifiInboundHandler::loop1() {
   
   // First handle all inbound traffic events 
   switch (loop2()) {

      case INBOUND_BUSY:     // keep calling in loop()
        break; 
      
      case INBOUND_IDLE:     // Nothing happening
        // if nothing is already CIPSEND pending, we can CIPSEND one reply
        if (clientPendingCIPSEND<0) {
          for (int clientId=0;clientId<MAX_CLIENTS;clientId++) {
             if (clientStatus[clientId]==REPLY_PENDING) {
                clientPendingCIPSEND=clientId;
                if (Diag::WIFI) DIAG( F("\nWiFi: AT+CIPSEND=%d,%d"), clientId, clientStream[clientId]->available());
                StringFormatter::send(wifiStream, F("AT+CIPSEND=%d,%d\r\n"), clientId, clientStream[clientId]->available());
                clientStatus[clientId]=CIPSEND_PENDING;
                break;
             }
          }
        }
        // if something waiting to process we can call one of them 
             
        for (int clientId=0;clientId<MAX_CLIENTS;clientId++) {
          if (clientStatus[clientId]==READY_TO_PROCESS) {
             processCommand(clientId); 
             break;
          }
        }
        break;
        
     case INBOUND_SENDNOW:  // > received for current CIPSEND
        wifiStream->write(clientBuffer[clientPendingCIPSEND], clientStream[clientPendingCIPSEND]->available());
        clientStatus[clientPendingCIPSEND]=UNUSED;
        break;
             
     case INBOUND_RESEND:    // CIPSEND denied busy, switch it back to pending           
        clientStatus[clientPendingCIPSEND]=REPLY_PENDING;
        break;
   }
}


// This is a Finite State Automation (FSA) handling the inbound bytes from an ES AT command processor    

WifiInboundHandler::INBOUND_STATE WifiInboundHandler::loop2() {
  while (wifiStream->available()) {
    int ch = wifiStream->read();

    // echo the char to the diagnostic stream in escaped format
    if (Diag::WIFI) StringFormatter::printEscape(ch); // DIAG in disguise

    switch (loopState) {
      case ANYTHING:  // looking for +IPD, > , busy ,  n CONNECTED, n CLOSED 
        if (ch == '+') {
          loopState = IPD;
          break; 
        }
        if (ch=='>') { 
          loopState=ANYTHING;
          return INBOUND_SENDNOW;
        }
        if (ch=='b') {   // This is a busy indicator... probabaly must restart a CIPSEND  
           loopState=SKIPTOEND; 
           return INBOUND_RESEND;
        }
        if (ch>='0' && ch<=('0'+MAX_CLIENTS)) { 
              runningClientId=ch-'0';
              loopState=GOT_CLIENT_ID;
              break;
        }
        
        break;
        
      case IPD:  // Looking for I   in +IPD
        loopState = (ch == 'I') ? IPD1 : SKIPTOEND;
        break;
        
      case IPD1:  // Looking for P   in +IPD
        loopState = (ch == 'P') ? IPD2 : SKIPTOEND;
        break;
        
      case IPD2:  // Looking for D   in +IPD
        loopState = (ch == 'D') ?  IPD2 : SKIPTOEND;
        break;
        
      case IPD3:  // Looking for ,   After +IPD
        loopState = (ch == ',') ? IPD4_CLIENT : SKIPTOEND;
        break;
        
      case IPD4_CLIENT:  // reading connection id
        if (ch >= '0' || ch <('0'+MAX_CLIENTS)){
           runningClientId=ch-'0';
           loopState=IPD5;
        }
        else loopState=SKIPTOEND;
        break;
        
      case IPD5:  // Looking for ,   After +IPD,client
        loopState = (ch == ',') ? IPD6_LENGTH : SKIPTOEND;
        dataLength=0;  // ready to start collecting the length
        break;
        
      case IPD6_LENGTH: // reading for length
        if (ch == ':') {
          if (dataLength==0) {
            loopState=ANYTHING;
            break;
          }
          clientStream[runningClientId]->flush(); // prepare streamer for input
          clientStatus[runningClientId]=INBOUND_ARRIVING;
          loopState=IPD_DATA;
          break; 
        }
        dataLength = dataLength * 10 + (ch - '0');
        break;
        
      case IPD_DATA: // reading data
        clientStream[runningClientId]->write(ch); // NOTE: The MemStream will throw away bytes that do not fit in the buffer.
                            // This protects against buffer overflows even with things as innocent
                            // as a browser which send massive, irrlevent HTTP headers.   
        dataLength--;
        if (dataLength == 0) {
          clientStatus[runningClientId]=READY_TO_PROCESS;
          loopState = ANYTHING;
        }
        break;

      case GOT_CLIENT_ID:  // got x before CLOSE or CONNECTED
        loopState=(ch==' ') ? GOT_CLIENT_ID2: SKIPTOEND;
        break;
        
      case GOT_CLIENT_ID2:  // got "x "  before CLOSE or CONNECTED
        loopState=(ch=='C') ? GOT_CLIENT_ID3: SKIPTOEND;
        break;
        
      case GOT_CLIENT_ID3:  // got "x C" before CLOSE or CONNECTED (which is ignored)
         if(ch=='L') clientStatus[runningClientId]=CLOSE_PENDING;
         loopState=SKIPTOEND;   
         break;
         
      case SKIPTOEND: // skipping for /n
        if (ch=='\n') loopState=ANYTHING;
        break;
    }  // switch
  } // available
  return (loopState==ANYTHING) ? INBOUND_IDLE: INBOUND_BUSY;
}


void WifiInboundHandler::processCommand(byte clientId) {
  clientStatus[clientId]=PROCESSING;             
  byte * buffer=clientBuffer[clientId];
  MemStream * streamer=clientStream[clientId];
  buffer[streamer->available()]='\0';

  if (Diag::WIFI) DIAG(F("\n%l Wifi(%d)<-[%e]\n"), millis(),clientId, buffer);
  streamer->setBufferContentPosition(0, 0); // reset write position to start of buffer
  // SIDE EFFECT WARNING:::
  //  We know that parser will read the entire buffer before starting to write to it.
  //  Otherwise we would have to copy the buffer elsewhere and RAM is in short supply.

 

/*******************
  // Intercept HTTP requests
  if (isHTTP(buffer)) {
    if (httpCallback) httpCallback(streamer, buffer);
    else {
      StringFormatter::send(streamer, F("HTTP/1.1 404 Not Found\nContent-Type: text/html\nConnnection: close\n\n"));
      StringFormatter::send(streamer, F("<html><body>This is <b>not</b> a web server.<br/></body></html>"));
    }
    closeAfter = true;
  }
  else 
  *********/
  if (buffer[0] == '<')  parser->parse(streamer, buffer, true); // tell JMRI parser that ACKS are blocking because we can't handle the async

  else WiThrottle::getThrottle(clientId)->parse(*streamer, buffer);
  
  buffer[streamer->available()]='\0'; // mark end of buffer, so it can be used as a string later
  
  if (streamer->available() == 0) {
    clientStatus[clientId]=UNUSED;
  }
  else {
    if (Diag::WIFI) DIAG(F("%l WiFi(%d)->[%e] l(%d)\n"), millis(), clientId, buffer, streamer->available());
    clientStatus[clientId]=REPLY_PENDING;  
  }
}
/*********** close after HTML stuff 
  if (Diag::WIFI) DIAG(F("\n Wifi AT+CIPCLOSE=%d\r\n"), connectionId);
          StringFormatter::send(wifiStream, F("AT+CIPCLOSE=%d\r\n"), connectionId);
          loopState = 0; // wait for +IPD
        }
        break;
*****/
