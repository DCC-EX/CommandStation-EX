/*
 *  © 2021 Fred Decker
 *  © 2021 Fred Decker
 *  © 2020-2021 Chris Harlow
 *  © 2020, Chris Harlow. All rights reserved.
 *  © 2020, Harald Barth.
 *  
 *  This file is part of Asbelos DCC API
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
 */
#ifndef ARDUINO_AVR_UNO_WIFI_REV2
#include <Arduino.h>
#include "WifiInboundHandler.h"
#include "RingStream.h"
#include "CommandDistributor.h"
#include "DIAG.h"

WifiInboundHandler * WifiInboundHandler::singleton;

void WifiInboundHandler::setup(Stream * ESStream) {
  singleton=new WifiInboundHandler(ESStream);
}

void WifiInboundHandler::loop() {
  singleton->loop1();
}


WifiInboundHandler::WifiInboundHandler(Stream * ESStream) {
  wifiStream=ESStream;
  clientPendingCIPSEND=-1;
  inboundRing=new RingStream(INBOUND_RING);
  outboundRing=new RingStream(OUTBOUND_RING);
  pendingCipsend=false;
} 


// Handle any inbound transmission
// +IPD,x,lll:data is stored in streamer[x]
// Other input returns  
void WifiInboundHandler::loop1() {
   // First handle all inbound traffic events because they will block the sending 
   if (loop2()!=INBOUND_IDLE) return;

   WiThrottle::loop(outboundRing);
   
    // if nothing is already CIPSEND pending, we can CIPSEND one reply
    if (clientPendingCIPSEND<0) {
       clientPendingCIPSEND=outboundRing->read();
       if (clientPendingCIPSEND>=0) {
         currentReplySize=outboundRing->count();
         pendingCipsend=true;
       }
     }
    

    if (pendingCipsend && millis()-lastCIPSEND > CIPSENDgap) {
         if (Diag::WIFI) DIAG( F("WiFi: [[CIPSEND=%d,%d]]"), clientPendingCIPSEND, currentReplySize);
         StringFormatter::send(wifiStream, F("AT+CIPSEND=%d,%d\r\n"),  clientPendingCIPSEND, currentReplySize);
         pendingCipsend=false;
         return;
      }
    
    
    // if something waiting to execute, we can call it 
      int clientId=inboundRing->read();
      if (clientId>=0) {
         int count=inboundRing->count();
         if (Diag::WIFI) DIAG(F("Wifi EXEC: %d %d:"),clientId,count); 
         byte cmd[count+1];
         for (int i=0;i<count;i++) cmd[i]=inboundRing->read();   
         cmd[count]=0;
         if (Diag::WIFI) DIAG(F("%e"),cmd); 
         
         CommandDistributor::parse(clientId,cmd,outboundRing);
         return;
      }
   }



// This is a Finite State Automation (FSA) handling the inbound bytes from an ES AT command processor    

WifiInboundHandler::INBOUND_STATE WifiInboundHandler::loop2() {
  while (wifiStream->available()) {
    int ch = wifiStream->read();

    // echo the char to the diagnostic stream in escaped format
    if (Diag::WIFI) {
      // DIAG(F(" %d/"), loopState);
      StringFormatter::printEscape(ch); // DIAG in disguise
    }

    switch (loopState) {
      case ANYTHING:  // looking for +IPD, > , busy ,  n,CONNECTED, n,CLOSED, ERROR, SEND OK 
        
        if (ch == '+') {
          loopState = IPD;
          break; 
        }
        
        if (ch=='>') { 
           if (Diag::WIFI) DIAG(F("[XMIT %d]"),currentReplySize); 
           for (int i=0;i<currentReplySize;i++) {
             int cout=outboundRing->read();
             wifiStream->write(cout);
             if (Diag::WIFI) StringFormatter::printEscape(cout); // DIAG in disguise
           }
           clientPendingCIPSEND=-1;
           pendingCipsend=false;
           loopState=SKIPTOEND;
           break;
        }
        
        if (ch=='R') { // Received ... bytes 
          loopState=SKIPTOEND;
          break;
        }
       
        if (ch=='S') { // SEND OK probably 
          loopState=SKIPTOEND;
          lastCIPSEND=0; // no need to wait next time 
          break;
        }
        
        if (ch=='b') {   // This is a busy indicator... probabaly must restart a CIPSEND  
           pendingCipsend=(clientPendingCIPSEND>=0);
           if (pendingCipsend) lastCIPSEND=millis(); // forces a gap to next CIPSEND
           loopState=SKIPTOEND; 
           break; 
        }
        
        if (ch>='0' && ch<='9') { 
              runningClientId=ch-'0';
              loopState=GOT_CLIENT_ID;
              break;
        }

        if (ch=='E' || ch=='l') { // ERROR or "link is not valid"
          if (clientPendingCIPSEND>=0) {
            // A CIPSEND was errored... just toss it away
            purgeCurrentCIPSEND(); 
          }
          loopState=SKIPTOEND; 
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
        loopState = (ch == 'D') ?  IPD3 : SKIPTOEND;
        break;
        
      case IPD3:  // Looking for ,   After +IPD
        loopState = (ch == ',') ? IPD4_CLIENT : SKIPTOEND;
        break;
        
      case IPD4_CLIENT:  // reading connection id
        if (ch >= '0' || ch <='9'){
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
          if (Diag::WIFI) DIAG(F("Wifi inbound data(%d:%d):"),runningClientId,dataLength); 
          if (inboundRing->freeSpace()<=(dataLength+1)) {
            // This input would overflow the inbound ring, ignore it  
            loopState=IPD_IGNORE_DATA;
            if (Diag::WIFI) DIAG(F("Wifi OVERFLOW IGNORING:"));    
            break;
          }
          inboundRing->mark(runningClientId);
          loopState=IPD_DATA;
          break; 
        }
        dataLength = dataLength * 10 + (ch - '0');
        break;
        
      case IPD_DATA: // reading data
        inboundRing->write(ch);    
        dataLength--;
        if (dataLength == 0) {
          inboundRing->commit();    
          loopState = ANYTHING;
        }
        break;

      case IPD_IGNORE_DATA: // ignoring data that would not fit in inbound ring
        dataLength--;
        if (dataLength == 0) loopState = ANYTHING;
        break;

      case GOT_CLIENT_ID:  // got x before CLOSE or CONNECTED
        loopState=(ch==',') ? GOT_CLIENT_ID2: SKIPTOEND;
        break;
        
      case GOT_CLIENT_ID2:  // got "x,"  
        if (ch=='C') {
         // got "x C" before CLOSE or CONNECTED, or CONNECT FAILED
         if (runningClientId==clientPendingCIPSEND) purgeCurrentCIPSEND();
         else CommandDistributor::forget(runningClientId);
        }
        loopState=SKIPTOEND;   
        break;
         
      case SKIPTOEND: // skipping for /n
        if (ch=='\n') loopState=ANYTHING;
        break;
    }  // switch
  } // available
  return (loopState==ANYTHING) ? INBOUND_IDLE: INBOUND_BUSY;
}

void WifiInboundHandler::purgeCurrentCIPSEND() {
         // A CIPSEND was sent but errored... or the client closed just toss it away
         CommandDistributor::forget(clientPendingCIPSEND); 
         DIAG(F("Wifi: DROPPING CIPSEND=%d,%d"),clientPendingCIPSEND,currentReplySize);
         for (int i=0;i<currentReplySize;i++) outboundRing->read();
         pendingCipsend=false;  
         clientPendingCIPSEND=-1;
}

#endif
