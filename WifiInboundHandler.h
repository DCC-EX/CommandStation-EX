#ifndef WifiInboundHandler_h
#define WifiInboundHandler_h

#include "RingStream.h"
#include "WiThrottle.h"
#include "DIAG.h"

class WifiInboundHandler {
 public:  
   static void setup(Stream * ESStream);
   static void loop();
   
   private:

   static WifiInboundHandler * singleton;
  
   
   enum INBOUND_STATE {
        INBOUND_BUSY,     // keep calling in loop() 
        INBOUND_IDLE     // Nothing happening, outbound may xcall CIPSEND
   };      

        enum LOOP_STATE {
          ANYTHING,    // ready for +IPD, n CLOSED, n CONNECTED, busy etc...
          SKIPTOEND,   // skip to newline
          
          // +IPD,client,length:data
          IPD,         // got +
          IPD1,        // got +I
          IPD2,        // got +IP
          IPD3,        // got +IPD
          IPD4_CLIENT,  // got +IPD,  reading cient id
          IPD5,        // got +IPD,c 
          IPD6_LENGTH, // got +IPD,c, reading length 
          IPD_DATA,    // got +IPD,c,ll,: collecting data
          IPD_IGNORE_DATA, // got +IPD,c,ll,: ignoring the data that won't fit inblound Ring

          GOT_CLIENT_ID,  // clientid prefix to CONNECTED / CLOSED
          GOT_CLIENT_ID2  // clientid prefix to CONNECTED / CLOSED
  };

  
   WifiInboundHandler(Stream * ESStream);
   void loop1();
   INBOUND_STATE loop2();
   void purgeCurrentCIPSEND();
   Stream * wifiStream;
   
   static const int INBOUND_RING = 512;
   static const int OUTBOUND_RING = 2048;
 
   RingStream * inboundRing;
   RingStream * outboundRing;
     
  LOOP_STATE loopState=ANYTHING;
  int runningClientId;   // latest client inbound processing data or CLOSE
  int dataLength; // dataLength of +IPD
  int clientPendingCIPSEND=-1;
  int currentReplySize;
  bool pendingCipsend;
};
#endif
