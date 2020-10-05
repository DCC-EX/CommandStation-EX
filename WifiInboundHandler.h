#ifndef WifiInboundHandler_h
#define WifiInboundHandler_h

#include "MemStream.h"
#include "DCCEXParser.h"
#include "DIAG.h"

class WifiInboundHandler {
 public:  
   static void setup(Stream * ESStream);
   static void loop();
   
   private:

   static WifiInboundHandler * singleton;
  
   static const byte MAX_CLIENTS=5;
   static const byte MAX_WIFI_BUFFER=255;
  
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

          GOT_CLIENT_ID,  // clientid prefix to CONNECTED / CLOSED
          GOT_CLIENT_ID2,  // clientid prefix to CONNECTED / CLOSED
          GOT_CLIENT_ID3  // clientid prefix to CONNECTED / CLOSED
  };

  enum CLIENT_STATUS {
    UNUSED,            // client slot not in use
    INBOUND_ARRIVING,      // data is arriving 
    READY_TO_PROCESS,  // data has arrived, may call parser now
    PROCESSING,        // command in progress
    REPLY_PENDING,     // reply is ready to CIPSEND
    CIPSEND_PENDING,   // CIPSEND waiting for >
    CLOSE_PENDING,     // CLOSE received
    CLOSE_AFTER_SEND   // Send CLOSE after CIPSEND completed  
  };
  
   WifiInboundHandler(Stream * ESStream);
   void loop1();
   INBOUND_STATE loop2();
   void processCommand(byte clientId);
   Stream * wifiStream;
   
   DCCEXParser  *parser;

  LOOP_STATE loopState=ANYTHING;
  int runningClientId;   // latest client inbound processing data or CLOSE
  int dataLength; // dataLength of +IPD
  byte * clientBuffer[MAX_CLIENTS];
  MemStream * clientStream[MAX_CLIENTS]; 
  CLIENT_STATUS clientStatus[MAX_CLIENTS];
  bool clientCloseAfterReply[MAX_CLIENTS];
  int clientPendingCIPSEND=-1;
};
#endif
