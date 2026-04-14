/*
 *  © 2023 Chris Harlow
 *  All rights reserved.
 *
 *  This file is part of CommandStation-EX
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


/**************************************************
  HOW IT WORKS

  1) Refer to https://developer.mozilla.org/en-US/docs/Web/API/WebSockets_API/Writing_WebSocket_servers
  
  2) When a new client sends in a socket stream, the
     CommandDistributor pass it to this code 
     checkConnectionString() to check for an HTTP
     protocol GET requesting a change to websocket protocol.
     [Note that the WifiInboundHandler has a shortcut to detecting this so that
      it does not need to use up 500+ bytes of RAM just to get at the one parameter that
      actually means something.]
     If that is found, the relevant answer is generated and queued and
     the CommandDistributor marks this client as a websocket client awaiting connection.
     Once the outbound handshake has completed, the CommandDistributor promotes the client
     from awaiting connection to connected websocket so that all
     future traffic for this client is handled with websocket protocol.

  3) When an input is received from a client marked as websocket,
     CommandDistributor calls  unmask() to strip off the websocket header and
     un-mask the input bytes. The command distributor will flag the
     clientid in the ringstream so that anyone transmitting this
     output will know to handle it differently.   

   4) when the  Wifi/Ethernet handler needs to transmit the result from the
      output ring, it recognises the websockets flag and adds the websocket 
      header to the output dynamically. 

 *************************************************************/
#include <Arduino.h>
#include "FSH.h"
#include "RingStream.h"
#include "libsha1.h"
#include "Websockets.h"
#include "DIAG.h"
#ifdef ARDUINO_ARCH_ESP32
  // ESP32 runtime or definitions has strlcat_P missing 
  #define strlcat_P strlcat
#endif  
static const char b64_table[] = {
  'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H',
  'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P',
  'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X',
  'Y', 'Z', 'a', 'b', 'c', 'd', 'e', 'f',
  'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n',
  'o', 'p', 'q', 'r', 's', 't', 'u', 'v',
  'w', 'x', 'y', 'z', '0', '1', '2', '3',
  '4', '5', '6', '7', '8', '9', '+', '/'
};

bool Websockets::checkConnectionString(byte clientId,byte * cmd, RingStream * outbound ) {
    // returns true if this input is a websocket connect    
    if (Diag::WEBSOCKET) DIAG(F("Websock check connection"));   
    /* Heuristic suppose this is a websocket GET
    typically looking like this:
  
    GET / HTTP/1.1
    Host: 192.168.1.242:2560
    Connection: Upgrade
    Pragma: no-cache
    Cache-Control: no-cache
    User-Agent: Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/119.0.0.0 Safari/537.36 Edg/119.0.0.0
    Upgrade: websocket
    Origin: null
    Sec-WebSocket-Version: 13
    Accept-Encoding: gzip, deflate
    Accept-Language: en-US,en;q=0.9
    Sec-WebSocket-Key: SpRkQKPPNZcO62pYf1X6Yg==
    Sec-WebSocket-Extensions: permessage-deflate; client_max_window_bits
    */
   
   // check contents to find Sec-WebSocket-Key: and get key up to \n 
   auto keyPos=strstr_P((char*)cmd,(char*)F("Sec-WebSocket-Key: "));
   if (!keyPos) return false;
   keyPos+=19; // length of Sec-Websocket-Key: 
   auto endkeypos=strstr(keyPos,"\r");
   if (!endkeypos) return false; 
   *endkeypos=0;
   
   if (Diag::WEBSOCKET) DIAG(F("Websock key=\"%s\""),keyPos);
// generate the reply key 
    uint8_t sha1HashBin[21] = { 0 }; // 21 to make it base64 div 3
    char replyKey[100];
    strlcpy(replyKey,keyPos, sizeof(replyKey));
    strlcat_P(replyKey,(char*)F("258EAFA5-E914-47DA-95CA-C5AB0DC85B11"), sizeof(replyKey));
     
    if (Diag::WEBSOCKET) DIAG(F("Websock replykey=%s"),replyKey);

    SHA1_CTX ctx;
    SHA1Init(&ctx);
    SHA1Update(&ctx, (unsigned char *)replyKey, strlen(replyKey));
    SHA1Final(sha1HashBin, &ctx);
      
    // generate the response and embed the base64 encode 
    // of the key
    outbound->mark(clientId);
    outbound->print(F("HTTP/1.1 101 Switching Protocols\r\n"
                    "Server: DCCEX-WebSocketsServer\r\n"
                    "Upgrade: websocket\r\n"
                    "Connection: Upgrade\r\n"
                    "Origin: null\r\n"
                    "Sec-WebSocket-Version: 13\r\n"
                    "Sec-WebSocket-Protocol: DCCEX\r\n"
                    "Sec-WebSocket-Accept: "));
// encode and emit the reply key as base 64
    auto * tmp=sha1HashBin;
    for (int i=0;i<7;i++) { 
      outbound->print(b64_table[(tmp[0] & 0xfc) >> 2]);
      outbound->print(b64_table[((tmp[0] & 0x03) << 4) + ((tmp[1] & 0xf0) >> 4)]);
      outbound->print(b64_table[((tmp[1] & 0x0f) << 2) + ((tmp[2] & 0xc0) >> 6)]);
      if (i<6) outbound->print(b64_table[tmp[2] & 0x3f]);
      tmp+=3;
    }
    outbound->print(F("=\r\n\r\n"));  // because we have padded 1 byte
    outbound->commit();
    return true;          
}

byte * Websockets::unmask(byte clientId,RingStream *ring, byte * buffer) {
 // buffer should have a websocket header
 //byte opcode=buffer[0] & 0x0f;
 if (Diag::WEBSOCKET) DIAG(F("Websock in: %x %x %x %x %x %x %x"),
      buffer[0],buffer[1],buffer[2],buffer[3],
       buffer[4],buffer[5],buffer[6]);

 byte opcode=buffer[0];
 bool maskbit=buffer[1]&0x80;
 int16_t payloadLength=buffer[1]&0x7f;
 
 byte * mask;
 if (payloadLength<126) {
     mask=buffer+2;
     }
 else {
     payloadLength=(buffer[3]<<8)|(buffer[2]);
     mask=buffer+4;
 }
 if (Diag::WEBSOCKET) DIAG(F("Websock op=%x mb=%b pl=%d m=%x %x %x %x"), opcode, maskbit, payloadLength, 
      mask[0],mask[1],mask[2], mask[3]);

 if (opcode==0x89) { // ping
     DIAG(F("Websock ping"));
     buffer[0]=0x8a;  // pong.. and send it back
     ring->mark(clientId &0x7f); // dont readjust
     ring->print((char *)buffer);
     ring->commit();
     return nullptr; 
     }
     
 if (opcode!=0x81) {
  DIAG(F("Websock unknown opcode 0x%x"),opcode);
  return nullptr;
 }
 
 byte * payload=mask+4;
 for (int i=0;i<payloadLength;i++) {
     payload[i]^=mask[i%4];
 }
 
 if (Diag::WEBSOCKET) DIAG(F("Websoc payload=%s"),payload);
   
 return payload; // payload will be parsed as normal
     
 }

 int16_t Websockets::getOutboundHeaderSize(uint16_t dataLength) {
   return (dataLength>=126)? 4:2;
 }

int Websockets::fillOutboundHeader(uint16_t dataLength, byte * buffer) {
    // text opcode, flag(126= use 2 length bytes, no mask bit) , length
    buffer[0]=0x81;
    if (dataLength<126) {
         buffer[1]=(byte)dataLength;
         return 2; 
    }
    buffer[1]=126;
    buffer[2]=(byte)(dataLength & 0xFF);
    buffer[3]= (byte)(dataLength>>8);
    return 4;  
}
    
 void Websockets::writeOutboundHeader(Print * stream,uint16_t dataLength) {
    byte prefix[4];
    int headerlen=fillOutboundHeader(dataLength,prefix);
    stream->write(prefix,sizeof(headerlen));
  }
 


