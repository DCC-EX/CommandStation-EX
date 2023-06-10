/*
    © 2022 Mark Muzzin
    © 2021 Fred Decker
    © 2021 Fred Decker
    © 2020-2021 Chris Harlow
    © 2020, Chris Harlow. All rights reserved.
    © 2020, Harald Barth.

    This file is part of Asbelos DCC API

    This is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    It is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with CommandStation.  If not, see <https://www.gnu.org/licenses/>.
*/

#ifndef ARDUINO_AVR_UNO_WIFI_REV2
#include <Arduino.h>
#include "WifiInboundHandler.h"
#include "RingStream.h"
#include "CommandDistributor.h"
#include "DIAG.h"

WifiInboundHandler * WifiInboundHandler::singleton;

void WifiInboundHandler::setup(Stream * ESStream) {
  singleton = new WifiInboundHandler(ESStream);
}

void WifiInboundHandler::loop() {
  singleton->loop1();
}


WifiInboundHandler::WifiInboundHandler(Stream * ESStream) {
  wifiStream = ESStream;
  clientPendingCIPSEND = -1;
  inboundRing = new RingStream(INBOUND_RING);
  outboundRing = new RingStream(OUTBOUND_RING);
  cipSendStatus = CIP_SEND_NONE;
}


// Handle any inbound transmission
// +IPD,x,lll:data is stored in streamer[x]
// Other input returns
void WifiInboundHandler::loop1()
{
  // First handle all inbound traffic events because they will block the sending
  if (loop2() != INBOUND_IDLE) return;

  WiThrottle::loop(outboundRing);

  // If no sends are pending we can check if there is any more data to send
  if (cipSendStatus == CIP_SEND_NONE) {

    // If nothing is already CIPSEND pending, we can CIPSEND one reply
    if (clientPendingCIPSEND < 0) {

      //Read the next character in the outbound ring.  If its -1, no data in the ring
      clientPendingCIPSEND = outboundRing->read();

      //If there is data in the ring
      if (clientPendingCIPSEND >= 0 ) {
        if (Diag::WIFI) { ts.cipSendStart = millis(); DIAG( F("cipSendStatus = CIP_SEND_PENDING")); }
        //Get the length of the reply to send
        currentReplySize = outboundRing->count();
        //Set the pending CIPSEND flag
        cipSendStatus = CIP_SEND_PENDING;
      }
    }
  }

  if (cipSendStatus == CIP_SEND_PENDING) {
    if (Diag::WIFI) DIAG( F("WiFi: [[CIPSEND=%d,%d]]"), clientPendingCIPSEND, currentReplySize);
    StringFormatter::send(wifiStream, F("AT+CIPSEND=%d,%d\r\n"),  clientPendingCIPSEND, currentReplySize);
    if (Diag::WIFI) DIAG( F("cipSendStatus = CIP_SEND_SENT [%lms]"),millis()-ts.cipSendStart);
    cipSendStatus = CIP_SEND_SENT;
    return;
  }

  // if something waiting to execute, we can call it
  int clientId = inboundRing->read();
  if (clientId >= 0) {
    int count = inboundRing->count();
    if (Diag::WIFI) DIAG(F("Wifi EXEC: %d %d:"), clientId, count);
    byte cmd[count + 1];
    for (int i = 0; i < count; i++) cmd[i] = inboundRing->read();
    cmd[count] = 0;
    if (Diag::WIFI) DIAG(F("%e"), cmd);

    outboundRing->mark(clientId);  // remember start of outbound data
    CommandDistributor::parse(clientId, cmd, outboundRing);

    // The commit call will either write the length bytes
    // OR rollback to the mark because the reply is empty or commend generated more than fits the buffer
    if (outboundRing->commit() == false) {
      DIAG(F("OUTBOUND FULL processing cmd:%s"), cmd);
      outboundRing->flush();
      inboundRing->flush();
    }
    return;
  }
}

// Fixed length atoi
int WifiInboundHandler::antoi(char* str, int len) {
  int res = 0;

  for (int i = 0; i != len; ++i)
    res = res * 10 + str[i] - '0';

  return res;
}


// This loop processes the inbound bytes from an ES AT command processor
WifiInboundHandler::INBOUND_STATE WifiInboundHandler::loop2() {

  INBOUND_STATE retVal = INBOUND_IDLE;
  char *stringStartPos = NULL;
  char *stringEndPos = NULL;
  char ch = 0;
  int i;
  bool exitLoop = false;

  //Main message input loop
  //This loop fills the small receive buffer one message at a time
  //The execption is the +IPD message.  Only the data up to the ':' character (end of length)
  //is put in this buffer.  The actual +IPD data is written directly into the inbound buffer
  while (!exitLoop)
  {   
    //We exit this loop either if there is no data available, or we have a complete message to process
    while (wifiStream->available() && messageToProcess == RECV_MSG_NONE)
    {
      //Read next character
      ch = wifiStream->read();

      //Add to receive buffer if there is space
      if (recBufferPos < INBOUND_CMD_BUFFER) {
        recBuffer[recBufferPos++] = ch;
      }
      else {
        if (Diag::WIFI) DIAG(F("WARN: Message too long for INBOUND BUFFER, purging"));
        memset(recBuffer, 0, INBOUND_CMD_BUFFER);
        recBufferPos = 0;
      }

      if (Diag::WIFI) StringFormatter::printEscape(ch);

      //Process data ending in \r\n
      if ((recBuffer[recBufferPos - 2] == '\r') && (recBuffer[recBufferPos - 1] == '\n')) {
        messageToProcess = RECV_MSG_CRLF;
      }

      //Ready to send data, will always be the first char in the message
      else if (recBuffer[0] == '>') {
        messageToProcess = RECV_MSG_SND_DATA;
      }

      //IPD message
      else if ((stringStartPos = strstr(recBuffer, "+IPD,")) != NULL && (stringEndPos = strstr(recBuffer, ":")) != NULL) {
        
        if (Diag::WIFI) { ts.ipdStart = millis(); DIAG( F("+IPD Message START")); }
        
        //Advance the start string to the beginning of the length
        stringStartPos += strlen("+IPD,");

        //Get the end of the client ID
        stringEndPos = strstr(stringStartPos, ",");

        //Get the client Id
        runningClientId = antoi(stringStartPos, stringEndPos - stringStartPos);

        //Get the data length
        stringStartPos = stringEndPos + 1;
        stringEndPos = strstr(stringStartPos, ":");
        dataLength = antoi(stringStartPos, stringEndPos - stringStartPos);

        if(dataLength > 0) {
          //Set the remaining data length
          dataRemaining = dataLength;
          messageToProcess = RECV_MSG_IPD_MSG;
        }
      }
    }

    //Process messages
    switch (messageToProcess) {
      case RECV_MSG_NONE:
        //Check if the buffer position is not zero
        if (recBufferPos != 0) {
          //No message to process, but buffer has some data already
          retVal = INBOUND_BUSY;
        }
        else {
          //No message to process, no data in buffer
          retVal = INBOUND_IDLE;
        }
        exitLoop = true;
        break;

      case RECV_MSG_CRLF:

        //Check for a SEND FAIL message
        if (strstr(recBuffer, "SEND FAIL") != NULL) {
          if (Diag::WIFI) DIAG(F("[SEND FAIL - detected"));
          if (cipSendStatus == CIP_SEND_DATA_SENT) {
            if (Diag::WIFI) DIAG( F("cipSendStatus = CIP_SEND_NONE [%lms]"),millis()-ts.cipSendStart);
            cipSendStatus = CIP_SEND_NONE;
          }
        }

        //Check for a SEND OK message
        else if (strstr(recBuffer, "SEND OK") != NULL) {
          if (Diag::WIFI) DIAG(F("[SEND OK - detected [%lms]"),millis()-ts.cipSendStart);
          if (cipSendStatus == CIP_SEND_DATA_SENT) {
            if (Diag::WIFI) DIAG( F("cipSendStatus = CIP_SEND_NONE [%lms]"),millis()-ts.cipSendStart);
            cipSendStatus = CIP_SEND_NONE;
          }
        }

        //Check for an error
        else if (strstr(recBuffer, "ERROR") != NULL) {
          if (clientPendingCIPSEND >= 0) {
            if (Diag::WIFI) DIAG(F("[ERROR detected - purging CIPSEND"));
            // A CIPSEND was errored... just toss it away
            purgeCurrentCIPSEND();
          }
        }

        //Check for an tcp connection closed
        else if (strstr(recBuffer, "link is not valid") != NULL) {
          if (clientPendingCIPSEND >= 0) {
            if (Diag::WIFI) DIAG(F("['link is not valid' detected - purging CIPSEND]"));
            purgeCurrentCIPSEND();
            //Clear out all data from esp8266 receive buffer
            while (wifiStream->available()) {
              wifiStream->read();
            }
          }
        }
        break;

      case RECV_MSG_SND_DATA:
        if (Diag::WIFI) DIAG( F("cipSendStatus = CIP_SEND_DATA_SENT [XMIT START %d] [%lms]"),currentReplySize, millis()-ts.cipSendStart);
        for (i = 0; i < currentReplySize; i++)
        {
          //Read data from the outboundRing and write to to Wifi
          int cout = outboundRing->read();
          wifiStream->write(cout);
          if (Diag::WIFI) StringFormatter::printEscape(cout); // DIAG in disguise
        }

        //Set CIPSEND flag to -1
        clientPendingCIPSEND = -1;
        if (Diag::WIFI) DIAG( F("cipSendStatus = CIP_SEND_DATA_SENT [XMIT END] [%lms]"),millis()-ts.cipSendStart);
        cipSendStatus = CIP_SEND_DATA_SENT;
        break;

      case RECV_MSG_IPD_MSG:

        //Mark the location with the client ID
        if (dataRemaining == dataLength) {
           if (Diag::WIFI) DIAG(F("Wifi inbound data(%d:%d) [%lms]:"), runningClientId, dataLength, millis()-ts.ipdStart);
          inboundRing->mark(runningClientId);
        }

        while (wifiStream->available()) {
          //Read next character (data byte)
          ch = wifiStream->read();
          if (Diag::WIFI) StringFormatter::printEscape(ch);
          if (Diag::WIFI) DIAG(F("[+IPD %d]"), dataRemaining);
          dataRemaining--;

          //Check if we would overflow the inbound ring
          if (inboundRing->freeSpace() <= (dataLength + 1))
          {
            DIAG(F("Wifi OVERFLOW IGNORING:"));

            //If we have an overflow, flush the remaining bytes
            do{
              ch = wifiStream->read();          
            }while(dataRemaining--);

            outboundRing->flush();
            inboundRing->flush();
            
            messageToProcess = RECV_MSG_IPD_DONE;
            break;
          }
          else
          {
            // Write data to inbound ring
            inboundRing->write(ch);

            // Commit inbound ring data
            if (dataRemaining == 0) {
              if (Diag::WIFI) DIAG(F("COMMIT [%lms]:"),millis()-ts.ipdStart);
              if(inboundRing->commit() == false) {
                outboundRing->flush();
                inboundRing->flush();
              }
              messageToProcess = RECV_MSG_IPD_DONE;
              break;
            }
          }
        }
        break;

      //Catch-all for messages we don't care about
      default:
        //Check if the buffer position is not zero
        if (recBufferPos != 0)
        {
          //In process of receiving, return with busy so we can wait for the remaining part of the message
          retVal = INBOUND_BUSY;
        }
        else {
          // Not receiving a message so this message was not important
          messageToProcess = RECV_MSG_GENERIC;
        }
        break;
    }

    //If we processed a message, clear and get ready for the next message
    if (messageToProcess != RECV_MSG_NONE && messageToProcess != RECV_MSG_IPD_MSG) {
      //Reset buffer to start processing next message
      memset(recBuffer, 0, INBOUND_CMD_BUFFER);
      //Reset receive buffer position
      recBufferPos = 0;
      //Reset message to process
      messageToProcess = RECV_MSG_NONE;
    }
  }

  return retVal;
}

void WifiInboundHandler::purgeCurrentCIPSEND() {
  // A CIPSEND was sent but errored... or the client closed just toss it away
  //CommandDistributor::forget(clientPendingCIPSEND);
  DIAG(F("Wifi: DROPPING CIPSEND=%d,%d"), clientPendingCIPSEND, currentReplySize);
  for (int i = 0; i < currentReplySize; i++) {
    outboundRing->read();
  }

  //If CIP SEND was sent, flush out by writing 0s */
  if (cipSendStatus == CIP_SEND_SENT) {
    for (int i = 0; i < currentReplySize; i++) {
      int c = 0;
      wifiStream->write(c);
    }
  }
  // Clear CIP send status
  cipSendStatus = CIP_SEND_NONE;
  clientPendingCIPSEND = -1;
}

#endif
