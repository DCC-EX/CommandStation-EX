/*
    © 2022 Mark Muzzin
    © 2021 Harald Barth
    © 2021 Fred Decker
    (c) 2021 Fred Decker.  All rights reserved.
    (c) 2020 Chris Harlow. All rights reserved.

    This file is part of CommandStation-EX

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

    enum INBOUND_STATE : byte {
      INBOUND_BUSY,      // Keep calling in loop()
      INBOUND_IDLE       // Nothing happening, outbound may xcall CIPSEND
    };

    enum RECV_MSG_STATE : byte {
      RECV_MSG_NONE,     // No message to process
      RECV_MSG_CRLF,     // Message ending in CRLF
      RECV_MSG_SND_DATA, // Send data message
      RECV_MSG_IPD_MSG,  // +IPD Message in progress
      RECV_MSG_IPD_DONE, // +IPD Message done
      RECV_MSG_GENERIC   // Catchall for incoming messages not used
    };

    enum CIP_SEND_STATUS : byte {
      CIP_SEND_NONE,
      CIP_SEND_PENDING,
      CIP_SEND_SENT,
      CIP_SEND_DATA_SENT
    };

    WifiInboundHandler(Stream * ESStream);
    void loop1();
    INBOUND_STATE loop2();
    void purgeCurrentCIPSEND();
    int antoi(char* str, int len);
    Stream * wifiStream;

    static const int INBOUND_CMD_BUFFER = 32;
    static const int INBOUND_RING = 512;
    static const int OUTBOUND_RING = 2048;

    RingStream * inboundRing;
    RingStream * outboundRing;

    RECV_MSG_STATE messageToProcess = RECV_MSG_NONE; // Track the current message to process

    int runningClientId;   // latest client inbound processing data or CLOSE
    int dataLength;        // dataLength of +IPD
    int dataRemaining = 0; // remaining +IPD data to receive
    int clientPendingCIPSEND = -1;
    int currentReplySize;
    CIP_SEND_STATUS cipSendStatus;
    char recBuffer[INBOUND_CMD_BUFFER] = {'\0'};
    int recBufferPos = 0;
    int recBufferWatermark = 0;
};
#endif
