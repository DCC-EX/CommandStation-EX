/*
 *  © 2021 Mike S
 *  © 2020-2021 Chris Harlow
 *  All rights reserved.
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
#ifndef WiThrottle_h
#define WiThrottle_h

#include "RingStream.h"

struct MYLOCO {
    char throttle; //indicates which throttle letter on client, often '0','1' or '2'
    int cab; //address of this loco
    bool broadcastPending;
    uint32_t functionMap;
    uint32_t functionToggles;
};

class WiThrottle {
  public:  
    static void loop(RingStream * stream);
    void parse(RingStream * stream, byte * cmd);
    static WiThrottle* getThrottle( int wifiClient); 
    static void markForBroadcast(int cab);
    static void forget(byte clientId);
    static void findUniqThrottle(int id, char *u);

  private: 
    WiThrottle( int wifiClientId);
    ~WiThrottle();
   
      static const int MAX_MY_LOCO=10;      // maximum number of locos assigned to a single client
      static const int HEARTBEAT_SECONDS=10; // heartbeat at 10 secs to provide messaging transport
      static const int HEARTBEAT_PRELOAD=2; // request fast callback when connecting multiple messages
      static const int ESTOP_SECONDS=20;     // eStop if no incoming messages for more than 8secs
      static WiThrottle* firstThrottle;
      static int getInt(byte * cmd);
      static int getLocoId(byte * cmd);
      static char LorS(int cab); 
      static bool isThrottleInUse(int cab);
      static void setSendTurnoutList();
      bool areYouUsingThrottle(int cab);
      WiThrottle* nextThrottle;
      int clientid;
      char uniq[17] = "";
       
      MYLOCO myLocos[MAX_MY_LOCO];   
      bool heartBeatEnable;
      unsigned long heartBeat;
      bool introSent=false; 
      bool turnoutsSent=false; 
      bool rosterSent=false; 
      bool routesSent=false; 
      bool heartrateSent=false;
      uint16_t mostRecentCab;
      bool lastPowerState;  // last power state sent to this client

      int DCCToWiTSpeed(int DCCSpeed);
      int WiTToDCCSpeed(int WiTSpeed);
      void multithrottle(RingStream * stream, byte * cmd);
      void locoAction(RingStream * stream, byte* aval, char throttleChar, int cab);
      void accessory(RingStream *, byte* cmd);
      void checkHeartbeat(RingStream * stream); 
      void markForBroadcast2(int cab);
      void sendIntro(Print * stream);
      void sendTurnouts(Print * stream);
      void sendRoster(Print * stream);
      void sendRoutes(Print * stream);
      void sendFunctions(Print* stream, byte loco);
       // callback stuff to support prog track acquire
       static RingStream * stashStream;
       static WiThrottle * stashInstance;
       static byte         stashClient;
       static char         stashThrottleChar;
       static void         getLocoCallback(int16_t locoid);

};
#endif
