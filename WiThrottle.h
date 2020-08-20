/*
 *  © 2020, Chris Harlow. All rights reserved.
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


struct MYLOCO {
    char throttle; //indicates which throttle letter on client, often '0','1' or '2'
    int cab; //address of this loco
};

class WiThrottle {
  public:  
    static void loop();
    void parse(Print & stream, byte * cmd);
    static WiThrottle* getThrottle( int wifiClient); 
    static bool annotateLeftRight;
  private: 
    WiThrottle( int wifiClientId);
    ~WiThrottle();
   
      static const int MAX_MY_LOCO=10; //maximum number of locos assigned to a single client
      static const int HEARTBEAT_TIMEOUT=2;// heartbeat at 2secs to provide messaging transport
      static WiThrottle* firstThrottle;
      static int getInt(byte * cmd);
      static int getLocoId(byte * cmd);
      static char LorS(int cab); 
      static bool isThrottleInUse(int cab);
      static void setSendTurnoutList();
      bool areYouUsingThrottle(int cab);
      WiThrottle* nextThrottle;
      int clientid;
       
      MYLOCO myLocos[MAX_MY_LOCO];   
      bool heartBeatEnable;
      unsigned long heartBeat;
      bool initSent; // valid connection established
      int turnoutListHash;  // used to check for changes to turnout list
      bool lastPowerState;  // last power state sent to this client
      int DCCToWiTSpeed(int DCCSpeed);
      int WiTToDCCSpeed(int WiTSpeed);
      void multithrottle(Print & stream, byte * cmd);
      void locoAction(Print & stream, byte* aval, char throttleChar, int cab);
      void accessory(Print & stream, byte* cmd);
      void checkHeartbeat();  
};
#endif
