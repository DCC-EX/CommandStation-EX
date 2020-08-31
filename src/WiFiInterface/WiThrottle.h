/*
 *  WiThrottle.h
 * 
 *  This file is part of CommandStation-EX.
 *
 *  CommandStation-EX is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  CommandStation-EX is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with CommandStation-EX.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef WIFIINTERFACE_WITHROTTLE_H_
#define WIFIINTERFACE_WITHROTTLE_H_

#include "../DCC/DCC.h"

struct LOCO {
  char throttle; //indicates which throttle letter on client, often '0','1' or '2'
  uint16_t cab; //address of this loco
};

class WiThrottle {
public:  
  static void setup(DCC*, DCC*);  // Call in the main file to associate the tracks
  static void loop();
  void parse(Print * stream, byte * cmd);
  static WiThrottle* getThrottle( int wifiClient); 
  static bool annotateLeftRight;
private: 
  WiThrottle(int wifiClientId);
  ~WiThrottle();

  static DCC* mainTrack;
  static DCC* progTrack;
   
  static const int kMaxLocosPerThrottle = 10; //maximum number of locos assigned to a single client
  static const int kHeartbeatTimeout = 2; // heartbeat at 2secs to provide messaging transport
  static WiThrottle* firstThrottle;
  static int getInt(uint8_t * cmd);
  static int getLocoId(uint8_t * cmd);
  static char LorS(int cab); 
  static bool isThrottleInUse(int cab);
  static void setSendTurnoutList();
  bool areYouUsingThrottle(int cab);
  WiThrottle* nextThrottle;
  int clientid;
    
  LOCO myLocos[kMaxLocosPerThrottle];   
  bool heartBeatEnable;
  unsigned long heartBeat;
  bool initSent; // valid connection established
  int turnoutListHash;  // used to check for changes to turnout list
  bool lastPowerState;  // last power state sent to this client
  int DCCToWiTSpeed(int DCCSpeed);
  int WiTToDCCSpeed(int WiTSpeed);
  void multithrottle(Print * stream, uint8_t * cmd);
  void locoAction(Print * stream, uint8_t * aval, char throttleChar, int cab);
  void accessory(Print * stream, uint8_t* cmd);
  void checkHeartbeat();  
};

#endif  // WIFIINTERFACE_WITHROTTLE_H_