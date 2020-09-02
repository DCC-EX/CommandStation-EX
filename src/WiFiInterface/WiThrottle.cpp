/*
 *  WiThrottle.cpp
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


/*
 * Truncated JMRI WiThrottle server implementation for DCC-EX command station
 * Credit is due to  Valerie Valley RR https://sites.google.com/site/valerievalleyrr/
 *  for showing how it could be done, but this code is very different to the original
 *  implementation as it is designed to run on the Arduino and not the ESP and is
 *  also calling directly into the DCCEX Api rather than  simulating JMRI text commands.
 * Refer JMRI WiFi Throttle Communications Protocol https://www.jmri.org/help/en/package/jmri/jmrit/withrottle/Protocol.shtml
 * 
 * 
 * PROTOTYPE NOTES:
 *  There will be one WiThrottle instance created for each WiThrottle client detected by the WifiInterface.
 *  Some shortcuts have been taken and there are some things that are yet to be included:
 *  e.g. Full response to adding a loco.
 *  What to do about unknown turnouts.
 *  Broadcasting to other WiThrottles when things change.  
 *    -  Bear in mind that changes may have taken place due to  
 *      other WiThrottles, OR JMRI commands received OR TPL automation. 
 *    - I suggest that at the end of parse(), then anything that has changed and is of interest could  
 *       be notified then.  (e.g loco speeds, directions or functions, turnout states.
 *       
 *  WiThrottle.h sets the max locos per client at 10, this is ok to increase but requires just an extra 3 bytes per loco per client.      
*/

#include <Arduino.h>
#include "WiThrottle.h"
#include "../CommInterface/CommManager.h"
#include "../Accessories/Turnouts.h"
#include "../Utils/DIAG.h"

DCC* WiThrottle::mainTrack;
DCC* WiThrottle::progTrack;

                                             /* cast fixes compiler warning, CAB is always > 0 in last compare */
#define LOOPLOCOS(THROTTLECHAR, CAB)  for (int loco=0;loco<kMaxLocosPerThrottle;loco++) \
  if ((myLocos[loco].throttle==THROTTLECHAR || '*'==THROTTLECHAR) && (CAB<0 || myLocos[loco].cab==(uint16_t)CAB))

WiThrottle * WiThrottle::firstThrottle=NULL;
bool WiThrottle::annotateLeftRight=false;

void WiThrottle::setup(DCC* _mainTrack, DCC* _progTrack) {
  mainTrack = _mainTrack;
  progTrack = _progTrack;
}

WiThrottle* WiThrottle::getThrottle( int wifiClient) {
  for (WiThrottle* wt=firstThrottle; wt!=NULL ; wt=wt->nextThrottle)  
    if (wt->clientid==wifiClient) return wt; 
      return new WiThrottle( wifiClient);
}

bool WiThrottle::isThrottleInUse(int cab) {
  for (WiThrottle* wt=firstThrottle; wt!=NULL ; wt=wt->nextThrottle)  
    if (wt->areYouUsingThrottle(cab)) return true;
      return false;
}

bool WiThrottle::areYouUsingThrottle(int cab) {
  LOOPLOCOS('*', cab) { // see if I have this cab in use
    return true;
  }
  return false;
}

// One instance of WiThrottle per connected client, so we know what the locos are  
WiThrottle::WiThrottle( int wificlientid) {
  DIAG(F("\nNew WiThrottle for client %d\n"),wificlientid); 
  nextThrottle=firstThrottle;
  firstThrottle= this;
  clientid=wificlientid;
  initSent=false; // prevent sending heartbeats before connection completed
  heartBeatEnable=false; // until client turns it on
  turnoutListHash = -1;  // make sure turnout list is sent once
  for (int loco=0;loco<kMaxLocosPerThrottle; loco++) myLocos[loco].throttle='\0';
}

WiThrottle::~WiThrottle() {
  if (firstThrottle== this) {
    firstThrottle=this->nextThrottle;
    return;
  }
  for (WiThrottle* wt=firstThrottle; wt!=NULL ; wt=wt->nextThrottle) {
    if (wt->nextThrottle==this) {
      wt->nextThrottle=this->nextThrottle;
      return;  
    }
  }
}

void WiThrottle::parse(Print * stream, uint8_t * cmdx) {
  // we have to take a copy of the cmd buffer as the reply will get built into the cmdx  
  uint8_t local[150];
  for (uint8_t i=0;i<sizeof(local);i++) {
    local[i]=cmdx[i];
    if (!cmdx[i]) break;
  }
  
  local[149]='\0'; // prevent runaway parser
  
  uint8_t * cmd = local;
  
  heartBeat = millis();
  //  DIAG(F("\nWiThrottle(%d)<-[%e]\n"),clientid, cmd);

  if (initSent) {
    // Send power state if different than last sent
    bool currentPowerState = mainTrack->board->getStatus();
    if (lastPowerState != currentPowerState) {
      CommManager::send(stream, F("PPA%x\n"), currentPowerState);
      lastPowerState = currentPowerState;  
    }
    // Send turnout list if changed since last sent (will replace list on client)
    if (turnoutListHash != Turnout::turnoutlistHash) {
      CommManager::send(stream,F("PTL"));
      for(Turnout *tt=Turnout::firstTurnout;tt!=NULL;tt=tt->nextTurnout){
        CommManager::send(stream,F("]\\[%d}|{%d}|{%c"), tt->data.id, tt->data.id, tt->data.tStatus?'4':'2');
      }
      CommManager::send(stream,F("\n"));
      turnoutListHash = Turnout::turnoutlistHash; // keep a copy of hash for later comparison
    }
  }

   while (cmd[0]) {
   switch (cmd[0]) {
     case '*':  // heartbeat control
      if (cmd[1]=='+') heartBeatEnable=true;
      else if (cmd[1]=='-') heartBeatEnable=false;
      break;
     case 'P':  
      if (cmd[1]=='P' && cmd[2]=='A' )  {  // PPA power mode 
        mainTrack->board->power(cmd[3] == '1' ? ON : OFF, true);
        CommManager::send(stream,F("PPA%x\n"),mainTrack->board->getStatus()==ON);
        lastPowerState = (mainTrack->board->getStatus()==ON); //remember power state sent for comparison later
      }
      else if (cmd[1]=='T' && cmd[2]=='A') { // PTA accessory toggle 
        int id=getInt(cmd+4); 
        bool newstate=false;
        Turnout* tt=Turnout::get(id);
        if (!tt) {
          // If turnout does not exist, create it
          int addr = ((id - 1) / 4) + 1;
          int subaddr = (id - 1) % 4;
          Turnout::create(&DIAGSERIAL, id, addr, subaddr);
          CommManager::send(stream, F("HmTurnout %d created\n"), id);
        }
        switch (cmd[3]) {
          case 'T': newstate=true; break;
          case 'C': newstate=false; break;
          case '2': newstate=!tt->data.tStatus;                 
        }
        tt->activate(&DIAGSERIAL, id, mainTrack);
        CommManager::send(stream, F("PTA%c%d\n"),newstate?'4':'2',id );   
      }
      break;
     case 'N':  // Heartbeat (2), only send if connection completed by 'HU' message
      if (initSent) { 
        CommManager::send(stream, F("*%d\n"),kHeartbeatTimeout); // return timeout value
      }
      break;
     case 'M': // multithrottle
      multithrottle(stream, cmd); 
      break;
     case 'H': // send initial connection info after receiving "HU" message
      if (cmd[1] == 'U') {
        CommManager::send(stream,F("VN2.0\nHTDCC++EX\nRL0\n"));
        if (annotateLeftRight) CommManager::send(stream,F("PTT]\\[Turnouts}|{Turnout]\\[Left}|{2]\\[Right}|{4\n"));
        else                   CommManager::send(stream,F("PTT]\\[Turnouts}|{Turnout]\\[Closed}|{2]\\[Thrown}|{4\n"));
        CommManager::send(stream,F("PPA%x\n"),mainTrack->board->getStatus()==ON);
        lastPowerState = (mainTrack->board->getStatus()==ON); //remember power state sent for comparison later
        CommManager::send(stream,F("*%d\n"),kHeartbeatTimeout);
        initSent = true;
      }
      break;           
    case 'Q': // 
      LOOPLOCOS('*', -1) { // tell client to drop any locos still assigned to this WiThrottle
        if (myLocos[loco].throttle!='\0') {
        CommManager::send(stream, F("M%c-%c%d<;>\n"), myLocos[loco].throttle, LorS(myLocos[loco].cab), myLocos[loco].cab);
        }
      }
      DIAG(F("WiThrottle(%d) Quit\n"), clientid);
      delete this; 
      break;           
   }
   // skip over cmd until 0 or past \r or \n
   while(*cmd !='\0' && *cmd != '\r' && *cmd !='\n') cmd++;
   if (*cmd!='\0') cmd++; // skip \r or \n  
   }           
}
int WiThrottle::getInt(uint8_t * cmd) {
  int i=0;
  while (cmd[0]>='0' && cmd[0]<='9') {
    i=i*10 + (cmd[0]-'0');
    cmd++;
  }
  return i;    
}

int WiThrottle::getLocoId(uint8_t * cmd) {
  if (cmd[0]=='*') return -1;  // match all locos 
  if (cmd[0]!='L' && cmd[0]!='S') return 0; // should not match any locos
  return getInt(cmd+1); 
}

void WiThrottle::multithrottle(Print * stream, uint8_t * cmd){ 
  char throttleChar=cmd[1];
  int locoid=getLocoId(cmd+3); // -1 for *
  uint8_t * aval=cmd;
  while(*aval !=';' && *aval !='\0') aval++;
  if (*aval) aval+=2;  // skip ;>

//  DIAG(F("\nMultithrottle aval=%c cab=%d"), aval[0],locoid);    
  switch(cmd[2]) {
  case '+':  // add loco request
    //return error if address zero requested
    if (locoid==0) { 
      CommManager::send(stream, F("HMAddress '0' not supported!\n"), cmd[3] ,locoid);                    
      return;
    }
    //return error if L or S from request doesn't match DCC++ assumptions
    if (cmd[3] != LorS(locoid)) { 
      CommManager::send(stream, F("HMLength '%c' not valid for %d!\n"), cmd[3] ,locoid);                    
      return;
    }
    //use first empty "slot" on this client's list, will be added to DCC registration list
    for (int loco=0;loco<kMaxLocosPerThrottle;loco++) {
      if (myLocos[loco].throttle=='\0') { 
      myLocos[loco].throttle=throttleChar;
      myLocos[loco].cab=locoid;
      CommManager::send(stream, F("M%c+%c%d<;>\n"), throttleChar, cmd[3] ,locoid); //tell client to add loco
      // TODO... get known Fn states from DCC (need memoryStream improvements to handle data length)
      // for(fKey=0; fKey<29; fKey++)CommManager::send(stream,F("M%cA%c<;>F0&s\n"),throttleChar,cmd[3],fkey);
      CommManager::send(stream, F("M%cA%c%d<;>V%d\n"), throttleChar, cmd[3], locoid, DCCToWiTSpeed(mainTrack->getThrottleSpeed(locoid)));
      CommManager::send(stream, F("M%cA%c%d<;>R%d\n"), throttleChar, cmd[3], locoid, mainTrack->getThrottleDirection(locoid));
      CommManager::send(stream, F("M%cA%c%d<;>s1\n"), throttleChar, cmd[3], locoid); //default speed step 128
      return;
      }
      }
      CommManager::send(stream, F("HMMax locos (%d) exceeded, %d not added!\n"), kMaxLocosPerThrottle ,locoid);                    
      break;
  case '-': // remove loco(s) from this client (leave in DCC registration)
      LOOPLOCOS(throttleChar, locoid) {
        myLocos[loco].throttle='\0';
        CommManager::send(stream, F("M%c-%c%d<;>\n"), throttleChar, LorS(myLocos[loco].cab), myLocos[loco].cab);
      }
  
  break;
  case 'A':
    locoAction(stream,aval, throttleChar, locoid);
  }
}

void WiThrottle::locoAction(Print * stream, uint8_t* aval, char throttleChar, int cab){
  // Note cab=-1 for all cabs in the consist called throttleChar.  
//    DIAG(F("\nLoco Action aval=%c%c throttleChar=%c, cab=%d"), aval[0],aval[1],throttleChar, cab);
  
  genericResponse response;

  switch (aval[0]) {
  case 'V':  // Vspeed
    { 
    uint8_t locospeed=WiTToDCCSpeed(getInt(aval+1));
    
    LOOPLOCOS(throttleChar, cab) {
      uint8_t speedCode = mainTrack->speedAndDirToCode(locospeed, mainTrack->getThrottleDirection(myLocos[loco].cab));
      mainTrack->setThrottle(myLocos[loco].cab, speedCode, response);
      CommManager::send(stream,F("M%cA%c%d<;>V%d\n"), throttleChar, LorS(myLocos[loco].cab), myLocos[loco].cab, locospeed);
    }
    } 
    break;
  case 'F': //F onOff function
    {
    bool funcstate;
    bool pressed = aval[1] == '1';
    int fKey = getInt(aval + 2);
    LOOPLOCOS(throttleChar, cab) {
      funcstate = mainTrack->changeFunction(myLocos[loco].cab, fKey, pressed);
      
      if(true)
        if(funcstate==0 || funcstate==1)
          CommManager::send(stream,F("M%cA%c%d<;>F%d%d\n"), throttleChar, LorS(myLocos[loco].cab), myLocos[loco].cab, funcstate, fKey);
    }
    }
    break;  
  case 'q':
    if (aval[1]=='V') {   //qV
      LOOPLOCOS(throttleChar, cab) {              
        CommManager::send(stream,F("M%cA%c%d<;>V%d\n"), throttleChar, LorS(myLocos[loco].cab), myLocos[loco].cab, DCCToWiTSpeed(mainTrack->getThrottleSpeed(myLocos[loco].cab)));
      }              
    }
    else if (aval[1]=='R') { // qR
      LOOPLOCOS(throttleChar, cab) {              
        CommManager::send(stream,F("M%cA%c%d<;>R%d\n"), throttleChar, LorS(myLocos[loco].cab), myLocos[loco].cab, mainTrack->getThrottleDirection(myLocos[loco].cab));
      }             
    }     
    break;    
  case 'R':
    { 
    bool forward=aval[1]!='0';
    LOOPLOCOS(throttleChar, cab) {              
      uint8_t speedCode = mainTrack->speedAndDirToCode(mainTrack->getThrottleSpeed(myLocos[loco].cab), forward);
      mainTrack->setThrottle(myLocos[loco].cab, speedCode, response);
      CommManager::send(stream,F("M%cA%c%d<;>R%d\n"), throttleChar, LorS(myLocos[loco].cab), myLocos[loco].cab, forward);
    }
    }        
    break;      
  case 'X':
    //Emergency Stop  (speed code 1)
    LOOPLOCOS(throttleChar, cab) {
      mainTrack->setThrottle(myLocos[loco].cab, 1, response);
      CommManager::send(stream,F("M%cA%c%d<;>V%d\n"), throttleChar, LorS(myLocos[loco].cab), myLocos[loco].cab, -1);
    }
    break;
  case 'I': // Idle, set speed to 0
  case 'Q': // Quit, set speed to 0
    LOOPLOCOS(throttleChar, cab) {
      mainTrack->setThrottle(myLocos[loco].cab, 0, response);
      CommManager::send(stream,F("M%cA%c%d<;>V%d\n"), throttleChar, LorS(myLocos[loco].cab), myLocos[loco].cab, 0);
    }
    break;
  }               
}

  // convert between DCC++ speed values and WiThrottle speed values
int WiThrottle::DCCToWiTSpeed(int DCCSpeed) {
  if (DCCSpeed == 0) return 0; //stop is stop
  if (DCCSpeed == 1) return -1; //eStop value
  return DCCSpeed - 1; //offset others by 1
}

  // convert between WiThrottle speed values and DCC++ speed values
int WiThrottle::WiTToDCCSpeed(int WiTSpeed) {
  if (WiTSpeed == 0) return 0;  //stop is stop
  if (WiTSpeed == -1) return 1; //eStop value
  return WiTSpeed + 1; //offset others by 1
}

void WiThrottle::loop() {
  // for each WiThrottle, check the heartbeat
  for (WiThrottle* wt=firstThrottle; wt!=NULL ; wt=wt->nextThrottle) 
   wt->checkHeartbeat();
}

void WiThrottle::checkHeartbeat() {
  // if 2 heartbeats missed... drop connection and eStop any locos still assigned to this client
  if(heartBeatEnable && (millis()-heartBeat > kHeartbeatTimeout*2000)) {
    DIAG(F("\n\nWiThrottle(%d) HB missed, severing conn\n\n"),clientid);
    genericResponse response;
    LOOPLOCOS('*', -1) { 
      if (myLocos[loco].throttle!='\0') {
        DIAG(F("eStopping cab %d\n"), myLocos[loco].cab);
        mainTrack->setThrottle(myLocos[loco].cab, 1, response); // speed 1 is eStop
      }
    }
    delete this;
  } else {
    // TODO  Check if anything has changed on my locos since last notified! 
  }
}

char WiThrottle::LorS(int cab) {
  return (cab<127)?'S':'L';
}
