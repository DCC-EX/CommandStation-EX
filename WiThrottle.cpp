/*
 *  © 2020, Chris Harlow. All rights reserved.
 *  © 2020, Harald Barth
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
#include "defines.h"
#include "WiThrottle.h"
#include "DCC.h"
#include "DCCWaveform.h"
#include "StringFormatter.h"
#include "Turnouts.h"
#include "DIAG.h"
#include "GITHUB_SHA.h"
#include "version.h"
#include "RMFT2.h"


#define LOOPLOCOS(THROTTLECHAR, CAB)  for (int loco=0;loco<MAX_MY_LOCO;loco++) \
      if ((myLocos[loco].throttle==THROTTLECHAR || '*'==THROTTLECHAR) && (CAB<0 || myLocos[loco].cab==CAB))

WiThrottle * WiThrottle::firstThrottle=NULL;

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
   if (Diag::WITHROTTLE) DIAG(F("%l Creating new WiThrottle for client %d"),millis(),wificlientid); 
   nextThrottle=firstThrottle;
   firstThrottle= this;
   clientid=wificlientid;
   initSent=false; // prevent sending heartbeats before connection completed
   heartBeatEnable=false; // until client turns it on
   turnoutListHash = -1;  // make sure turnout list is sent once
   exRailSent=false;
   mostRecentCab=0;                
   for (int loco=0;loco<MAX_MY_LOCO; loco++) myLocos[loco].throttle='\0';
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

void WiThrottle::parse(RingStream * stream, byte * cmdx) {
  
  byte * cmd=cmdx;
  
  heartBeat=millis();
  if (Diag::WITHROTTLE) DIAG(F("%l WiThrottle(%d)<-[%e]"),millis(),clientid,cmd);

  if (initSent) {
    // Send power state if different than last sent
    bool currentPowerState = (DCCWaveform::mainTrack.getPowerMode()==POWERMODE::ON);
    if (lastPowerState != currentPowerState) {
      StringFormatter::send(stream,F("PPA%x\n"),currentPowerState);
      lastPowerState = currentPowerState;  
    }
    // Send turnout list if changed since last sent (will replace list on client)
    if (turnoutListHash != Turnout::turnoutlistHash) {
      StringFormatter::send(stream,F("PTL"));
      for(Turnout *tt=Turnout::first();tt!=NULL;tt=tt->next()){
          int id=tt->getId();
          StringFormatter::send(stream,F("]\\[%d}|{%d}|{%c"), id, id, Turnout::isClosed(id)?'2':'4');
      }
      StringFormatter::send(stream,F("\n"));
      turnoutListHash = Turnout::turnoutlistHash; // keep a copy of hash for later comparison
    }

    else if (!exRailSent) {
      // Send ExRail routes list if not already sent (but not at same time as turnouts above)
      exRailSent=true;
#ifdef RMFT_ACTIVE
      RMFT2::emitWithrottleRouteList(stream);
#endif    
    }
  }

   while (cmd[0]) {
   switch (cmd[0]) {
       case '*':  // heartbeat control
            if (cmd[1]=='+') heartBeatEnable=true;
            else if (cmd[1]=='-') heartBeatEnable=false;
            break;
       case 'P':  
            if (cmd[1]=='P' && cmd[2]=='A' )  {  //PPA power mode 
              DCCWaveform::mainTrack.setPowerMode(cmd[3]=='1'?POWERMODE::ON:POWERMODE::OFF);
	      if (MotorDriver::commonFaultPin) // commonFaultPin prevents individual track handling
		DCCWaveform::progTrack.setPowerMode(cmd[3]=='1'?POWERMODE::ON:POWERMODE::OFF);
              StringFormatter::send(stream,F("PPA%x\n"),DCCWaveform::mainTrack.getPowerMode()==POWERMODE::ON);
              lastPowerState = (DCCWaveform::mainTrack.getPowerMode()==POWERMODE::ON); //remember power state sent for comparison later
            }
#if defined(RMFT_ACTIVE)
            else if (cmd[1]=='R' && cmd[2]=='A' && cmd[3]=='2' ) { // Route activate
              // exrail routes are RA2Rn , Animations are RA2An 
              int route=getInt(cmd+5);
              uint16_t cab=cmd[4]=='A' ? mostRecentCab : 0; 
              RMFT2::createNewTask(route, cab);
            }
#endif    
            else if (cmd[1]=='T' && cmd[2]=='A') { // PTA accessory toggle 
                int id=getInt(cmd+4); 
                if (!Turnout::exists(id)) {
                  // If turnout does not exist, create it
                  int addr = ((id - 1) / 4) + 1;
                  int subaddr = (id - 1) % 4;
                  DCCTurnout::create(id,addr,subaddr);
                  StringFormatter::send(stream, F("HmTurnout %d created\n"),id);
                }
                switch (cmd[3]) {
            		  // T and C according to RCN-213 where 0 is Stop, Red, Thrown, Diverging.
            		  case 'T': 
            		     Turnout::setClosed(id,false);
                     break;
            		  case 'C': 
                     Turnout::setClosed(id,true);
                     break;
            		  case '2': 
            		     Turnout::setClosed(id,!Turnout::isClosed(id));
                     break;
            		  default :
            		     Turnout::setClosed(id,true);
                     break;
                }
		            StringFormatter::send(stream, F("PTA%c%d\n"),Turnout::isClosed(id)?'2':'4',id );
		        }
            break;
       case 'N':  // Heartbeat (2), only send if connection completed by 'HU' message
            if (initSent) { 
              StringFormatter::send(stream, F("*%d\n"),HEARTBEAT_SECONDS); // return timeout value
            }
            break;
       case 'M': // multithrottle
            multithrottle(stream, cmd); 
            break;
       case 'H': // send initial connection info after receiving "HU" message
            if (cmd[1] == 'U') {
              StringFormatter::send(stream,F("VN2.0\nHTDCC-EX\nRL0\n"));
              StringFormatter::send(stream,F("HtDCC-EX v%S, %S, %S, %S\n"), F(VERSION), F(ARDUINO_TYPE), DCC::getMotorShieldName(), F(GITHUB_SHA));
              StringFormatter::send(stream,F("PTT]\\[Turnouts}|{Turnout]\\[THROW}|{2]\\[CLOSE}|{4\n"));
              StringFormatter::send(stream,F("PPA%x\n"),DCCWaveform::mainTrack.getPowerMode()==POWERMODE::ON);
              lastPowerState = (DCCWaveform::mainTrack.getPowerMode()==POWERMODE::ON); //remember power state sent for comparison later
              StringFormatter::send(stream,F("*%d\n"),HEARTBEAT_SECONDS);
              initSent = true;
            }
            break;           
      case 'Q': // 
            LOOPLOCOS('*', -1) { // tell client to drop any locos still assigned to this WiThrottle
              if (myLocos[loco].throttle!='\0') {
                StringFormatter::send(stream, F("M%c-%c%d<;>\n"), myLocos[loco].throttle, LorS(myLocos[loco].cab), myLocos[loco].cab);
              }
            }
            if (Diag::WITHROTTLE) DIAG(F("%l WiThrottle(%d) Quit"),millis(),clientid);
            delete this; 
            break;           
   }
   // skip over cmd until 0 or past \r or \n
   while(*cmd !='\0' && *cmd != '\r' && *cmd !='\n') cmd++;
   if (*cmd!='\0') cmd++; // skip \r or \n  
   }           
}
int WiThrottle::getInt(byte * cmd) {
    int i=0;
    while (cmd[0]>='0' && cmd[0]<='9') {
      i=i*10 + (cmd[0]-'0');
      cmd++;
    }
    return i;    
}

int WiThrottle::getLocoId(byte * cmd) {
    if (cmd[0]=='*') return -1;  // match all locos 
    if (cmd[0]!='L' && cmd[0]!='S') return 0; // should not match any locos
    return getInt(cmd+1); 
}

void WiThrottle::multithrottle(RingStream * stream, byte * cmd){ 
          char throttleChar=cmd[1];
          int locoid=getLocoId(cmd+3); // -1 for *
          byte * aval=cmd;
          while(*aval !=';' && *aval !='\0') aval++;
          if (*aval) aval+=2;  // skip ;>

//       DIAG(F("Multithrottle aval=%c cab=%d"), aval[0],locoid);    
       switch(cmd[2]) {
          case '+':  // add loco request
                if (cmd[3]=='*') { 
                  // M+* means get loco from prog track, then join tracks ready to drive away
                  // Stash the things the callback will need later
                  stashStream= stream;
                  stashClient=stream->peekTargetMark();
                  stashThrottleChar=throttleChar;
                  stashInstance=this;
                  // ask DCC to call us back when the loco id has been read
                  DCC::getLocoId(getLocoCallback); // will remove any previous join                    
                  return; // return nothing in stream as response is sent later in the callback 
                }
                //return error if address zero requested
                if (locoid==0) { 
                  StringFormatter::send(stream, F("HMAddress '0' not supported!\n"), cmd[3] ,locoid);                    
                  return;
                }
                //return error if L or S from request doesn't match DCC++ assumptions
                if (cmd[3] != LorS(locoid)) { 
                  StringFormatter::send(stream, F("HMLength '%c' not valid for %d!\n"), cmd[3] ,locoid);                    
                  return;
                }
                //use first empty "slot" on this client's list, will be added to DCC registration list
                for (int loco=0;loco<MAX_MY_LOCO;loco++) {
                  if (myLocos[loco].throttle=='\0') { 
                    myLocos[loco].throttle=throttleChar;
                    myLocos[loco].cab=locoid;
                    mostRecentCab=locoid;
                    StringFormatter::send(stream, F("M%c+%c%d<;>\n"), throttleChar, cmd[3] ,locoid); //tell client to add loco
                    //Get known Fn states from DCC 
                    for(int fKey=0; fKey<=28; fKey++) { 
                      int fstate=DCC::getFn(locoid,fKey);
                        if (fstate>=0) StringFormatter::send(stream,F("M%cA%c%d<;>F%d%d\n"),throttleChar,cmd[3],locoid,fstate,fKey);                     
                    }
                    StringFormatter::send(stream, F("M%cA%c%d<;>V%d\n"), throttleChar, cmd[3], locoid, DCCToWiTSpeed(DCC::getThrottleSpeed(locoid)));
                    StringFormatter::send(stream, F("M%cA%c%d<;>R%d\n"), throttleChar, cmd[3], locoid, DCC::getThrottleDirection(locoid));
                    StringFormatter::send(stream, F("M%cA%c%d<;>s1\n"), throttleChar, cmd[3], locoid); //default speed step 128
                    return;
                  }
               }
               StringFormatter::send(stream, F("HMMax locos (%d) exceeded, %d not added!\n"), MAX_MY_LOCO ,locoid);                    
               break;
          case '-': // remove loco(s) from this client (leave in DCC registration)
                 LOOPLOCOS(throttleChar, locoid) {
                     myLocos[loco].throttle='\0';
                     StringFormatter::send(stream, F("M%c-%c%d<;>\n"), throttleChar, LorS(myLocos[loco].cab), myLocos[loco].cab);
                  }
            
            break;
          case 'A':
              locoAction(stream,aval, throttleChar, locoid);
            }
}

void WiThrottle::locoAction(RingStream * stream, byte* aval, char throttleChar, int cab){
    // Note cab=-1 for all cabs in the consist called throttleChar.  
//    DIAG(F("Loco Action aval=%c%c throttleChar=%c, cab=%d"), aval[0],aval[1],throttleChar, cab);
     switch (aval[0]) {
           case 'V':  // Vspeed
             { 
              int witSpeed=getInt(aval+1);
              LOOPLOCOS(throttleChar, cab) {
                mostRecentCab=myLocos[loco].cab;
                DCC::setThrottle(myLocos[loco].cab, WiTToDCCSpeed(witSpeed), DCC::getThrottleDirection(myLocos[loco].cab));
                StringFormatter::send(stream,F("M%cA%c%d<;>V%d\n"), throttleChar, LorS(myLocos[loco].cab), myLocos[loco].cab, witSpeed);
                }
             } 
            break;
           case 'F': //F onOff function
              {
		            bool funcstate;
                bool pressed=aval[1]=='1';
                int fKey = getInt(aval+2);
                LOOPLOCOS(throttleChar, cab) {
		              funcstate = DCC::changeFn(myLocos[loco].cab, fKey, pressed);
		              if(funcstate==0 || funcstate==1)
			              StringFormatter::send(stream,F("M%cA%c%d<;>F%d%d\n"), throttleChar, LorS(myLocos[loco].cab), 
						          myLocos[loco].cab, funcstate, fKey);
		              }
                }
                break;  
            case 'q':
                if (aval[1]=='V') {   //qV
                  LOOPLOCOS(throttleChar, cab) {              
                    StringFormatter::send(stream,F("M%cA%c%d<;>V%d\n"), throttleChar, LorS(myLocos[loco].cab), myLocos[loco].cab, DCCToWiTSpeed(DCC::getThrottleSpeed(myLocos[loco].cab)));
                  }              
                }
                else if (aval[1]=='R') { // qR
                  LOOPLOCOS(throttleChar, cab) {              
                    StringFormatter::send(stream,F("M%cA%c%d<;>R%d\n"), throttleChar, LorS(myLocos[loco].cab), myLocos[loco].cab, DCC::getThrottleDirection(myLocos[loco].cab));
                  }             
                }     
            break;    
            case 'R':
            { 
              bool forward=aval[1]!='0';
              LOOPLOCOS(throttleChar, cab) {
                mostRecentCab=myLocos[loco].cab;
                DCC::setThrottle(myLocos[loco].cab, DCC::getThrottleSpeed(myLocos[loco].cab), forward);
                StringFormatter::send(stream,F("M%cA%c%d<;>R%d\n"), throttleChar, LorS(myLocos[loco].cab), myLocos[loco].cab, forward);
              }
            }        
            break;      
            case 'X':
              //Emergency Stop  (speed code 1)
              LOOPLOCOS(throttleChar, cab) {
                DCC::setThrottle(myLocos[loco].cab, 1, DCC::getThrottleDirection(myLocos[loco].cab));
                StringFormatter::send(stream,F("M%cA%c%d<;>V%d\n"), throttleChar, LorS(myLocos[loco].cab), myLocos[loco].cab, -1);
              }
              break;
            case 'I': // Idle, set speed to 0
            case 'Q': // Quit, set speed to 0
              LOOPLOCOS(throttleChar, cab) {
                mostRecentCab=myLocos[loco].cab;
                DCC::setThrottle(myLocos[loco].cab, 0, DCC::getThrottleDirection(myLocos[loco].cab));
                StringFormatter::send(stream,F("M%cA%c%d<;>V%d\n"), throttleChar, LorS(myLocos[loco].cab), myLocos[loco].cab, 0);
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

void WiThrottle::loop(RingStream * stream) {
  // for each WiThrottle, check the heartbeat
  for (WiThrottle* wt=firstThrottle; wt!=NULL ; wt=wt->nextThrottle) 
     wt->checkHeartbeat();

   // TODO... any broadcasts to be done 
   (void)stream; 
   /* MUST follow this model in  this loop. 
    *   stream->mark();
    *   send 1 digit client id, and any data 
    *   stream->commit() 
     */

}

void WiThrottle::checkHeartbeat() {
  // if eStop time passed... eStop any locos still assigned to this client and then drop the connection
  if(heartBeatEnable && (millis()-heartBeat > ESTOP_SECONDS*1000)) {
  if (Diag::WITHROTTLE)  DIAG(F("%l WiThrottle(%d) eStop(%ds) timeout, drop connection"), millis(), clientid, ESTOP_SECONDS);
    LOOPLOCOS('*', -1) { 
      if (myLocos[loco].throttle!='\0') {
        if (Diag::WITHROTTLE) DIAG(F("%l  eStopping cab %d"),millis(),myLocos[loco].cab);
        DCC::setThrottle(myLocos[loco].cab, 1, DCC::getThrottleDirection(myLocos[loco].cab)); // speed 1 is eStop
      }
    }
    delete this;
   }
}

char WiThrottle::LorS(int cab) {
    return (cab<127)?'S':'L';
}

// Drive Away feature. Callback handling
 
RingStream * WiThrottle::stashStream;
WiThrottle * WiThrottle::stashInstance;
byte         WiThrottle::stashClient;
char         WiThrottle::stashThrottleChar;

void WiThrottle::getLocoCallback(int16_t locoid) {
  stashStream->mark(stashClient);
  if (locoid<0) StringFormatter::send(stashStream,F("HMNo loco found on prog track\n"));
  else {
    char addcmd[20]={'M',stashThrottleChar,'+',LorS(locoid) };
    itoa(locoid,addcmd+4,10);
    stashInstance->multithrottle(stashStream, (byte *)addcmd);
    DCCWaveform::progTrack.setPowerMode(POWERMODE::ON);
    DCC::setProgTrackSyncMain(true);  // <1 JOIN> so we can drive loco away
  }
  stashStream->commit();
}
