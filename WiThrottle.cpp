/*
 *  © 2021 Neil McKechnie
 *  © 2021 Mike S
 *  © 2020-2022 Harald Barth
 *  © 2020-2021 M Steve Todd
 *  © 2020-2021 Chris Harlow
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
#include "EXRAIL2.h"
#include "CommandDistributor.h"
#include "TrackManager.h"
#include "DCCTimer.h"

#define LOOPLOCOS(THROTTLECHAR, CAB)  for (int loco=0;loco<MAX_MY_LOCO;loco++) \
      if ((myLocos[loco].throttle==THROTTLECHAR || '*'==THROTTLECHAR) && (CAB<0 || myLocos[loco].cab==CAB))

WiThrottle * WiThrottle::firstThrottle=NULL;

WiThrottle* WiThrottle::getThrottle( int wifiClient) {
  for (WiThrottle* wt=firstThrottle; wt!=NULL ; wt=wt->nextThrottle)  
     if (wt->clientid==wifiClient) return wt; 
  return new WiThrottle( wifiClient);
}

void WiThrottle::forget( byte clientId) {
  for (WiThrottle* wt=firstThrottle; wt!=NULL ; wt=wt->nextThrottle)  
     if (wt->clientid==clientId) {
      DIAG(F("Withrottle client %d dropped"),clientId);
      delete wt;
      break; 
     }
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
   heartBeatEnable=false; // until client turns it on
   mostRecentCab=0;                
   for (int loco=0;loco<MAX_MY_LOCO; loco++) myLocos[loco].throttle='\0';
}

WiThrottle::~WiThrottle() {
  if (Diag::WITHROTTLE) DIAG(F("Deleting WiThrottle client %d"),this->clientid);
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
  
  // On first few commands, send turnout, roster and routes 
  if (introSent) {  
    if (!turnoutsSent) sendTurnouts(stream);
    else if(!rosterSent) sendRoster(stream);
    else if (!routesSent) sendRoutes(stream);
    else if (!heartrateSent) {
         heartrateSent=true;
        // allow heartbeat to slow down once all metadata sent     
        StringFormatter::send(stream,F("*%d\nHMConnected\n"),HEARTBEAT_SECONDS);

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
	TrackManager::setMainPower(cmd[3]=='1'?POWERMODE::ON:POWERMODE::OFF);
/* TODO 
	if (MotorDriver::commonFaultPin) // commonFaultPin prevents individual track handling
	  DCCWaveform::progTrack.setPowerMode(cmd[3]=='1'?POWERMODE::ON:POWERMODE::OFF);
*/

      }
#if defined(EXRAIL_ACTIVE)
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
      sendIntro(stream);
      StringFormatter::send(stream, F("*%d\n"), heartrateSent ? HEARTBEAT_SECONDS : HEARTBEAT_PRELOAD); // return timeout value
      break;
    case 'M': // multithrottle
      multithrottle(stream, cmd); 
      break;
    case 'H': // send initial connection info after receiving "HU" message
      if (cmd[1] == 'U') {    
	sendIntro(stream);
      }
      break;           
    case 'Q': // 
      LOOPLOCOS('*', -1) { // tell client to drop any locos still assigned to this WiThrottle
	if (myLocos[loco].throttle!='\0') {
	  StringFormatter::send(stream, F("M%c-%c%d<;>\n"), myLocos[loco].throttle, LorS(myLocos[loco].cab), myLocos[loco].cab);
	}
      }
      if (Diag::WITHROTTLE) DIAG(F("WiThrottle(%d) Quit"),clientid);
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
  bool negate=cmd[0]=='-';
  if (negate) cmd++;
  while (cmd[0]>='0' && cmd[0]<='9') {
    i=i*10 + (cmd[0]-'0');
    cmd++;
  }
  if (negate) i=0-i;
  return i ;    
}

int WiThrottle::getLocoId(byte * cmd) {
    if (cmd[0]=='*') return -1;  // match all locos 
    if (cmd[0]!='L' && cmd[0]!='S') return 0; // should not match any locos
    return getInt(cmd+1); 
}

void WiThrottle::multithrottle(RingStream * stream, byte * cmd){ 
  char throttleChar=cmd[1];
  int locoid=getLocoId(cmd+3); // -1 for *
  if (locoid > 10239 || locoid < -1) {
    StringFormatter::send(stream, F("No valid DCC loco %d\n"), locoid);
    return;
  }
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
	      myLocos[loco].functionMap=DCC::getFunctionMap(locoid); 
	      myLocos[loco].broadcastPending=true; // means speed/dir will be sent later
	      mostRecentCab=locoid;
	      StringFormatter::send(stream, F("M%c+%c%d<;>\n"), throttleChar, cmd[3] ,locoid); //tell client to add loco
	      sendFunctions(stream,loco);
	      //speed and direction will be published at next broadcast cycle
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
  (void) stream;
  switch (aval[0]) {
  case 'V':  // Vspeed
    { 
      int witSpeed=getInt(aval+1);
      LOOPLOCOS(throttleChar, cab) {
	mostRecentCab=myLocos[loco].cab;
	DCC::setThrottle(myLocos[loco].cab, WiTToDCCSpeed(witSpeed), DCC::getThrottleDirection(myLocos[loco].cab));
	// SetThrottle will cause speed change broadcast
      }
    } 
    break;
  case 'F': // Function key pressed/released
    {  
      bool pressed=aval[1]=='1';
      int fKey = getInt(aval+2);
      LOOPLOCOS(throttleChar, cab) {
	bool unsetOnRelease = myLocos[loco].functionToggles & (1L<<fKey);
	if (unsetOnRelease) DCC::setFn(myLocos[loco].cab,fKey, pressed);
	else if (pressed)  DCC::changeFn(myLocos[loco].cab, fKey);
      }
      break;  
    }
  case 'q':
    if (aval[1]=='V' || aval[1]=='R' ) {   //qV or qR
      // just flag the loco for broadcast and it will happen.
      bool foundone = false;
      LOOPLOCOS(throttleChar, cab) {
	foundone = true;
	myLocos[loco].broadcastPending=true;
      }
      if (!foundone)
	StringFormatter::send(stream,F("HMCS loco list empty\n"));
    }     
    break;    
  case 'R':
    { 
      bool forward=aval[1]!='0';
      LOOPLOCOS(throttleChar, cab) {
	mostRecentCab=myLocos[loco].cab;
	int8_t speed = DCC::getThrottleSpeed(myLocos[loco].cab);
	if (speed < 0) //can not find any speed for this cab
	  speed = 0;
	DCC::setThrottle(myLocos[loco].cab, speed, forward);
	// setThrottle will cause a broadcast so notification will be sent
      }
    }        
    break;      
  case 'X':
    //Emergency Stop  (speed code 1)
    LOOPLOCOS(throttleChar, cab) {
      DCC::setThrottle(myLocos[loco].cab, 1, DCC::getThrottleDirection(myLocos[loco].cab));
      // setThrottle will cause a broadcast so notification will be sent
    }
    break;
  case 'I': // Idle, set speed to 0
  case 'Q': // Quit, set speed to 0
    LOOPLOCOS(throttleChar, cab) {
      mostRecentCab=myLocos[loco].cab;
      DCC::setThrottle(myLocos[loco].cab, 0, DCC::getThrottleDirection(myLocos[loco].cab));
      // setThrottle will cause a broadcast so notification will be sent
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
  // for each WiThrottle, check the heartbeat and broadcast needed
  for (WiThrottle* wt=firstThrottle; wt!=NULL ; wt=wt->nextThrottle) 
    wt->checkHeartbeat(stream);
}

void WiThrottle::checkHeartbeat(RingStream * stream) {
  // if eStop time passed... eStop any locos still assigned to this client and then drop the connection
  if(heartBeatEnable && (millis()-heartBeat > ESTOP_SECONDS*1000)) {
    if (Diag::WITHROTTLE)  DIAG(F("%l WiThrottle(%d) eStop(%ds) timeout, drop connection"), millis(), clientid, ESTOP_SECONDS);
    LOOPLOCOS('*', -1) { 
      if (myLocos[loco].throttle!='\0') {
        if (Diag::WITHROTTLE) DIAG(F("%l  eStopping cab %d"),millis(),myLocos[loco].cab);
        DCC::setThrottle(myLocos[loco].cab, 1, DCC::getThrottleDirection(myLocos[loco].cab)); // speed 1 is eStop
	heartBeat=millis(); // We have just stopped everyting, we don't need to do that again at next loop.
      }
    }
    // if it does come back, the throttle should re-acquire 
    delete this;
    return;
  }
   
   // send any outstanding speed/direction/function changes for this clients locos
   // Changes may have been caused by this client, or another non-Withrottle or Exrail
  bool streamHasBeenMarked=false; 
  LOOPLOCOS('*', -1) { 
    if (myLocos[loco].throttle!='\0' && myLocos[loco].broadcastPending) {
      if (!streamHasBeenMarked) {
	stream->mark(clientid);
	streamHasBeenMarked=true;
      }
      myLocos[loco].broadcastPending=false;
      int cab=myLocos[loco].cab;
      char lors=LorS(cab);
      char throttle=myLocos[loco].throttle;
      StringFormatter::send(stream,F("M%cA%c%d<;>V%d\n"),
			    throttle, lors , cab, DCCToWiTSpeed(DCC::getThrottleSpeed(cab)));
      StringFormatter::send(stream,F("M%cA%c%d<;>R%d\n"), 
			    throttle, lors , cab, DCC::getThrottleDirection(cab));
      
      // compare the DCC functionmap with the local copy and send changes  
      uint32_t dccFunctionMap=DCC::getFunctionMap(cab);
      uint32_t myFunctionMap=myLocos[loco].functionMap;
      myLocos[loco].functionMap=dccFunctionMap;
      
      // loop the maps sending any bit changed
      // Loop is terminated as soon as no changes are left
      for (byte fn=0;dccFunctionMap!=myFunctionMap;fn++) {
	if ((dccFunctionMap&1) != (myFunctionMap&1)) {
	  StringFormatter::send(stream,F("M%cA%c%d<;>F%c%d\n"),
				throttle, lors , cab, (dccFunctionMap&1)?'1':'0',fn);
	} 
	// shift just checked bit off end of both maps
	dccFunctionMap>>=1;
	myFunctionMap>>=1;
      } 
    }
    }
  if (streamHasBeenMarked)   stream->commit();     
}

void WiThrottle::markForBroadcast(int cab) {
  for (WiThrottle* wt=firstThrottle; wt!=NULL ; wt=wt->nextThrottle) 
      wt->markForBroadcast2(cab);
}
void WiThrottle::markForBroadcast2(int cab) {
  LOOPLOCOS('*', cab) { 
    myLocos[loco].broadcastPending=true;
  }
}


char WiThrottle::LorS(int cab) {
  return (cab<=HIGHEST_SHORT_ADDR)?'S':'L';
}

// Drive Away feature. Callback handling

RingStream * WiThrottle::stashStream;
WiThrottle * WiThrottle::stashInstance;
byte         WiThrottle::stashClient;
char         WiThrottle::stashThrottleChar;

void WiThrottle::getLocoCallback(int16_t locoid) {
  //DIAG(F("LocoCallback mark client %d"), stashClient);
  stashStream->mark(stashClient);
  
  if (locoid<=0) {
    StringFormatter::send(stashStream,F("HMNo loco found on prog track\n"));
    //DIAG(F("LocoCallback commit (noloco)"));
    stashStream->commit();                  // done here, commit and return
    return;
  }

  // short or long
  char addrchar;
  if (locoid & LONG_ADDR_MARKER) {          // maker bit indicates long addr
    locoid = locoid ^ LONG_ADDR_MARKER;     // remove marker bit to get real long addr
    if (locoid <= HIGHEST_SHORT_ADDR ) {    // out of range for long addr
      StringFormatter::send(stashStream,F("HMLong addr %d <= %d unsupported\n"), locoid, HIGHEST_SHORT_ADDR);
      //DIAG(F("LocoCallback commit (error)"));
      stashStream->commit();                // done here, commit and return
      return;
    }
    addrchar = 'L';
  } else {
    addrchar = 'S';
  }
  
  char addcmd[20]={'M',stashThrottleChar,'+', addrchar};
  itoa(locoid,addcmd+4,10);
  stashInstance->multithrottle(stashStream, (byte *)addcmd);
  TrackManager::setMainPower(POWERMODE::ON);
  TrackManager::setProgPower(POWERMODE::ON);
  TrackManager::setJoin(true);          // <1 JOIN> so we can drive loco away
  DIAG(F("LocoCallback commit success"));
  stashStream->commit();
}

void WiThrottle::sendIntro(Print* stream) {
  if (introSent) // sendIntro only once
    return;
  introSent=true; 
  StringFormatter::send(stream,F("VN2.0\nHTDCC-EX\nRL0\n"));
  StringFormatter::send(stream,F("HtDCC-EX v%S, %S, %S, %S\n"), F(VERSION), F(ARDUINO_TYPE), DCC::getMotorShieldName(), F(GITHUB_SHA));
  StringFormatter::send(stream,F("PTT]\\[Turnouts}|{Turnout]\\[THROW}|{2]\\[CLOSE}|{4\n"));
  StringFormatter::send(stream,F("PPA%x\n"),TrackManager::getMainPower()==POWERMODE::ON);
  // set heartbeat to 2 seconds because we need to sync the metadata (1 second is too short!)
  StringFormatter::send(stream,F("*%d\nHMConnecting..\n"), HEARTBEAT_PRELOAD);
}

void WiThrottle::sendTurnouts(Print* stream) {
     turnoutsSent=true;
      StringFormatter::send(stream,F("PTL"));
      for(Turnout *tt=Turnout::first();tt!=NULL;tt=tt->next()){
          if (tt->isHidden()) continue;
          int id=tt->getId();
          const FSH * tdesc=NULL;
          #ifdef EXRAIL_ACTIVE
          tdesc=RMFT2::getTurnoutDescription(id);
          #endif
          char tchar=Turnout::isClosed(id)?'2':'4';
          if (tdesc==NULL) // turnout with no description
              StringFormatter::send(stream,F("]\\[%d}|{T%d}|{T%c"), id,id,tchar);
	        else 
              StringFormatter::send(stream,F("]\\[%d}|{%S}|{%c"), id,tdesc,tchar);
      }
      StringFormatter::send(stream,F("\n"));
}
void WiThrottle::sendRoster(Print* stream) {
  rosterSent=true;
#ifdef EXRAIL_ACTIVE
  StringFormatter::send(stream,F("RL%d"), RMFT2::rosterNameCount);
  for (int16_t r=0;;r++) {
      int16_t cabid=GETHIGHFLASHW(RMFT2::rosterIdList,r*2);
      if (cabid == INT16_MAX)
	break;
      if (cabid > 0)
	StringFormatter::send(stream,F("]\\[%S}|{%d}|{%c"),
			      RMFT2::getRosterName(cabid),cabid,cabid<128?'S':'L');
  }
  StringFormatter::send(stream,F("\n"));       
#else
   (void)stream; // remove warning
#endif
}
void WiThrottle::sendRoutes(Print* stream) {
  routesSent=true; 
#ifdef EXRAIL_ACTIVE
   StringFormatter::send(stream,F("PRT]\\[Routes}|{Route]\\[Set}|{2]\\[Handoff}|{4\nPRL"));
    // first pass automations
    for (int ix=0;;ix+=2) {
        int16_t id =GETHIGHFLASHW(RMFT2::automationIdList,ix);
        if (id==INT16_MAX) break;
        const FSH * desc=RMFT2::getRouteDescription(id);
        StringFormatter::send(stream,F("]\\[A%d}|{%S}|{4"),id,desc);
    }
    // second pass routes.
    for (int ix=0;;ix+=2) {
        int16_t id=GETHIGHFLASHW(RMFT2::routeIdList,ix);
        if (id==INT16_MAX) break;
        const FSH * desc=RMFT2::getRouteDescription(id);
        StringFormatter::send(stream,F("]\\[R%d}|{%S}|{2"),id,desc);
    }
   StringFormatter::send(stream,F("\n"));
#else
   (void)stream; // remove warning
#endif
}

void WiThrottle::sendFunctions(Print* stream, byte loco) {
  int16_t locoid=myLocos[loco].cab;
  int fkeys=29;
	myLocos[loco].functionToggles=1<<2; // F2 (HORN)  is a non-toggle
        
#ifdef EXRAIL_ACTIVE
	const FSH * functionNames= RMFT2::getRosterFunctions(locoid);
	if (functionNames == NULL) {
	  // no roster entry for locoid, try to find default entry
	  functionNames= RMFT2::getRosterFunctions(0);
	}
	if (functionNames == NULL) {
	  // no default roster entry either, use non-exrail presets as above 
	}
	else if (GETFLASH(functionNames)=='\0') {
	  // "" = Roster but no functions given
	  fkeys=0;
	}  
	else {
	  // we have function names... 
	  // scan names list emitting names, counting functions and 
	  // flagging non-toggling things like horn.
	  myLocos[loco].functionToggles =0;
	  StringFormatter::send(stream, F("M%cL%c%d<;>]\\["), myLocos[loco].throttle,LorS(locoid),locoid);   
	  fkeys=0;
	  bool firstchar=true;
	  for (int fx=0;;fx++) {
	    char c=GETFLASH((char *)functionNames+fx);
	    if (c=='\0') {
	      fkeys++;
	      break;
	    }
	    if (c=='/') {
	      fkeys++;
	      StringFormatter::send(stream,F("]\\["));
	      firstchar=true;
	    }
	    else if (firstchar && c=='*') {
	      myLocos[loco].functionToggles |= 1UL<<fkeys;
	      firstchar=false;
	    } 
	    else {
	      firstchar=false;
	      stream->write(c);
	    }
	  }
	  StringFormatter::send(stream,F("\n"));
	}
        
#endif
	
	for(int fKey=0; fKey<fkeys; fKey++) { 
      int fstate=DCC::getFn(locoid,fKey);
      if (fstate>=0) StringFormatter::send(stream,F("M%cA%c%d<;>F%d%d\n"),myLocos[loco].throttle,LorS(locoid),locoid,fstate,fKey);                     
	}
}
