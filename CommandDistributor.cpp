/*
 *  © 2022 Harald Barth
 *  © 2020-2021 Chris Harlow
 *  © 2020 Gregor Baues
 *  © 2022 Colin Murdoch
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

#include <Arduino.h>
#include "CommandDistributor.h"
#include "SerialManager.h"
#include "WiThrottle.h"
#include "DIAG.h"
#include "defines.h"
#include "DCCWaveform.h"
#include "DCC.h"
#include "TrackManager.h"
#include "StringFormatter.h"

// variables to hold clock time
int16_t lastclocktime;
int8_t lastclockrate;


#if WIFI_ON || ETHERNET_ON || defined(SERIAL1_COMMANDS) || defined(SERIAL2_COMMANDS) || defined(SERIAL3_COMMANDS)
// use a buffer to allow broadcast
StringBuffer * CommandDistributor::broadcastBufferWriter=new StringBuffer();
template<typename... Targs> void CommandDistributor::broadcastReply(clientType type, Targs... msg){
  broadcastBufferWriter->flush();
  StringFormatter::send(broadcastBufferWriter, msg...);
  broadcastToClients(type);
}
#else
// on a single USB connection config, write direct to Serial and ignore flush/shove
template<typename... Targs> void CommandDistributor::broadcastReply(clientType type, Targs... msg){
  (void)type; //shut up compiler warning
  StringFormatter::send(&USB_SERIAL, msg...);
}
#endif 

#ifdef CD_HANDLE_RING
  // wifi or ethernet ring streams with multiple client types
  RingStream *  CommandDistributor::ring=0;
  CommandDistributor::clientType  CommandDistributor::clients[8]={
    NONE_TYPE,NONE_TYPE,NONE_TYPE,NONE_TYPE,NONE_TYPE,NONE_TYPE,NONE_TYPE,NONE_TYPE};

// Parse is called by Withrottle or Ethernet interface to determine which
// protocol the client is using and call the appropriate part of dcc++Ex
void  CommandDistributor::parse(byte clientId,byte * buffer, RingStream * stream) {
  if (Diag::WIFI && Diag::CMD)
    DIAG(F("Parse C=%d T=%d B=%s"),clientId, clients[clientId], buffer);
  ring=stream;

  // First check if the client is not known
  // yet and in that case determinine type
  // NOTE: First character of transmission determines if this
  // client is using the DCC++ protocol where all commands start
  // with '<'
  if (clients[clientId] == NONE_TYPE) {
    if (buffer[0] == '<')
      clients[clientId]=COMMAND_TYPE;
    else
      clients[clientId]=WITHROTTLE_TYPE;
  }

  // mark buffer that is sent to parser
  ring->mark(clientId);

  // When type is known, send the string
  // to the right parser
  if (clients[clientId] == COMMAND_TYPE) {
    DCCEXParser::parse(stream, buffer, ring);
  } else if (clients[clientId] == WITHROTTLE_TYPE) {
    WiThrottle::getThrottle(clientId)->parse(ring, buffer);
  }

  if (ring->peekTargetMark()!=RingStream::NO_CLIENT) {
    // The commit call will either write the length bytes
    // OR rollback to the mark because the reply is empty
    // or the command generated more output than fits in
    // the buffer
    if (!ring->commit()) {
      DIAG(F("OUTBOUND FULL processing cmd:%s"),buffer);
    }
  } else {
    DIAG(F("CD parse: was alredy committed")); //XXX Could have been committed by broadcastClient?!
  }
}

void CommandDistributor::forget(byte clientId) {
  if (clients[clientId]==WITHROTTLE_TYPE) WiThrottle::forget(clientId);
  clients[clientId]=NONE_TYPE;
  if (virtualLCDClient==clientId) virtualLCDClient=RingStream::NO_CLIENT;
}
#endif 

// This will not be called on a uno 
void CommandDistributor::broadcastToClients(clientType type) {

  byte rememberClient;
  (void)rememberClient; // shut up compiler warning

  // Broadcast to Serials
  if (type==COMMAND_TYPE) SerialManager::broadcast(broadcastBufferWriter->getString());

#ifdef CD_HANDLE_RING
  // If we are broadcasting from a wifi/eth process we need to complete its output
  // before merging broadcasts in the ring, then reinstate it in case
  // the process continues to output to its client.
  if (ring) {
    if ((rememberClient = ring->peekTargetMark()) != RingStream::NO_CLIENT) {
      //DIAG(F("CD precommit client %d"), rememberClient);
      ring->commit();
    }
    // loop through ring clients
    for (byte clientId=0; clientId<sizeof(clients); clientId++) {
      if (clients[clientId]==type)  {
	//DIAG(F("CD mark client %d"), clientId);
	ring->mark(clientId);
	ring->print(broadcastBufferWriter->getString());
	//DIAG(F("CD commit client %d"), clientId);
	ring->commit();
      }
    }
    // at this point ring is committed (NO_CLIENT) either from
    // 4 or 13 lines above.
    if (rememberClient != RingStream::NO_CLIENT) {
      //DIAG(F("CD postmark client %d"), rememberClient);
      ring->mark(rememberClient);
    }
  }
#endif
}

// Public broadcast functions below 
void  CommandDistributor::broadcastSensor(int16_t id, bool on ) {
  broadcastReply(COMMAND_TYPE, F("<%c %d>\n"), on?'Q':'q', id);
}

void  CommandDistributor::broadcastTurnout(int16_t id, bool isClosed ) {
  // For DCC++ classic compatibility, state reported to JMRI is 1 for thrown and 0 for closed;
  // The string below contains serial and Withrottle protocols which should
  // be safe for both types.
  broadcastReply(COMMAND_TYPE, F("<H %d %d>\n"),id, !isClosed);
#ifdef CD_HANDLE_RING
  broadcastReply(WITHROTTLE_TYPE, F("PTA%c%d\n"), isClosed?'2':'4', id);
#endif
}

void CommandDistributor::broadcastTurntable(int16_t id, uint8_t position, bool moving) {
  broadcastReply(COMMAND_TYPE, F("<I %d %d %d>\n"), id, position, moving);
}

void  CommandDistributor::broadcastClockTime(int16_t time, int8_t rate) {
  // The JMRI clock command is of the form : PFT65871<;>4
  // The CS broadcast is of the form "<jC mmmm nn" where mmmm is time minutes and dd speed
  // The string below contains serial and Withrottle protocols which should
  // be safe for both types.
  broadcastReply(COMMAND_TYPE, F("<jC %d %d>\n"),time, rate);
#ifdef CD_HANDLE_RING
  broadcastReply(WITHROTTLE_TYPE, F("PFT%l<;>%d\n"), (int32_t)time*60, rate);
#endif
}

void CommandDistributor::setClockTime(int16_t clocktime, int8_t clockrate, byte opt) {
  // opt - case 1 save the latest time if changed
  //       case 2 broadcast the time when requested
  //       case 3 display latest time
  switch (opt)
  {
    case 1:
      if (clocktime != lastclocktime){
        // CAH. DIAG removed because LCD does it anyway. 
        LCD(6,F("Clk Time:%d Sp %d"), clocktime, clockrate);
        // look for an event for this time
        RMFT2::clockEvent(clocktime,1);
        // Now tell everyone else what the time is.
        CommandDistributor::broadcastClockTime(clocktime, clockrate);
        lastclocktime = clocktime;
        lastclockrate = clockrate;
      }
      return;

    case 2:
      CommandDistributor::broadcastClockTime(lastclocktime, lastclockrate);
      return;
  }
 
}

int16_t CommandDistributor::retClockTime() {
  return lastclocktime;
}

void  CommandDistributor::broadcastLoco(byte slot) {
  DCC::LOCO * sp=&DCC::speedTable[slot];
  broadcastReply(COMMAND_TYPE, F("<l %d %d %d %l>\n"), sp->loco,slot,sp->speedCode,sp->functions);
#ifdef SABERTOOTH
  if (Serial2 && sp->loco == SABERTOOTH) {
    static uint8_t rampingmode = 0;
    bool direction = (sp->speedCode & 0x80) !=0; // true for forward
    int32_t speed = sp->speedCode & 0x7f;
    if (speed == 1) { // emergency stop
      if (rampingmode != 1) {
	rampingmode = 1;
	Serial2.print("R1: 0\r\n");
	Serial2.print("R2: 0\r\n");
      }
      Serial2.print("MD: 0\r\n");
    } else {
      if (speed != 0) {
        // speed is here 2 to 127
	speed = (speed - 1) * 1625 / 100;
	speed = speed * (direction ? 1 : -1);
	// speed is here -2047 to 2047
      }
      if (rampingmode != 2) {
	rampingmode = 2;
	Serial2.print("R1: 2047\r\n");
	Serial2.print("R2: 2047\r\n");
      }
      Serial2.print("M1: ");
      Serial2.print(speed);
      Serial2.print("\r\n");
      Serial2.print("M2: ");
      Serial2.print(speed);
      Serial2.print("\r\n");
    }
  }
#endif
#ifdef CD_HANDLE_RING
  WiThrottle::markForBroadcast(sp->loco);
#endif
}

void  CommandDistributor::broadcastPower() {
  char pstr[] = "? x";
  for(byte t=0; t<TrackManager::MAX_TRACKS; t++)
    if (TrackManager::getPower(t, pstr))
      broadcastReply(COMMAND_TYPE, F("<p%s>\n"),pstr);

  byte trackcount=0;
  byte oncount=0;
  byte offcount=0;
  for(byte t=0; t<TrackManager::MAX_TRACKS; t++) {
    if (TrackManager::isActive(t)) {
      trackcount++;
      // do not call getPower(t) unless isActive(t)!
      if (TrackManager::getPower(t) == POWERMODE::ON)
	oncount++;
      else
	offcount++;
    }
  }
  //DIAG(F("t=%d on=%d off=%d"), trackcount, oncount, offcount);

  char state='2';
  if (oncount==0 || offcount == trackcount)
    state = '0';
  else if (oncount == trackcount) {
    state = '1';
  }

  // additional info about MAIN, PROG and JOIN
  bool main=TrackManager::getMainPower()==POWERMODE::ON;
  bool prog=TrackManager::getProgPower()==POWERMODE::ON;
  bool join=TrackManager::isJoined();
  //DIAG(F("m=%d p=%d j=%d"), main, prog, join);
  const FSH * reason=F("");
  if (join) {
    reason = F(" JOIN"); // with space at start so we can append without space
    broadcastReply(COMMAND_TYPE, F("<p1 %S>\n"),reason);
  } else {
    if (main) {
      //reason = F("MAIN");
      broadcastReply(COMMAND_TYPE, F("<p1 MAIN>\n"));
    }
    if (prog) {
      //reason = F("PROG");
      broadcastReply(COMMAND_TYPE, F("<p1 PROG>\n"));
    }
  }

  if (state != '2')
    broadcastReply(COMMAND_TYPE, F("<p%c>\n"),state);
#ifdef CD_HANDLE_RING
  // send '1' if all main are on, otherwise global state (which in that case is '0' or '2')
  broadcastReply(WITHROTTLE_TYPE, F("PPA%c\n"), main?'1': state);
#endif

  LCD(2,F("Power %S%S"),state=='1'?F("On"): ( state=='0'? F("Off") : F("SC") ),reason);
}

void CommandDistributor::broadcastRaw(clientType type, char * msg) {
  broadcastReply(type, F("%s"),msg);
}

void CommandDistributor::broadcastTrackState(const FSH* format, byte trackLetter, const FSH *modename, int16_t dcAddr) {
  broadcastReply(COMMAND_TYPE, format, trackLetter, modename, dcAddr);
}

void  CommandDistributor::broadcastRouteState(uint16_t routeId, byte state ) {
  broadcastReply(COMMAND_TYPE, F("<jB %d %d>\n"),routeId,state);
}

void  CommandDistributor::broadcastRouteCaption(uint16_t routeId, const FSH* caption ) {
  broadcastReply(COMMAND_TYPE, F("<jB %d \"%S\">\n"),routeId,caption);
}

Print * CommandDistributor::getVirtualLCDSerial(byte screen, byte row) {
  Print * stream=virtualLCDSerial;
  #ifdef  CD_HANDLE_RING
  rememberVLCDClient=RingStream::NO_CLIENT;
  if (!stream && virtualLCDClient!=RingStream::NO_CLIENT) {
    // If we are broadcasting from a wifi/eth process we need to complete its output
    // before merging broadcasts in the ring, then reinstate it in case
    // the process continues to output to its client.
    if ((rememberVLCDClient = ring->peekTargetMark()) != RingStream::NO_CLIENT) {
      ring->commit();
    }
    ring->mark(virtualLCDClient);   
    stream=ring; 
  }
  #endif
  if (stream) StringFormatter::send(stream,F("<@ %d %d \""), screen,row);
  return stream;  
}

void CommandDistributor::commitVirtualLCDSerial() {
  #ifdef  CD_HANDLE_RING
  if (virtualLCDClient!=RingStream::NO_CLIENT) {
    StringFormatter::send(ring,F("\">\n"));
    ring->commit();
    if (rememberVLCDClient!=RingStream::NO_CLIENT) ring->mark(rememberVLCDClient);
    return;  
   }
  #endif
  StringFormatter::send(virtualLCDSerial,F("\">\n"));  
}

void CommandDistributor::setVirtualLCDSerial(Print * stream) {
  #ifdef  CD_HANDLE_RING
  virtualLCDClient=RingStream::NO_CLIENT;
  if (stream && stream->availableForWrite()==RingStream::THIS_IS_A_RINGSTREAM) {
     virtualLCDClient=((RingStream *) stream)->peekTargetMark();
     virtualLCDSerial=nullptr;
     return;
  }      
    #endif
  virtualLCDSerial=stream;
}

Print* CommandDistributor::virtualLCDSerial=&USB_SERIAL;
byte CommandDistributor::virtualLCDClient=0xFF;
byte CommandDistributor::rememberVLCDClient=0;

