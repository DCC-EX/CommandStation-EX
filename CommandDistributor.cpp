/*
 *  © 2020,Gregor Baues,  Chris Harlow. All rights reserved.
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

const byte NO_CLIENT=255; 

RingStream *  CommandDistributor::ring=0;
byte CommandDistributor::ringClient=NO_CLIENT;
CommandDistributor::clientType  CommandDistributor::clients[8]={
  NONE_TYPE,NONE_TYPE,NONE_TYPE,NONE_TYPE,NONE_TYPE,NONE_TYPE,NONE_TYPE,NONE_TYPE};
RingStream * CommandDistributor::broadcastBufferWriter=new RingStream(100);  

void  CommandDistributor::parse(byte clientId,byte * buffer, RingStream * stream) {
  ring=stream;
  ringClient=stream->peekTargetMark();
  if (buffer[0] == '<')  {
    clients[clientId]=COMMAND_TYPE;
     DCCEXParser::parse(stream, buffer, ring);
    }
  else {
    clients[clientId]=WITHROTTLE_TYPE;
    WiThrottle::getThrottle(clientId)->parse(ring, buffer);
  }  
  ringClient=NO_CLIENT;   
}

void CommandDistributor::forget(byte clientId) {
  clients[clientId]=NONE_TYPE;    
}

  
void CommandDistributor::broadcast() {
  broadcastBufferWriter->write((byte)'\0'); 

  /* Boadcast to Serials */ 
  SerialManager::broadcast(broadcastBufferWriter);    

#if defined(WIFI_ON) | defined(ETHERNET_ON)
  // If we are broadcasting from a wifi/eth process we need to complete its output
  // before merging broadcasts in the ring, then reinstate it in case
  // the process continues to output to its client.  
  if (ringClient!=NO_CLIENT) ring->commit();

  /* loop through ring clients */
     for (byte clientId=0; clientId<sizeof(clients); clientId++) {
        if (clients[clientId]==NONE_TYPE) continue;
        ring->mark(clientId);
        
        if (clients[clientId]==COMMAND_TYPE) broadcastBufferWriter->printBuffer(ring);    
        else if (clients[clientId]==WITHROTTLE_TYPE) {
          // TODO... withrottle broadcasts?
        }
        
        ring->commit();
      }
      if (ringClient!=NO_CLIENT) ring->mark(ringClient);
 
#endif
 broadcastBufferWriter->flush();
}

void  CommandDistributor::broadcastSensor(int16_t id, bool on ) {
  StringFormatter::send(broadcastBufferWriter,F("<%c %d>\n"), on?'Q':'q', id);
  broadcast();
  }  

void  CommandDistributor::broadcastTurnout(int16_t id, bool isClosed ) {
  // For DCC++ classic compatibility, state reported to JMRI is 1 for thrown and 0 for closed; 
  StringFormatter::send(broadcastBufferWriter,F("<H %d %d>\n"),id, !isClosed);
  broadcast();
  }  
 
 void  CommandDistributor::broadcastLoco(byte slot) {
   DCC::LOCO * sp=&DCC::speedTable[slot];
  StringFormatter::send(broadcastBufferWriter,F("<l %d %d %d %l>\n"),
     sp->loco,slot,sp->speedCode,sp->functions);    
  broadcast();
}
 
void  CommandDistributor::broadcastPower() {
  const FSH * reason;
  bool main=DCCWaveform::mainTrack.getPowerMode()==POWERMODE::ON;      
  bool prog=DCCWaveform::progTrack.getPowerMode()==POWERMODE::ON;
  bool join=DCCWaveform::progTrackSyncMain;
  if (main && prog && join) reason=F("p1 JOIN");
  else if (main && prog) reason=F("p1");
  else if (main) reason=F("p1 MAIN");
  else if (prog) reason=F("p1 PROG");
  StringFormatter::send(broadcastBufferWriter,F("<%S>\n"),reason);
  LCD(2,reason);    
  broadcast();
}


