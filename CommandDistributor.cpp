/*
 *  © 2022 Harald Barth
 *  © 2020-2021 Chris Harlow
 *  © 2020 Gregor Baues
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

#if defined(BIG_MEMORY) | defined(WIFI_ON) | defined(ETHERNET_ON)
// This section of CommandDistributor is simply not relevant on a uno or similar
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
  } else {
    clients[clientId]=WITHROTTLE_TYPE;
    WiThrottle::getThrottle(clientId)->parse(ring, buffer);
  }
  ringClient=NO_CLIENT;
}

void CommandDistributor::forget(byte clientId) {
  clients[clientId]=NONE_TYPE;
}


void CommandDistributor::broadcast(bool includeWithrottleClients) {
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
    if ( clients[clientId]==WITHROTTLE_TYPE && !includeWithrottleClients) continue;
    ring->mark(clientId);
    broadcastBufferWriter->printBuffer(ring);
    ring->commit();
  }
  if (ringClient!=NO_CLIENT) ring->mark(ringClient);

#endif
 broadcastBufferWriter->flush();
}
#else
// For a UNO/NANO we can broadcast direct to just one Serial instead of the ring
// Redirect ring output ditrect to Serial
#define broadcastBufferWriter &Serial
// and ignore the internal broadcast call.
void CommandDistributor::broadcast(bool includeWithrottleClients) {
  (void)includeWithrottleClients;
}
#endif

void  CommandDistributor::broadcastSensor(int16_t id, bool on ) {
  StringFormatter::send(broadcastBufferWriter,F("<%c %d>\n"), on?'Q':'q', id);
  broadcast(false);
}

void  CommandDistributor::broadcastTurnout(int16_t id, bool isClosed ) {
  // For DCC++ classic compatibility, state reported to JMRI is 1 for thrown and 0 for closed;
  // The string below contains serial and Withrottle protocols which should
  // be safe for both types.
  StringFormatter::send(broadcastBufferWriter,F("<H %d %d>\n"),id, !isClosed);
#if defined(WIFI_ON) | defined(ETHERNET_ON)
  StringFormatter::send(broadcastBufferWriter,F("PTA%c%d\n"), isClosed?'2':'4', id);
#endif
  broadcast(true);
}

void  CommandDistributor::broadcastLoco(byte slot) {
  DCC::LOCO * sp=&DCC::speedTable[slot];
  StringFormatter::send(broadcastBufferWriter,F("<l %d %d %d %l>\n"),
			sp->loco,slot,sp->speedCode,sp->functions);
  broadcast(false);
#if defined(WIFI_ON) | defined(ETHERNET_ON)
  WiThrottle::markForBroadcast(sp->loco);
#endif
}

void  CommandDistributor::broadcastPower() {
  bool main=DCCWaveform::mainTrack.getPowerMode()==POWERMODE::ON;
  bool prog=DCCWaveform::progTrack.getPowerMode()==POWERMODE::ON;
  bool join=DCCWaveform::progTrackSyncMain;
  const FSH * reason=F("");
  char state='1';
  if (main && prog && join) reason=F(" JOIN");
  else if (main && prog);
  else if (main) reason=F(" MAIN");
  else if (prog) reason=F(" PROG");
  else state='0';

  StringFormatter::send(broadcastBufferWriter,
                        F("<p%c%S>\nPPA%c\n"),state,reason, main?'1':'0');
  LCD(2,F("Power %S%S"),state=='1'?F("On"):F("Off"),reason);
  broadcast(true);
}

void CommandDistributor::broadcastText(const FSH * msg) {
  StringFormatter::send(broadcastBufferWriter,F("%S"),msg);
  broadcast(false); 
}
