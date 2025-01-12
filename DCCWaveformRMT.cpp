/*
 *  © 2021 Neil McKechnie
 *  © 2021 Mike S
 *  © 2021 Fred Decker
 *  © 2020-2022 Harald Barth
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
// This code is ESP32 ONLY.  
#ifdef ARDUINO_ARCH_ESP32
#include "DCCWaveform.h"
#include "DCCACK.h"

DCCWaveform  DCCWaveform::mainTrack(PREAMBLE_BITS_MAIN, true);
DCCWaveform  DCCWaveform::progTrack(PREAMBLE_BITS_PROG, false);
RMTChannel *DCCWaveform::rmtMainChannel = NULL;
RMTChannel *DCCWaveform::rmtProgChannel = NULL;

DCCWaveform::DCCWaveform(byte preambleBits, bool isMain) {
  isMainTrack = isMain;
  requiredPreambles = preambleBits;
}
void DCCWaveform::begin() {
  for(const auto& md: TrackManager::getMainDrivers()) {
    pinpair p = md->getSignalPin();
    if(rmtMainChannel) {
      //DIAG(F("added pins %d %d to MAIN channel"), p.pin, p.invpin);
      rmtMainChannel->addPin(p); // add pin to existing main channel
    } else {
      //DIAG(F("new MAIN channel with pins %d %d"), p.pin, p.invpin);
      rmtMainChannel = new RMTChannel(p, true); /* create new main channel */
    }
  }
  MotorDriver *md = TrackManager::getProgDriver();
  if (md) {
    pinpair p = md->getSignalPin();
    if (rmtProgChannel) {
      //DIAG(F("added pins %d %d to PROG channel"), p.pin, p.invpin);
      rmtProgChannel->addPin(p); // add pin to existing prog channel
    } else {
      //DIAG(F("new PROGchannel with pins %d %d"), p.pin, p.invpin);
      rmtProgChannel = new RMTChannel(p, false);
    }
  }
}

void DCCWaveform::schedulePacket(const byte buffer[], byte byteCount, byte repeats) {
  if (byteCount > MAX_PACKET_SIZE) return; // allow for chksum
  
  byte checksum = 0;
  for (byte b = 0; b < byteCount; b++) {
    checksum ^= buffer[b];
    pendingPacket[b] = buffer[b];
  }
  // buffer is MAX_PACKET_SIZE but pendingPacket is one bigger
  pendingPacket[byteCount] = checksum;
  pendingLength = byteCount + 1;
  pendingRepeats = repeats;
// DIAG repeated commands (accesories)
//  if (pendingRepeats > 0)
//    DIAG(F("Repeats=%d on %s track"), pendingRepeats, isMainTrack ? "MAIN" : "PROG");
  // The resets will be zero not only now but as well repeats packets into the future
  clearResets(repeats+1);
  {
    int ret = 0;
    do {
      if(isMainTrack) {
	if (rmtMainChannel != NULL)
	  ret = rmtMainChannel->RMTfillData(pendingPacket, pendingLength, pendingRepeats);
      } else {
	if (rmtProgChannel != NULL)
	  ret = rmtProgChannel->RMTfillData(pendingPacket, pendingLength, pendingRepeats);
      }
    } while(ret > 0);
  }
}

bool DCCWaveform::isReminderWindowOpen() {
  if(isMainTrack) {
    if (rmtMainChannel == NULL)
      return false;
    return !rmtMainChannel->busy();
  } else {
    if (rmtProgChannel == NULL)
      return false;
    return !rmtProgChannel->busy();
  }
}
void IRAM_ATTR DCCWaveform::loop() {
  DCCACK::checkAck(progTrack.getResets());
}

bool DCCWaveform::setRailcom(bool on, bool debug) {
  // TODO... ESP32 railcom waveform
  return false;
}

#endif
