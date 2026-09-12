/*
 *  © 2026 Chris Harlow
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

#include "defines.h"
#ifndef MOTOR_SHIELD_TYPE
#include "DCCWaveform.h"

DCCWaveform DCCWaveform::mainTrack(PREAMBLE_BITS_MAIN, true);
DCCWaveform DCCWaveform::progTrack(PREAMBLE_BITS_PROG, false);
//RMTChannel *DCCWaveform::rmtMainChannel = NULL;
//RMTChannel *DCCWaveform::rmtProgChannel = NULL;

bool DCCWaveform::railcomPossible = false;
volatile bool DCCWaveform::railcomActive = false;

DCCWaveform::DCCWaveform(byte preambleBits, bool isMain) {
  (void)preambleBits;
  isMainTrack = isMain;
}

void DCCWaveform::begin() {
}

void DCCWaveform::loop() {
}

void DCCWaveform::schedulePacket(const byte buffer[], byte byteCount, byte repeats) {
  (void)buffer;
  (void)byteCount;
  (void)repeats;
}

bool DCCWaveform::isReminderWindowOpen() {
  return false;
}

void DCCWaveform::promotePendingPacket() {
}

bool DCCWaveform::setRailcom(bool on) {
  (void)on;
  return false;
}

void DCCWaveform::clearResets(byte fudge) {
  (void)fudge;
}
byte DCCWaveform::getResets() { return 0; }

#endif