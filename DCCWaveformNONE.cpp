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

void DCCWaveform::clearResets() {}
byte DCCWaveform::getResets() { return 0; }

#endif