
#pragma once
#include <Arduino.h>
#include "DCCPacket.h"
#include "DCCWaveform.h"

class DCCTrack {
 public:
  DCCTrack(DCCWaveform *w);
  void schedulePacket(const byte buffer[], byte byteCount, byte repeats);
  void schedulePacket(dccPacket packet);
  inline void addDriver(MotorDriver *m) { mD.push_back(m); };
  static DCCTrack mainTrack;
  static DCCTrack progTrack;
 private:
  DCCWaveform *waveform;
  std::vector<MotorDriver *>mD;
};


