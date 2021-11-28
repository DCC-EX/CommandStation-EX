
#pragma once
#include <Arduino.h>
#include "DCCPacket.h"
#include "DCCWaveform.h"
#include "DIAG.h"

class DCCTrack {
 public:
  DCCTrack(DCCWaveform *w);
  void schedulePacket(const byte buffer[], byte byteCount, byte repeats);
  void schedulePacket(dccPacket packet);
  inline void addDriver(MotorDriver *m) { mD.push_back(m);
    DIAG(F("Track: mDType=%d count=%d"),m->type(), mD.size());
  };
  static DCCTrack mainTrack;
  static DCCTrack progTrack;
 private:
  DCCWaveform *waveform;
  std::vector<MotorDriver *>mD;
};


