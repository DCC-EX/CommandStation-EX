
#include "defines.h"
#include "DCCTrack.h"
#include "DIAG.h"

DCCTrack::DCCTrack(DCCWaveform *w) {
  waveform = w;
}

void DCCTrack::schedulePacket(const byte buffer[], byte byteCount, byte repeats) {
  dccPacket packet;

  // add checksum now, makes stuff easier later
  byte checksum = 0;
  for (byte b = 0; b < byteCount; b++) {
    checksum ^= buffer[b];
    packet.data[b] = buffer[b];
  }
  packet.data[byteCount] = checksum;
  packet.length = byteCount + 1;
  packet.repeat = repeats;
  schedulePacket(packet);
};

void DCCTrack::schedulePacket(dccPacket packet) {
  bool once=true;
  for (const auto& driver: mD) {
    if (driver->type() == RMT_MAIN || driver->type() == RMT_PROG) {
      //DIAG(F("DCCTrack::schedulePacket RMT l=%d d=%x"),packet.length, packet.data[0]);
      driver->schedulePacket(packet);
    }
    if (driver->type() == TIMERINTERRUPT && waveform && once) {
      //DIAG(F("DCCTrack::schedulePacket WAVE l=%d d=%x"),packet.length, packet.data[0]);
      waveform->schedulePacket(packet);
      once=false;
    }
  }
}
