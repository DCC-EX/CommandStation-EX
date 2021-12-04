#pragma once

const byte MAX_PACKET_SIZE = 5;  // NMRA standard extended packets, payload size WITHOUT checksum.

// Number of preamble bits (moved here so MotorDriver and Waveform know)
const int   PREAMBLE_BITS_MAIN = 16;
const int   PREAMBLE_BITS_PROG = 22;

class dccPacket {
 public:
  byte data[MAX_PACKET_SIZE+1]; // space for checksum if needed
  byte length   : 4; // future proof up to 15
  byte repeat   : 4; // hopefully 15 enough for ever
  //byte priority : 2; // 0 repeats; 1 mobile function ; 2 accessory ; 3 mobile speed
};

