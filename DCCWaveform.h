/*
 *  © 2021 M Steve Todd
 *  © 2021 Mike S
 *  © 2021 Fred Decker
 *  © 2020-2024 Harald Barth
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
#ifndef DCCWaveform_h
#define DCCWaveform_h

#include "MotorDriver.h"
#ifdef ARDUINO_ARCH_ESP32
#include "DCCRMT.h"
#include "TrackManager.h"
#endif



// Number of preamble bits.
const byte PREAMBLE_BITS_MAIN = 16;
const byte PREAMBLE_BITS_PROG = 22;
const byte MAX_PACKET_SIZE = 5;     // NMRA standard extended packets, payload size WITHOUT checksum.


// The WAVE_STATE enum is deliberately numbered because a change of order would be catastrophic
// to the transform array.
enum  WAVE_STATE : byte {
  WAVE_START=0,  // wave going high at start of bit 
  WAVE_MID_1=1,  // middle of 1 bit 
  WAVE_HIGH_0=2, // first part of 0 bit high
  WAVE_MID_0=3,  // middle of 0 bit
  WAVE_LOW_0=4,  // first part of 0 bit low
  WAVE_PENDING=5 // next bit not yet known
  };

// NOTE: static functions are used for the overall controller, then
// one instance is created for each track.

class DCCWaveform {
  public:
    DCCWaveform( byte preambleBits, bool isMain);
    static void begin();
    static void loop();
    static DCCWaveform  mainTrack;
    static DCCWaveform  progTrack;
    inline void clearRepeats() { transmitRepeats=0; }
#ifndef ARDUINO_ARCH_ESP32
    inline void clearResets() { sentResetsSincePacket=0; }
    inline byte getResets() { return sentResetsSincePacket; }
#else
  // extrafudge is added when we know that the resets will first come extrafudge  packets in the future
    inline void clearResets(byte extrafudge=0) {
      if ((isMainTrack ? rmtMainChannel : rmtProgChannel) == NULL) return;
      resetPacketBase = isMainTrack ? rmtMainChannel->packetCount() : rmtProgChannel->packetCount();
      resetPacketBase += extrafudge;
    };
    inline byte getResets() {
      if ((isMainTrack ? rmtMainChannel : rmtProgChannel) == NULL) return 0;
      uint32_t packetcount = isMainTrack ?
	rmtMainChannel->packetCount() : rmtProgChannel->packetCount();
      uint32_t count = packetcount - resetPacketBase; // Beware of unsigned interger arithmetic.
      if (count > UINT32_MAX/2)                       // we are in the extrafudge area
	return 0;
      if (count > 255)                                // cap to 255
	return 255;
      return count;                                   // all special cases handled above
    };
#endif
    void schedulePacket(const byte buffer[], byte byteCount, byte repeats);
    bool isReminderWindowOpen();
    void promotePendingPacket();
    static bool setRailcom(bool on, bool debug);
    static bool isRailcom() {return railcomActive;}
    
  private:
#ifndef ARDUINO_ARCH_ESP32
    volatile bool packetPending;
    volatile bool reminderWindowOpen;
    volatile byte sentResetsSincePacket;
#else
    volatile uint32_t resetPacketBase;
#endif
    static void interruptHandler();
    void interrupt2();
    
    bool isMainTrack;
    // Transmission controller
    byte transmitPacket[MAX_PACKET_SIZE+1]; // +1 for checksum
    byte transmitLength;
    byte transmitRepeats;      // remaining repeats of transmission
    byte remainingPreambles;
    byte requiredPreambles;
    byte bits_sent;           // 0-8 (yes 9 bits) sent for current byte
    byte bytes_sent;          // number of bytes sent from transmitPacket
    WAVE_STATE state;         // wave generator state machine
    byte pendingPacket[MAX_PACKET_SIZE+1]; // +1 for checksum
    byte pendingLength;
    byte pendingRepeats;
    static volatile bool railcomActive;     // switched on by user
    static volatile bool railcomDebug;     // switched on by user
    
#ifdef ARDUINO_ARCH_ESP32
  static RMTChannel *rmtMainChannel;
  static RMTChannel *rmtProgChannel;
#endif
};
#endif
