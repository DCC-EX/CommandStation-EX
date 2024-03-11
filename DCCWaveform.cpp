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
#ifndef ARDUINO_ARCH_ESP32
  // This code is replaced entirely on an ESP32
#include <Arduino.h>

#include "DCCWaveform.h"
#include "TrackManager.h"
#include "DCCTimer.h"
#include "DCCACK.h"
#include "DIAG.h"


DCCWaveform  DCCWaveform::mainTrack(PREAMBLE_BITS_MAIN, true);
DCCWaveform  DCCWaveform::progTrack(PREAMBLE_BITS_PROG, false);


// This bitmask has 9 entries as each byte is trasmitted as a zero + 8 bits.
const byte bitMask[] = {0x00, 0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01};

const byte idlePacket[] = {0xFF, 0x00, 0xFF};
const byte resetPacket[] = {0x00, 0x00, 0x00};


// For each state of the wave  nextState=stateTransform[currentState] 
const WAVE_STATE stateTransform[]={
   /* WAVE_START   -> */ WAVE_PENDING,
   /* WAVE_MID_1   -> */ WAVE_START,
   /* WAVE_HIGH_0  -> */ WAVE_MID_0,
   /* WAVE_MID_0   -> */ WAVE_LOW_0,
   /* WAVE_LOW_0   -> */ WAVE_START,
   /* WAVE_PENDING (should not happen) -> */ WAVE_PENDING};

// For each state of the wave, signal pin is HIGH or LOW   
const bool signalTransform[]={
   /* WAVE_START   -> */ HIGH,
   /* WAVE_MID_1   -> */ LOW,
   /* WAVE_HIGH_0  -> */ HIGH,
   /* WAVE_MID_0   -> */ LOW,
   /* WAVE_LOW_0   -> */ LOW,
   /* WAVE_PENDING (should not happen) -> */ LOW};

void DCCWaveform::begin() {
  DCCTimer::begin(DCCWaveform::interruptHandler);     
}

void DCCWaveform::loop() {
 // empty placemarker in case ESP32 needs something here 
}

#pragma GCC push_options
#pragma GCC optimize ("-O3")
void DCCWaveform::interruptHandler() {
  // call the timer edge sensitive actions for progtrack and maintrack
  // member functions would be cleaner but have more overhead
  byte sigMain=signalTransform[mainTrack.state];
  byte sigProg=TrackManager::progTrackSyncMain? sigMain : signalTransform[progTrack.state];
  
  // Set the signal state for both tracks
  TrackManager::setDCCSignal(sigMain);
  TrackManager::setPROGSignal(sigProg);

  // Refresh the values in the ADCee object buffering the values of the ADC HW
  ADCee::scan();

  // Move on in the state engine
  mainTrack.state=stateTransform[mainTrack.state];    
  progTrack.state=stateTransform[progTrack.state];    

  // WAVE_PENDING means we dont yet know what the next bit is
  if (mainTrack.state==WAVE_PENDING) mainTrack.interrupt2();  
  if (progTrack.state==WAVE_PENDING) progTrack.interrupt2();
  else DCCACK::checkAck(progTrack.getResets());

}
#pragma GCC pop_options

// An instance of this class handles the DCC transmissions for one track. (main or prog)
// Interrupts are marshalled via the statics.
// A track has a current transmit buffer, and a pending buffer.
// When the current buffer is exhausted, either the pending buffer (if there is one waiting) or an idle buffer.



DCCWaveform::DCCWaveform( byte preambleBits, bool isMain) {
  isMainTrack = isMain;
  packetPending = false;
  reminderWindowOpen = false;
  memcpy(transmitPacket, idlePacket, sizeof(idlePacket));
  state = WAVE_START;
  // The +1 below is to allow the preamble generator to create the stop bit
  // for the previous packet. 
  requiredPreambles = preambleBits+1;  
  bytes_sent = 0;
  bits_sent = 0;
}
    
volatile bool DCCWaveform::railcomActive=false;     // switched on by user
volatile bool DCCWaveform::railcomDebug=false;     // switched on by user

bool DCCWaveform::setRailcom(bool on, bool debug) {
  if (on) {
    // TODO check possible
    railcomActive=true;
    railcomDebug=debug;
  }
  else {
    railcomActive=false;
    railcomDebug=false;
  } 
  return railcomActive;
}

#pragma GCC push_options
#pragma GCC optimize ("-O3")
void DCCWaveform::interrupt2() {
  // calculate the next bit to be sent:
  // set state WAVE_MID_1  for a 1=bit
  //        or WAVE_HIGH_0 for a 0 bit.
  if (remainingPreambles > 0 ) {
    state=WAVE_MID_1;  // switch state to trigger LOW on next interrupt
    remainingPreambles--;
  
    // As we get to the end of the preambles, open the reminder window.
    // This delays any reminder insertion until the last moment so
    // that the reminder doesn't block a more urgent packet. 
    reminderWindowOpen=transmitRepeats==0 && remainingPreambles<4 && remainingPreambles>1;
    if (remainingPreambles==1) promotePendingPacket();
    else if (remainingPreambles==10 && isMainTrack && railcomActive) DCCTimer::ackRailcomTimer();
    // Update free memory diagnostic as we don't have anything else to do this time.
    // Allow for checkAck and its called functions using 22 bytes more.
    else DCCTimer::updateMinimumFreeMemoryISR(22); 
    return;
  }

  // Wave has gone HIGH but what happens next depends on the bit to be transmitted
  // beware OF 9-BIT MASK  generating a zero to start each byte
  state=(transmitPacket[bytes_sent] & bitMask[bits_sent])? WAVE_MID_1 : WAVE_HIGH_0; 
  bits_sent++;

  // If this is the last bit of a byte, prepare for the next byte

  if (bits_sent == 9) { // zero followed by 8 bits of a byte
    //end of Byte
    bits_sent = 0;
    bytes_sent++;
    // if this is the last byte, prepere for next packet
    if (bytes_sent >= transmitLength) {
      // end of transmission buffer... repeat or switch to next message
      bytes_sent = 0;
      // preamble for next packet will start...
      remainingPreambles = requiredPreambles;
      
      // set the railcom coundown to trigger half way 
      // through the first preamble bit.
      // Note.. we are still sending the last packet bit
      //    and we then have to allow for the packet end bit
      if (isMainTrack && railcomActive) DCCTimer::startRailcomTimer(9);
      }
  }  
}
#pragma GCC pop_options

// Wait until there is no packet pending, then make this pending
void DCCWaveform::schedulePacket(const byte buffer[], byte byteCount, byte repeats) {
  if (byteCount > MAX_PACKET_SIZE) return; // allow for chksum
  while (packetPending);

  byte checksum = 0;
  for (byte b = 0; b < byteCount; b++) {
    checksum ^= buffer[b];
    pendingPacket[b] = buffer[b];
  }
  // buffer is MAX_PACKET_SIZE but pendingPacket is one bigger
  pendingPacket[byteCount] = checksum;
  pendingLength = byteCount + 1;
  pendingRepeats = repeats;
  packetPending = true;
  clearResets();
}

bool DCCWaveform::isReminderWindowOpen() {
  return reminderWindowOpen && ! packetPending;
}

void DCCWaveform::promotePendingPacket() {
    // fill the transmission packet from the pending packet
    
    // Just keep going if repeating  
    if (transmitRepeats > 0) {
        transmitRepeats--;
        return;
      }

    if (packetPending) {
        // Copy pending packet to transmit packet
        // a fixed length memcpy is faster than a variable length loop for these small lengths
        // for (int b = 0; b < pendingLength; b++) transmitPacket[b] = pendingPacket[b];
        memcpy( transmitPacket, pendingPacket, sizeof(pendingPacket));
        
        transmitLength = pendingLength;
        transmitRepeats = pendingRepeats;
        packetPending = false;
        clearResets();
        return;
      }
      
      // nothing to do, just send idles or resets
      // Fortunately reset and idle packets are the same length
      // Note: If railcomDebug is on, then we send resets to the main
      //       track instead of idles. This means that all data will be zeros
      //       and only the porersets will be ones, making it much
      //       easier to read on a logic analyser.
      memcpy( transmitPacket, (isMainTrack && (!railcomDebug)) ? idlePacket : resetPacket, sizeof(idlePacket));
      transmitLength = sizeof(idlePacket);
      transmitRepeats = 0;
      if (getResets() < 250) sentResetsSincePacket++; // only place to increment (private!)
}
#endif

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
    int ret;
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
