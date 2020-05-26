#include <Arduino.h>
#include "Hardware.h"
#include "DCCWaveform.h"
#include "DIAG.h"
#include "Railcom.h"
DCCWaveform  DCCWaveform::mainTrack(PREAMBLE_BITS_MAIN, true);
DCCWaveform  DCCWaveform::progTrack(PREAMBLE_BITS_PROG, false);

void DCCWaveform::begin() {
  Hardware::init();
  Hardware::setCallback(58, interruptHandler);
  mainTrack.beginTrack();
  progTrack.beginTrack();
}

void DCCWaveform::loop() {
  mainTrack.checkPowerOverload();
  progTrack.checkPowerOverload();
}


// static //
void DCCWaveform::interruptHandler() {
  // call the timer edge sensitive actions for progtrack and maintrack
  bool mainCall2 = mainTrack.interrupt1();
  bool progCall2 = progTrack.interrupt1();

  // call (if necessary) the procs to get the current bits
  // these must complete within 50microsecs of the interrupt
  // but they are only called ONCE PER BIT TRANSMITTED
  // after the rising edge of the signal
  if (mainCall2) mainTrack.interrupt2();
  if (progCall2) progTrack.interrupt2();
}


// An instance of this class handles the DCC transmissions for one track. (main or prog)
// Interrupts are marshalled via the statics.
// A track has a current transmit buffer, and a pending buffer.
// When the current buffer is exhausted, either the pending buffer (if there is one waiting) or an idle buffer.


// This bitmask has 9 entries as each byte is trasmitted as a zero + 8 bits.
const byte bitMask[] = {0x00, 0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01};


DCCWaveform::DCCWaveform( byte preambleBits, bool isMain) {
  // establish appropriate pins
  isMainTrack = isMain;
  packetPending = false;
  memcpy(transmitPacket, idlePacket, sizeof(idlePacket));
  state = 0;
  requiredPreambles = preambleBits;
  bytes_sent = 0;
  bits_sent = 0;
  nextSampleDue = 0;

}
void DCCWaveform::beginTrack() {
  setPowerMode(POWERMODE::ON);
}

POWERMODE DCCWaveform::getPowerMode() {
  return powerMode;
}

void DCCWaveform::setPowerMode(POWERMODE mode) {
  powerMode = mode;
  Hardware::setPower(isMainTrack, mode == POWERMODE::ON);
  if (mode == POWERMODE::ON) delay(200);
}


void DCCWaveform::checkPowerOverload() {
  if (millis() < nextSampleDue) return;
  int current;
  int delay;

  switch (powerMode) {
    case POWERMODE::OFF:
      delay = POWER_SAMPLE_OFF_WAIT;
      break;
    case POWERMODE::ON:
      // Check current
      current = Hardware::getCurrentMilliamps(isMainTrack);
      if (current < POWER_SAMPLE_MAX)  delay = POWER_SAMPLE_ON_WAIT;
      else {
        setPowerMode(POWERMODE::OVERLOAD);
        DIAG(F("\n*** %s TRACK POWER OVERLOAD current=%d max=%d ***\n"), isMainTrack ? "MAIN" : "PROG", current, POWER_SAMPLE_MAX);
        delay = POWER_SAMPLE_OVERLOAD_WAIT;
      }
      break;
    case POWERMODE::OVERLOAD:
      // Try setting it back on after the OVERLOAD_WAIT
      setPowerMode(POWERMODE::ON);
      delay = POWER_SAMPLE_ON_WAIT;
      break;
    default:
      delay = 999; // cant get here..meaningless statement to avoid compiler warning.
  }
  nextSampleDue = millis() + delay;
}





// process time-edge sensitive part of interrupt
// return true if second level required
bool DCCWaveform::interrupt1() {
  // NOTE: this must consume transmission buffers even if the power is off
  // otherwise can cause hangs in main loop waiting for the pendingBuffer.
  switch (state) {
    case 0:  // start of bit transmission
      Hardware::setSignal(isMainTrack, HIGH);
      checkRailcom();
      state = 1;
      return true; // must call interrupt2 to set currentBit

    case 1:  // 58us after case 0
      if (currentBit) {
        Hardware::setSignal(isMainTrack, LOW);
        state = 0;
      }
      else state = 2;
      break;
    case 2:  // 116us after case 0
      Hardware::setSignal(isMainTrack, LOW);
      state = 3;
      break;
    case 3:  // finished sending zero bit
      state = 0;
      break;
  }
  return false;

}


void DCCWaveform::interrupt2() {
  // set currentBit to be the next bit to be sent.

  if (remainingPreambles > 0 ) {
    currentBit = true;
    remainingPreambles--;
    return;
  }

  // beware OF 9-BIT MASK  generating a zero to start each byte
  currentBit = transmitPacket[bytes_sent] & bitMask[bits_sent];
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
      remainingPreambles = requiredPreambles;

      if (transmitRepeats > 0) {
        transmitRepeats--;
      }
      else if (packetPending) {
        // Copy pending packet to transmit packet
        for (int b = 0; b < pendingLength; b++) transmitPacket[b] = pendingPacket[b];
        transmitLength = pendingLength;
        transmitRepeats = pendingRepeats;
        packetPending = false;
      }
      else {
        // Fortunately reset and idle packets are the same length
        memcpy( transmitPacket, isMainTrack ? idlePacket : resetPacket, sizeof(idlePacket));
        transmitLength = sizeof(idlePacket);
        transmitRepeats = 0;
      }
    }
  }
}

void DCCWaveform::checkRailcom() {
  if (isMainTrack && RAILCOM_CUTOUT) {
    byte preamble = PREAMBLE_BITS_MAIN - remainingPreambles;
    if (preamble == RAILCOM_PREAMBLES_BEFORE_CUTOUT) {
       Railcom::startCutout();
    }
  }
}

// Wait until there is no packet pending, then make this pending
void DCCWaveform::schedulePacket(const byte buffer[], byte byteCount, byte repeats) {
  if (byteCount >= MAX_PACKET_SIZE) return; // allow for chksum
  while (packetPending);

  byte checksum = 0;
  for (int b = 0; b < byteCount; b++) {
    checksum ^= buffer[b];
    pendingPacket[b] = buffer[b];
  }
  pendingPacket[byteCount] = checksum;
  pendingLength = byteCount + 1;
  pendingRepeats = repeats;
  packetPending = true;
}



bool DCCWaveform::getAck()
{

  if (isMainTrack) return false; // cant do this on main track

  while (packetPending); // wait until transmitter has started transmitting the message
  unsigned long timeout = millis() + ACK_TIMEOUT;
  int maxCurrent = 0;
  bool result = false;
  int upsamples = 0;
  int downsamples = 0;

  // Monitor looking for a reading high enough to be an ack
  while (result == false && timeout > millis()) {
    upsamples++;
    int current = Hardware::getCurrentMilliamps(isMainTrack);
    maxCurrent = max(maxCurrent, current);
    result = current > ACK_MIN_PULSE;
  }

  // Monitor current until ack signal dies back
  if (result) while ( true) {
      downsamples++;
      int current = Hardware::getCurrentMilliamps(isMainTrack);
      maxCurrent = max(maxCurrent, current);
      if (current <= ACK_MAX_NOT_PULSE) break;
    }
  // The following DIAG is really useful as it can show how long and how far the
  // current changes during an ACK from the decoder.
  DIAG(F("\nack=%d  max=%d, up=%d, down=%d "), result, maxCurrent, upsamples, downsamples);
  return result;
}
