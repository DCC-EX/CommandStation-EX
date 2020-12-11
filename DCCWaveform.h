/*
 *  © 2020, Chris Harlow. All rights reserved.
 *  © 2020, Harald Barth.
 *  
 *  This file is part of Asbelos DCC API
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
#include "ArduinoTimers.h"

// Wait times for power management. Unit: milliseconds
const int  POWER_SAMPLE_ON_WAIT = 100;
const int  POWER_SAMPLE_OFF_WAIT = 1000;
const int  POWER_SAMPLE_OVERLOAD_WAIT = 20;

// Number of preamble bits.
const int   PREAMBLE_BITS_MAIN = 16;
const int   PREAMBLE_BITS_PROG = 22;



const byte   MAX_PACKET_SIZE = 12;
// NOTE: static functions are used for the overall controller, then
// one instance is created for each track.


enum class POWERMODE { OFF, ON, OVERLOAD };

const byte idlePacket[] = {0xFF, 0x00, 0xFF};
const byte resetPacket[] = {0x00, 0x00, 0x00};

class DCCWaveform {
  public:
    DCCWaveform( byte preambleBits, bool isMain);
    static void begin(MotorDriver * mainDriver, MotorDriver * progDriver, byte timerNumber);
    static void setDiagnosticSlowWave(bool slow);
    static void loop();
    static DCCWaveform  mainTrack;
    static DCCWaveform  progTrack;

    void beginTrack();
    void setPowerMode(POWERMODE);
    POWERMODE getPowerMode();
    void checkPowerOverload();
    int  getLastCurrent();
    inline int get1024Current() {
	if (powerMode == POWERMODE::ON)
	    return (int)(lastCurrent*(long int)1024/motorDriver->getRawCurrentTripValue());
	return 0;
    }
    void schedulePacket(const byte buffer[], byte byteCount, byte repeats);
    volatile bool packetPending;
    volatile byte sentResetsSincePacket;
    volatile bool autoPowerOff=false;
    void setAckBaseline();  //prog track only
    void setAckPending();  //prog track only
    byte getAck();               //prog track only 0=NACK, 1=ACK 2=keep waiting
    static bool progTrackSyncMain;  // true when prog track is a siding switched to main
    static bool progTrackBoosted;   // true when prog track is not current limited
    inline void doAutoPowerOff() {
	if (autoPowerOff) {
	    setPowerMode(POWERMODE::OFF);
	    autoPowerOff=false;
	}
    };
    inline void setAckLimit(int mA) {
	ackLimitmA = mA;
    }
    inline void setMinAckPulseDuration(unsigned int i) {
	minAckPulseDuration = i;
    }
    inline void setMaxAckPulseDuration(unsigned int i) {
	maxAckPulseDuration = i;
    }

  private:
    static VirtualTimer * interruptTimer;      
    static void interruptHandler();
    bool interrupt1();
    void interrupt2();
    void checkAck();
    void setSignal(bool high);
    
    bool isMainTrack;
    MotorDriver*  motorDriver;
    // Transmission controller
    byte transmitPacket[MAX_PACKET_SIZE];  // packet being transmitted
    byte transmitLength;
    byte transmitRepeats;      // remaining repeats of transmission
    byte remainingPreambles;
    byte requiredPreambles;
    bool currentBit;           // bit to be transmitted
    byte bits_sent;           // 0-8 (yes 9 bits) sent for current byte
    byte bytes_sent;          // number of bytes sent from transmitPacket
    byte state;               // wave generator state machine

    byte pendingPacket[MAX_PACKET_SIZE];
    byte pendingLength;
    byte pendingRepeats;
    int lastCurrent;

    
    // current sampling
    POWERMODE powerMode;
    unsigned long lastSampleTaken;
    unsigned int sampleDelay;
    // Trip current for programming track, 250mA. Change only if you really
    // need to be non-NMRA-compliant because of decoders that are not either.
    static const int TRIP_CURRENT_PROG=250;
    unsigned long power_sample_overload_wait = POWER_SAMPLE_OVERLOAD_WAIT;
    unsigned int power_good_counter = 0;

    // ACK management (Prog track only)  
    volatile bool ackPending;
    volatile bool ackDetected;
    int  ackThreshold; 
    int  ackLimitmA = 60;
    int ackMaxCurrent;
    unsigned long ackCheckStart; // millis
    unsigned int ackCheckDuration; // millis       
    
    unsigned int ackPulseDuration;  // micros
    unsigned long ackPulseStart; // micros

    unsigned int minAckPulseDuration = 2000; // micros
    unsigned int maxAckPulseDuration = 8500; // micros
           
};
#endif
