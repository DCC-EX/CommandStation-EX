/*
 *  © 2022 Chris Harlow
 *  © 2022 Harald Barth
 *  © 2023 Colin Murdoch
 * 
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
#ifdef ARDUINO_ARCH_ESP32
#include <vector>
#endif
#ifndef TrackManager_h
#define TrackManager_h
#include "FSH.h"
#include "MotorDriver.h"
// Virtualised Motor shield multi-track hardware Interface

// These constants help EXRAIL macros say SET_TRACK(2,mode) OR SET_TRACK(C,mode) etc.
const byte TRACK_NUMBER_0=0, TRACK_NUMBER_A=0;    
const byte TRACK_NUMBER_1=1, TRACK_NUMBER_B=1;    
const byte TRACK_NUMBER_2=2, TRACK_NUMBER_C=2;    
const byte TRACK_NUMBER_3=3, TRACK_NUMBER_D=3;    
const byte TRACK_NUMBER_4=4, TRACK_NUMBER_E=4;    
const byte TRACK_NUMBER_5=5, TRACK_NUMBER_F=5;    
const byte TRACK_NUMBER_6=6, TRACK_NUMBER_G=6;    
const byte TRACK_NUMBER_7=7, TRACK_NUMBER_H=7;    

// These constants help EXRAIL macros convert Track Power e.g. SET_POWER(A ON|OFF).
const byte TRACK_POWER_0=0, TRACK_POWER_OFF=0;    
const byte TRACK_POWER_1=1, TRACK_POWER_ON=1;   

class TrackManager {
  public:
    static void Setup(const FSH * shieldName,
                MotorDriver * track0,
                 MotorDriver * track1=NULL,
                 MotorDriver * track2=NULL,
                 MotorDriver * track3=NULL,
                 MotorDriver * track4=NULL,
                 MotorDriver * track5=NULL,
                 MotorDriver * track6=NULL,
                 MotorDriver * track7=NULL
                 );
    
    static void setDCCSignal( bool on);
    static void setPROGSignal( bool on);
    static void setDCSignal(int16_t cab, byte speedbyte);
    static MotorDriver * getProgDriver();
#ifdef ARDUINO_ARCH_ESP32
    static std::vector<MotorDriver *>getMainDrivers();
#endif
  
    static void setPower(POWERMODE mode) {setMainPower(mode); setProgPower(mode);}
    static void setTrackPower(POWERMODE mode, byte t);
    static void setTrackPower(TRACK_MODE trackmode, POWERMODE powermode);
    static void setMainPower(POWERMODE mode) {setTrackPower(TRACK_MODE_MAIN, mode);}
    static void setProgPower(POWERMODE mode) {setTrackPower(TRACK_MODE_PROG, mode);}

    static const int16_t MAX_TRACKS=8;
    static bool setTrackMode(byte track, TRACK_MODE mode, int16_t DCaddr=0);
    static bool parseEqualSign(Print * stream,  int16_t params, int16_t p[]);
    static void loop();
    static POWERMODE getMainPower();
    static POWERMODE getProgPower();
    static inline POWERMODE getPower(byte t) { return track[t]->getPower(); }
    static bool getPower(byte t, char s[]);
    static void setJoin(bool join);
    static bool isJoined() { return progTrackSyncMain;}
    static inline bool isActive (byte tr) {
      if (tr > lastTrack) return false;
      return track[tr]->getMode() & (TRACK_MODE_MAIN|TRACK_MODE_PROG|TRACK_MODE_DC|TRACK_MODE_BOOST|TRACK_MODE_EXT);}
    static void setJoinRelayPin(byte joinRelayPin);
    static void sampleCurrent();
    static void reportGauges(Print* stream);
    static void reportCurrent(Print* stream);
    static void reportObsoleteCurrent(Print* stream); 
    static void streamTrackState(Print* stream, byte t);
    static bool isPowerOn(byte t);
    static bool isProg(byte t);
    static TRACK_MODE getMode(byte t);
    static int16_t returnDCAddr(byte t);
    static const FSH* getModeName(TRACK_MODE Mode);

    static int16_t joinRelay;
    static bool progTrackSyncMain;  // true when prog track is a siding switched to main
    static bool progTrackBoosted;   // true when prog track is not current limited

#ifdef DEBUG_ADC
  public:
#else
  private:
#endif
    static MotorDriver* track[MAX_TRACKS];

  private:
    static void addTrack(byte t, MotorDriver* driver);
    static byte lastTrack;
    static byte nextCycleTrack;
    static void applyDCSpeed(byte t);

    static int16_t trackDCAddr[MAX_TRACKS];  // dc address if TRACK_MODE_DC
#ifdef ARDUINO_ARCH_ESP32
    static byte tempProgTrack; // holds the prog track number during join
#endif
    };

#endif
