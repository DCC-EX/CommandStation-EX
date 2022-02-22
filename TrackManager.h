/*
 *  © 2022 Chris Harlow
 *  All rights reserved.
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
#ifndef TrackManager_h
#define TrackManager_h
#include "FSH.h"
#include "MotorDriver.h"
// Virtualised Motor shield multi-track hardware Interface


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
    static void setCutout( bool on);
    static void setPROGSignal( bool on);
    static void setDCSignal(int16_t cab, byte speedbyte);
    static const int16_t TRACK_MODE_MAIN=32760;
    static const int16_t TRACK_MODE_PROG=32761;
    static const int16_t TRACK_MODE_OFF=0;
    static const int16_t MAX_TRACKS=8;
    static bool setTrackMode(byte track, int16_t DCaddrOrMode);
    static bool parseJ(Print * stream,  int16_t params, int16_t p[]);


    
  private:
    static MotorDriver* track[MAX_TRACKS];
    static int16_t trackMode[MAX_TRACKS];  // dc address or TRACK_MODE_DCC, TRACK_MODE_PROG, TRACK_MODE_OFF
};

#endif
