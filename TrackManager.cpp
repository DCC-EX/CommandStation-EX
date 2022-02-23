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
#include "TrackManager.h"
#include "FSH.h"
#include "DCCWaveform.h"
#include "MotorDriver.h"
#include "DIAG.h"
// Virtualised Motor shield multi-track hardware Interface

#define LOOPMODE(findmode,function) \
        for (byte t=0;t<8;t++) \
            if (trackMode[t]==findmode) \
                track[t]->function;

const int16_t HASH_KEYWORD_PROG = -29718;
const int16_t HASH_KEYWORD_MAIN = 11339;
const int16_t HASH_KEYWORD_OFF = 9191; // TODO 
const int16_t HASH_KEYWORD_DC = 9192;  // TODO

MotorDriver * TrackManager::track[MAX_TRACKS];
int16_t TrackManager::trackMode[MAX_TRACKS];
 POWERMODE TrackManager::mainPowerGuess=POWERMODE::OFF;

               
void TrackManager::Setup(const FSH * shieldname,
        MotorDriver * track0, MotorDriver * track1, MotorDriver * track2,
        MotorDriver * track3, MotorDriver * track4,  MotorDriver * track5,
        MotorDriver * track6, MotorDriver * track7 ) { 
    (void) shieldname; // TODO        
    track[0]=track0;
    track[1]=track1;
    track[2]=track2;
    track[3]=track3;
    track[4]=track4;
    track[5]=track5;
    track[6]=track6;
    track[7]=track7;

    setTrackMode(0,TRACK_MODE_MAIN);
    setTrackMode(1,TRACK_MODE_PROG);
    setTrackMode(2,TRACK_MODE_OFF);
    setTrackMode(3,TRACK_MODE_OFF);
    setTrackMode(4,TRACK_MODE_OFF);
    setTrackMode(5,TRACK_MODE_OFF);
    setTrackMode(6,TRACK_MODE_OFF);
    setTrackMode(7,TRACK_MODE_OFF);
      // TODO Fault pin config for odd motor boards (example pololu)
  // MotorDriver::commonFaultPin = ((mainDriver->getFaultPin() == progDriver->getFaultPin())
  //				 && (mainDriver->getFaultPin() != UNUSED_PIN));
  DIAG(F("Signal pin config: %S accuracy waveform"),
	 MotorDriver::usePWM ? F("high") : F("normal") );
}
    
void TrackManager::setDCCSignal( bool on) {
    LOOPMODE(TRACK_MODE_MAIN,setSignal(on));
}

void TrackManager::setCutout( bool on) {
    (void) on;
    // TODO      LOOPMODE(TRACK_MODE_MAIN,setCutout(on));
}

void TrackManager::setPROGSignal( bool on) {
    LOOPMODE(TRACK_MODE_PROG,setSignal(on));
}

void TrackManager::setDCSignal(int16_t cab, byte speedbyte) {
    LOOPMODE(cab,setDCSignal(speedbyte));
}

bool TrackManager::setTrackMode(byte trackToSet, int16_t modeOrAddr) {
    if (trackToSet>=8 || track[trackToSet]==NULL) return false;
    if (modeOrAddr==TRACK_MODE_PROG) {
        // only allow 1 track to be prog
        for (byte t=0;t<8;t++)
            if (trackMode[t]==TRACK_MODE_PROG) trackMode[t]=TRACK_MODE_OFF;
    }
    trackMode[trackToSet]=modeOrAddr;
    // re-evaluate HighAccuracy mode
    bool canDo=true;
    for (byte t=0;t<8;t++)
        if (trackMode[t]==TRACK_MODE_MAIN ||trackMode[t]==TRACK_MODE_PROG)
            canDo &= track[t]->isPWMCapable();
    MotorDriver::usePWM=canDo;
    return true; 
}

bool TrackManager::parseJ(Print *stream, int16_t params, int16_t p[])
{
    
    if (params==0) { // <J>  List track assignments
        for (byte t=0;t<8;t++) {
            if (track[t]==NULL) break;
            StringFormatter::send(stream,F("<j %d "),t);
            switch(trackMode[t]) {
              case TRACK_MODE_MAIN:
                StringFormatter::send(stream,F("MAIN"));
                break;
              case TRACK_MODE_PROG:
                StringFormatter::send(stream,F("PROG"));
                break;
              case TRACK_MODE_OFF:
                StringFormatter::send(stream,F("OFF"));
                break;
              default:
                StringFormatter::send(stream,F("DC %d"),trackMode[t]);
            }
            StringFormatter::send(stream,F(">\n"));
        }
        return true;
    }
    
    if (params>1 && (p[1]<0 || p[1]>=MAX_TRACKS)) 
        return false;
    
    if (params==2  && p[1]==HASH_KEYWORD_MAIN) // <J id MAIN>
        return setTrackMode(p[1],TRACK_MODE_MAIN);
    
    if (params==2  && p[1]==HASH_KEYWORD_PROG) // <J id PROG>
        return setTrackMode(p[1],TRACK_MODE_PROG);
    
    if (params==2  && p[1]==HASH_KEYWORD_OFF) // <J id OFF>
        return setTrackMode(p[1],TRACK_MODE_OFF);
    
    if (params==3  && p[1]==HASH_KEYWORD_DC) // <J id DC cab>
        return setTrackMode(p[1],p[2]);

    return false;
}

byte TrackManager::nextCycleTrack=MAX_TRACKS;

void TrackManager::loop(bool dontLimitProg) {
    nextCycleTrack++;
    if (nextCycleTrack>=MAX_TRACKS) nextCycleTrack=0;
    if (track[nextCycleTrack]==NULL) return;
    MotorDriver * motorDriver=track[nextCycleTrack];
    bool useProgLimit=dontLimitProg? false: trackMode[nextCycleTrack]==TRACK_MODE_PROG;
    motorDriver->checkPowerOverload(useProgLimit, nextCycleTrack);   
}

MotorDriver * TrackManager::getProgDriver() {
    for (byte t=0;t<8;t++)
        if (trackMode[t]==TRACK_MODE_PROG) return track[t];
    return NULL;
}        
void TrackManager::setPower2(bool setProg,POWERMODE mode) {
    if (setProg) {
         LOOPMODE(TRACK_MODE_PROG,setPower(mode))
    }
    else {
        mainPowerGuess=mode; 
        for (byte t=0;t<8;t++)
        if (track[t] 
             && trackMode[t]!=TRACK_MODE_OFF
             && trackMode[t]!=TRACK_MODE_PROG 
             ) track[t]->setPower(mode);
    }
}
  POWERMODE TrackManager::getProgPower() {
        for (byte t=0;t<8;t++)
            if (trackMode[t]==TRACK_MODE_PROG) 
                return track[t]->getPower();
        return POWERMODE::OFF;   
  }
   
