/*
 *  © 2022 Chris Harlow
 *  All rights reserved.
 *  
 *  This file is part of DCC++EX
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
#include "DCC.h"
#include "MotorDriver.h"
#include "DIAG.h"
// Virtualised Motor shield multi-track hardware Interface
#define FOR_EACH_TRACK(t) for (byte t=0;t<=lastTrack;t++)
    
#define APPLY_BY_MODE(findmode,function) \
        FOR_EACH_TRACK(t) \
            if (trackMode[t]==findmode) \
                track[t]->function;

const int16_t HASH_KEYWORD_PROG = -29718;
const int16_t HASH_KEYWORD_MAIN = 11339;
const int16_t HASH_KEYWORD_OFF = 22479;  
const int16_t HASH_KEYWORD_DC = 2183;  
const int16_t HASH_KEYWORD_A = 65; // parser makes single chars the ascii.   

MotorDriver * TrackManager::track[MAX_TRACKS];
int16_t TrackManager::trackMode[MAX_TRACKS];
POWERMODE TrackManager::mainPowerGuess=POWERMODE::OFF;
byte TrackManager::lastTrack=0;
bool TrackManager::progTrackSyncMain=false; 
bool TrackManager::progTrackBoosted=false; 
int16_t TrackManager::joinRelay=UNUSED_PIN;


// The setup call is done this way so that the tracks can be in a list 
// from the config... the tracks default to NULL in the declaration                 
void TrackManager::Setup(const FSH * shieldname,
        MotorDriver * track0, MotorDriver * track1, MotorDriver * track2,
        MotorDriver * track3, MotorDriver * track4,  MotorDriver * track5,
        MotorDriver * track6, MotorDriver * track7 ) {       
    addTrack(0,track0);
    addTrack(1,track1);
    addTrack(2,track2);
    addTrack(3,track3);
    addTrack(4,track4);
    addTrack(5,track5);
    addTrack(6,track6);
    addTrack(7,track7);
    
    // Default the first 2 tracks (which mat be null) and perform HA waveform check.
    setTrackMode(0,TRACK_MODE_MAIN);
    setTrackMode(1,TRACK_MODE_PROG);
  
  // TODO Fault pin config for odd motor boards (example pololu)
  // MotorDriver::commonFaultPin = ((mainDriver->getFaultPin() == progDriver->getFaultPin())
  //				 && (mainDriver->getFaultPin() != UNUSED_PIN));
  DIAG(F("Signal pin config: %S accuracy waveform"),
	 MotorDriver::usePWM ? F("high") : F("normal") );
  DCC::begin(shieldname);   
}

void TrackManager::addTrack(byte t, MotorDriver* driver) {
     track[t]=driver;
     trackMode[t]=TRACK_MODE_OFF;
     if (driver) lastTrack=t; 
}    

void TrackManager::setDCCSignal( bool on) {
    APPLY_BY_MODE(TRACK_MODE_MAIN,setSignal(on));
}

void TrackManager::setCutout( bool on) {
    (void) on;
    // TODO      APPLY_BY_MODE(TRACK_MODE_MAIN,setCutout(on));
}

void TrackManager::setPROGSignal( bool on) {
    APPLY_BY_MODE(TRACK_MODE_PROG,setSignal(on));
}

void TrackManager::setDCSignal(int16_t cab, byte speedbyte) {
    APPLY_BY_MODE(cab,setDCSignal(speedbyte));
}

bool TrackManager::setTrackMode(byte trackToSet, int16_t modeOrAddr) {
    if (trackToSet>lastTrack || track[trackToSet]==NULL) return false;
    if (modeOrAddr==TRACK_MODE_PROG) {
        // only allow 1 track to be prog
        FOR_EACH_TRACK(t)
            if (trackMode[t]==TRACK_MODE_PROG) trackMode[t]=TRACK_MODE_OFF;
    }
    trackMode[trackToSet]=modeOrAddr;
    
    // re-evaluate HighAccuracy mode
    // We can only do this is all main and prog tracks agree
    bool canDo=true;
    FOR_EACH_TRACK(t)
        if (trackMode[t]==TRACK_MODE_MAIN ||trackMode[t]==TRACK_MODE_PROG)
            canDo &= track[t]->isPWMCapable();
    MotorDriver::usePWM=canDo;
    return true; 
}

bool TrackManager::parseJ(Print *stream, int16_t params, int16_t p[])
{
    
    if (params==0) { // <=>  List track assignments
        FOR_EACH_TRACK(t)
            if (track[t]!=NULL) {
                StringFormatter::send(stream,F("<= %c "),'A'+t);
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
    
    p[0]-=HASH_KEYWORD_A;  // convert A... to 0.... 

    if (params>1 && (p[0]<0 || p[0]>=MAX_TRACKS)) 
        return false;
    
    if (params==2  && p[1]==HASH_KEYWORD_MAIN) // <= id MAIN>
        return setTrackMode(p[0],TRACK_MODE_MAIN);
    
    if (params==2  && p[1]==HASH_KEYWORD_PROG) // <= id PROG>
        return setTrackMode(p[0],TRACK_MODE_PROG);
    
    if (params==2  && p[1]==HASH_KEYWORD_OFF) // <= id OFF>
        return setTrackMode(p[0],TRACK_MODE_OFF);
    
    if (params==3  && p[1]==HASH_KEYWORD_DC && p[2]>0) // <= id DC cab>
        return setTrackMode(p[0],p[2]);

    return false;
}

byte TrackManager::nextCycleTrack=MAX_TRACKS;

void TrackManager::loop() {
    DCCWaveform::loop(); 
    DCCACK::loop(); 
    bool dontLimitProg=DCCACK::isActive() || progTrackSyncMain || progTrackBoosted;
    nextCycleTrack++;
    if (nextCycleTrack>lastTrack) nextCycleTrack=0;
    if (track[nextCycleTrack]==NULL) return;
    MotorDriver * motorDriver=track[nextCycleTrack];
    bool useProgLimit=dontLimitProg? false: trackMode[nextCycleTrack]==TRACK_MODE_PROG;
    motorDriver->checkPowerOverload(useProgLimit, nextCycleTrack);   
}

MotorDriver * TrackManager::getProgDriver() {
    FOR_EACH_TRACK(t)
        if (trackMode[t]==TRACK_MODE_PROG) return track[t];
    return NULL;
}        
void TrackManager::setPower2(bool setProg,POWERMODE mode) {
    if (setProg) {
         APPLY_BY_MODE(TRACK_MODE_PROG,setPower(mode))
    }
    else {
        mainPowerGuess=mode; 
        FOR_EACH_TRACK(t)
        if (track[t] 
             && trackMode[t]!=TRACK_MODE_OFF
             && trackMode[t]!=TRACK_MODE_PROG 
             ) track[t]->setPower(mode);
    }
}
  
POWERMODE TrackManager::getProgPower() {
    FOR_EACH_TRACK(t)
        if (trackMode[t]==TRACK_MODE_PROG) 
                return track[t]->getPower();
    return POWERMODE::OFF;   
  }
   
void TrackManager::setJoinRelayPin(byte joinRelayPin) {
  joinRelay=joinRelayPin;
  if (joinRelay!=UNUSED_PIN) {
    pinMode(joinRelay,OUTPUT);
    digitalWrite(joinRelay,LOW);  // LOW is relay disengaged
  }
}

void TrackManager::setJoin(bool joined) {
  progTrackSyncMain=joined;
  if (joinRelay!=UNUSED_PIN) digitalWrite(joinRelay,joined?HIGH:LOW);
}
