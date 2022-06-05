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
#include "DCCTimer.h"
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
const int16_t HASH_KEYWORD_DCX = 6463; // DC reversed polarity 
const int16_t HASH_KEYWORD_EXT = 8201; // External DCC signal
const int16_t HASH_KEYWORD_A = 65; // parser makes single chars the ascii.   

MotorDriver * TrackManager::track[MAX_TRACKS];
TRACK_MODE TrackManager::trackMode[MAX_TRACKS];
int16_t TrackManager::trackDCAddr[MAX_TRACKS];

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
    
    // Default the first 2 tracks (which may be null) and perform HA waveform check.
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
     trackMode[t]=TRACK_MODE_OFF;
     track[t]=driver;
     if (driver) {
         track[t]->setPower(POWERMODE::OFF);
         lastTrack=t;
     } 
}

// defined in Motordriver.cpp
extern byte fakePORTA;
extern byte fakePORTB;
extern byte fakePORTC;

void TrackManager::setDCCSignal( bool on) {
  HAVE_PORTA(fakePORTA=PORTA);
  HAVE_PORTB(fakePORTB=PORTB);
  HAVE_PORTC(fakePORTC=PORTC);
  APPLY_BY_MODE(TRACK_MODE_MAIN,setSignal(on));
  HAVE_PORTA(PORTA=fakePORTA);
  HAVE_PORTB(PORTB=fakePORTB);
  HAVE_PORTC(PORTC=fakePORTC);
}

void TrackManager::setCutout( bool on) {
    (void) on;
    // TODO Cutout needs fake ports as well
    // TODO      APPLY_BY_MODE(TRACK_MODE_MAIN,setCutout(on));
}

void TrackManager::setPROGSignal( bool on) {
  HAVE_PORTA(fakePORTA=PORTA);
  HAVE_PORTB(fakePORTB=PORTB);
  HAVE_PORTC(fakePORTC=PORTC);
  APPLY_BY_MODE(TRACK_MODE_PROG,setSignal(on));
  HAVE_PORTA(PORTA=fakePORTA);
  HAVE_PORTB(PORTB=fakePORTB);
  HAVE_PORTC(PORTC=fakePORTC);
}

void TrackManager::setDCSignal(int16_t cab, byte speedbyte) {
  HAVE_PORTA(fakePORTA=PORTA);
  HAVE_PORTB(fakePORTB=PORTB);
  HAVE_PORTC(fakePORTC=PORTC);
  FOR_EACH_TRACK(t) {
    if (trackDCAddr[t]!=cab) continue;
    if (trackMode[t]==TRACK_MODE_DC) track[t]->setDCSignal(speedbyte);
    else if (trackMode[t]==TRACK_MODE_DCX) track[t]->setDCSignal(speedbyte ^ 128);
  }
  HAVE_PORTA(PORTA=fakePORTA);
  HAVE_PORTB(PORTB=fakePORTB);
  HAVE_PORTC(PORTC=fakePORTC);
}    


bool TrackManager::setTrackMode(byte trackToSet, TRACK_MODE mode, int16_t dcAddr) {
    if (trackToSet>lastTrack || track[trackToSet]==NULL) return false;

    DIAG(F("Track=%c"),trackToSet+'A');
    // DC tracks require a motorDriver that can set brake!
    if ((mode==TRACK_MODE_DC || mode==TRACK_MODE_DCX)
         && !track[trackToSet]->brakeCanPWM()) {
             DIAG(F("Brake pin can't PWM: No DC"));
             return false; 
         }

    if (mode==TRACK_MODE_PROG) {
        // only allow 1 track to be prog
        FOR_EACH_TRACK(t)
            if (trackMode[t]==TRACK_MODE_PROG && t != trackToSet) {
                track[t]->setPower(POWERMODE::OFF);
                trackMode[t]=TRACK_MODE_OFF;
            }
    } else {
      track[trackToSet]->setResetCounterPointer(NULL); // only the prog track has this pointer set
    }
    trackMode[trackToSet]=mode;
    trackDCAddr[trackToSet]=dcAddr;
    
    // When a track is switched, we must clear any side effects of its previous 
    // state, otherwise trains run away or just dont move.  
    if (mode==TRACK_MODE_DC || mode==TRACK_MODE_DCX) {
        // DC tracks need to be given speed of the throttle for that cab address
        // otherwise will not match other tracks on same cab.
        // This also needs to allow for inverted DCX
        applyDCSpeed(trackToSet);
 
    }
    else {
      // DCC tracks need to have set the PWM to zero or they will not work.
      // 128 is speed=0 and dir=0 and then loosen brake.
      track[trackToSet]->setDCSignal(128);
      track[trackToSet]->setBrake(false);
    }

    // EXT is a special case where the signal pin is
    // turned off. So unless that is set, the signal
    // pin should be turned on
    track[trackToSet]->enableSignal(mode != TRACK_MODE_EXT);

    // re-evaluate HighAccuracy mode
    // We can only do this is all main and prog tracks agree
    bool canDo=true;
    FOR_EACH_TRACK(t) {
      // DC tracks must not have the DCC PWM switched on
      // so we globally turn it off if one of the PWM
      // capable tracks is now DC or DCX.
      if (trackMode[t]==TRACK_MODE_DC || trackMode[t]==TRACK_MODE_DCX) {
	if (track[t]->isPWMCapable()) {
	  canDo=false;
	  break;
	}
      } else if (trackMode[t]==TRACK_MODE_MAIN || trackMode[t]==TRACK_MODE_PROG)
	canDo &= track[t]->isPWMCapable();
    }
    //DIAG(F("HAMode=%d"),canDo);
    if (!canDo) {
      DCCTimer::clearPWM();
    }
    if (MotorDriver::usePWM != canDo)
      DIAG(F("HA mode changed from %d to %d"), MotorDriver::usePWM, canDo);
    MotorDriver::usePWM=canDo;

    
    // Normal running tracks are set to the global power state 
    track[trackToSet]->setPower(
        (mode==TRACK_MODE_MAIN || mode==TRACK_MODE_DC || mode==TRACK_MODE_DCX || mode==TRACK_MODE_EXT) ?
        mainPowerGuess : POWERMODE::OFF);
    DIAG(F("TrackMode=%d"),mode);
    return true; 
}

void TrackManager::applyDCSpeed(byte t) {
  uint8_t speedByte=DCC::getThrottleSpeedByte(trackDCAddr[t]);
  if (trackMode[t]==TRACK_MODE_DCX)
    speedByte = (speedByte & 0xF7) | ~(speedByte & 0x80); // Reverse highest bit
  track[t]->setDCSignal(speedByte);
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
                    case TRACK_MODE_EXT:
                        StringFormatter::send(stream,F("EXT"));
                        break;
                    case TRACK_MODE_DC:
                        StringFormatter::send(stream,F("DC %d"),trackDCAddr[t]);
                        break;
                    case TRACK_MODE_DCX:
                        StringFormatter::send(stream,F("DCX %d"),trackDCAddr[t]);
                        break;
                    default:
                        break; // unknown, dont care    
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

    if (params==2  && p[1]==HASH_KEYWORD_EXT) // <= id EXT>
        return setTrackMode(p[0],TRACK_MODE_EXT);

    if (params==3  && p[1]==HASH_KEYWORD_DC && p[2]>0) // <= id DC cab>
        return setTrackMode(p[0],TRACK_MODE_DC,p[2]);
    
    if (params==3  && p[1]==HASH_KEYWORD_DCX && p[2]>0) // <= id DCX cab>
        return setTrackMode(p[0],TRACK_MODE_DCX,p[2]);

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
    if (!setProg) mainPowerGuess=mode; 
    FOR_EACH_TRACK(t) {
        MotorDriver * driver=track[t]; 
        if (!driver) continue; 
        switch (trackMode[t]) {
            case TRACK_MODE_MAIN:
                if (setProg) break; 
                // toggle brake before turning power on - resets overcurrent error
                // on the Pololu board if brake is wired to ^D2.
		// XXX see if we can make this conditional
                driver->setBrake(true);
                driver->setBrake(false); // DCC runs with brake off
                driver->setPower(mode);  
                break; 
            case TRACK_MODE_DC:
            case TRACK_MODE_DCX:
                if (setProg) break; 
                driver->setBrake(true); // DC starts with brake on
                applyDCSpeed(t);        // speed match DCC throttles
                driver->setPower(mode);
                break;  
            case TRACK_MODE_PROG:
                if (!setProg) break; 
                driver->setBrake(true);
                driver->setBrake(false);
                driver->setPower(mode);
                break;  
            case TRACK_MODE_EXT:
	        driver->setBrake(true);
	        driver->setBrake(false);
		driver->setPower(mode);
	        break;
            case TRACK_MODE_OFF:
                break;
        }
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
