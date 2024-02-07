/*
 *  © 2022 Chris Harlow
 *  © 2022,2023 Harald Barth
 *  © 2023 Colin Murdoch
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
#include "CommandDistributor.h"
#include "DCCEXParser.h"
#include "KeywordHasher.h"
// Virtualised Motor shield multi-track hardware Interface
#define FOR_EACH_TRACK(t) for (byte t=0;t<=lastTrack;t++)
    
#define APPLY_BY_MODE(findmode,function) \
        FOR_EACH_TRACK(t) \
	    if (track[t]->getMode()==findmode)	\
                track[t]->function;

MotorDriver * TrackManager::track[MAX_TRACKS];
int16_t TrackManager::trackDCAddr[MAX_TRACKS];

byte TrackManager::lastTrack=0;
bool TrackManager::progTrackSyncMain=false; 
bool TrackManager::progTrackBoosted=false; 
int16_t TrackManager::joinRelay=UNUSED_PIN;
#ifdef ARDUINO_ARCH_ESP32
byte TrackManager::tempProgTrack=MAX_TRACKS+1; // MAX_TRACKS+1 is the unused flag
#endif

#ifdef ANALOG_READ_INTERRUPT
/*
 * sampleCurrent() runs from Interrupt
 */
void TrackManager::sampleCurrent() {
  static byte tr = 0;
  byte trAtStart = tr;
  static bool waiting = false;

  if (waiting) {
    if (! track[tr]->sampleCurrentFromHW()) {
      return; // no result, continue to wait
    }
    // found value, advance at least one track
    // for scope debug track[1]->setBrake(0);
    waiting = false;
    tr++;
    if (tr > lastTrack) tr = 0;
    if (lastTrack < 2 || track[tr]->getMode() & TRACK_MODE_PROG) {
      return; // We could continue but for prog track we
              // rather do it in next interrupt beacuse
              // that gives us well defined sampling point.
              // For other tracks we care less unless we
              // have only few (max 2) tracks.
    }
  }
  if (!waiting) {
    // look for a valid track to sample or until we are around
    while (true) {
      if (track[tr]->getMode() & ( TRACK_MODE_MAIN|TRACK_MODE_PROG|TRACK_MODE_DC|TRACK_MODE_BOOST|TRACK_MODE_EXT )) {
	track[tr]->startCurrentFromHW();
	// for scope debug track[1]->setBrake(1);
	waiting = true;
	break;
      }
      tr++;
      if (tr > lastTrack) tr = 0;
      if (tr == trAtStart) // we are through and nothing found to do
	return;
    }
  }
}
#endif

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
#ifndef DISABLE_PROG
    setTrackMode(1,TRACK_MODE_PROG);
#else
    setTrackMode(1,TRACK_MODE_MAIN);
#endif
  
  // Fault pin config for odd motor boards (example pololu)
  FOR_EACH_TRACK(t) {
    for (byte s=t+1;s<=lastTrack;s++) {
      if (track[t]->getFaultPin() != UNUSED_PIN &&
	  track[t]->getFaultPin() == track[s]->getFaultPin()) {
	track[t]->setCommonFaultPin();
	track[s]->setCommonFaultPin();
	DIAG(F("Common Fault pin tracks %c and %c"), t+'A', s+'A');
      }
    }
  }
  DCC::setShieldName(shieldname);
}

void TrackManager::addTrack(byte t, MotorDriver* driver) {
     track[t]=driver;
     if (driver) {
         track[t]->setPower(POWERMODE::OFF);
         track[t]->setMode(TRACK_MODE_NONE);
	 track[t]->setTrackLetter('A'+t);
         lastTrack=t;
     } 
}

// setDCCSignal(), called from interrupt context
// does assume ports are shadowed if they can be
void TrackManager::setDCCSignal( bool on) {
  HAVE_PORTA(shadowPORTA=PORTA);
  HAVE_PORTB(shadowPORTB=PORTB);
  HAVE_PORTC(shadowPORTC=PORTC);
  HAVE_PORTD(shadowPORTD=PORTD);
  HAVE_PORTE(shadowPORTE=PORTE);
  HAVE_PORTF(shadowPORTF=PORTF);
  APPLY_BY_MODE(TRACK_MODE_MAIN,setSignal(on));
  HAVE_PORTA(PORTA=shadowPORTA);
  HAVE_PORTB(PORTB=shadowPORTB);
  HAVE_PORTC(PORTC=shadowPORTC);
  HAVE_PORTD(PORTD=shadowPORTD);
  HAVE_PORTE(PORTE=shadowPORTE);
  HAVE_PORTF(PORTF=shadowPORTF);
}

// setPROGSignal(), called from interrupt context
// does assume ports are shadowed if they can be
void TrackManager::setPROGSignal( bool on) {
  HAVE_PORTA(shadowPORTA=PORTA);
  HAVE_PORTB(shadowPORTB=PORTB);
  HAVE_PORTC(shadowPORTC=PORTC);
  HAVE_PORTD(shadowPORTD=PORTD);
  HAVE_PORTE(shadowPORTE=PORTE);
  HAVE_PORTF(shadowPORTF=PORTF);
  APPLY_BY_MODE(TRACK_MODE_PROG,setSignal(on));
  HAVE_PORTA(PORTA=shadowPORTA);
  HAVE_PORTB(PORTB=shadowPORTB);
  HAVE_PORTC(PORTC=shadowPORTC);
  HAVE_PORTD(PORTD=shadowPORTD);
  HAVE_PORTE(PORTE=shadowPORTE);
  HAVE_PORTF(PORTF=shadowPORTF);
}

// setDCSignal(), called from normal context
// MotorDriver::setDCSignal handles shadowed IO port changes.
// with interrupts turned off around the critical section
void TrackManager::setDCSignal(int16_t cab, byte speedbyte) {
  FOR_EACH_TRACK(t) {
    if (trackDCAddr[t]!=cab && cab != 0) continue;
    if (track[t]->getMode() & TRACK_MODE_DC)
      track[t]->setDCSignal(speedbyte);
  }
}    

bool TrackManager::setTrackMode(byte trackToSet, TRACK_MODE mode, int16_t dcAddr) {
    if (trackToSet>lastTrack || track[trackToSet]==NULL) return false;

    // Remember track mode we came from for later
    TRACK_MODE oldmode = track[trackToSet]->getMode();

    //DIAG(F("Track=%c Mode=%d"),trackToSet+'A', mode);
    // DC tracks require a motorDriver that can set brake!
    if (mode & TRACK_MODE_DC) {
#if defined(ARDUINO_AVR_UNO)
      DIAG(F("Uno has no PWM timers available for DC"));
      return false;
#endif
      if (!track[trackToSet]->brakeCanPWM()) {
	DIAG(F("Brake pin can't PWM: No DC"));
	return false;
      }
    }

#ifdef ARDUINO_ARCH_ESP32
    // remove pin from MUX matrix and turn it off
    pinpair p = track[trackToSet]->getSignalPin();
    //DIAG(F("Track=%c remove  pin %d"),trackToSet+'A', p.pin);
    gpio_reset_pin((gpio_num_t)p.pin);
    if (p.invpin != UNUSED_PIN) {
      //DIAG(F("Track=%c remove ^pin %d"),trackToSet+'A', p.invpin);
      gpio_reset_pin((gpio_num_t)p.invpin);
    }
#ifdef BOOSTER_INPUT
    if (mode & TRACK_MODE_BOOST) {
      //DIAG(F("Track=%c mode boost pin %d"),trackToSet+'A', p.pin);
      pinMode(BOOSTER_INPUT, INPUT);
      gpio_matrix_in(26, SIG_IN_FUNC228_IDX, false); //pads 224 to 228 available as loopback
      gpio_matrix_out(p.pin, SIG_IN_FUNC228_IDX, false, false);
      if (p.invpin != UNUSED_PIN) {
	gpio_matrix_out(p.invpin, SIG_IN_FUNC228_IDX, true /*inverted*/, false);
      }
    } else // elseif clause continues
#endif
    if (mode & (TRACK_MODE_MAIN | TRACK_MODE_PROG | TRACK_MODE_DC)) {
      // gpio_reset_pin may reset to input
      pinMode(p.pin, OUTPUT);
      if (p.invpin != UNUSED_PIN)
	pinMode(p.invpin, OUTPUT);
    }

#endif
#ifndef DISABLE_PROG
    if (mode & TRACK_MODE_PROG) {
#else
    if (false) {
#endif
      // only allow 1 track to be prog
      FOR_EACH_TRACK(t)
	if ( (track[t]->getMode() & TRACK_MODE_PROG) && t != trackToSet) {
	  track[t]->setPower(POWERMODE::OFF);
	  track[t]->setMode(TRACK_MODE_NONE);
	  track[t]->makeProgTrack(false);     // revoke prog track special handling
	  streamTrackState(NULL,t);
	}
      track[trackToSet]->makeProgTrack(true); // set for prog track special handling
    } else {
      track[trackToSet]->makeProgTrack(false); // only the prog track knows it's type
    }
    track[trackToSet]->setMode(mode);
    trackDCAddr[trackToSet]=dcAddr;

    // When a track is switched, we must clear any side effects of its previous 
    // state, otherwise trains run away or just dont move.

    // This can be done BEFORE the PWM-Timer evaluation (methinks)
    if (!(mode & TRACK_MODE_DC)) {
      // DCC tracks need to have set the PWM to zero or they will not work.
      track[trackToSet]->detachDCSignal();
      track[trackToSet]->setBrake(false);
    }

    // BOOST:
    //  Leave it as is
    // otherwise:
    //  EXT is a special case where the signal pin is
    //  turned off. So unless that is set, the signal
    //  pin should be turned on
    if (!(mode & TRACK_MODE_BOOST))
      track[trackToSet]->enableSignal(!(mode & TRACK_MODE_EXT));

#ifndef ARDUINO_ARCH_ESP32
    // re-evaluate HighAccuracy mode
    // We can only do this is all main and prog tracks agree
    bool canDo=true;
    FOR_EACH_TRACK(t) {
      // DC tracks must not have the DCC PWM switched on
      // so we globally turn it off if one of the PWM
      // capable tracks is now DC or DCX.
      if (track[t]->getMode() & TRACK_MODE_DC) {
	if (track[t]->isPWMCapable()) {
	  canDo=false;    // this track is capable but can not run PWM
	  break;          // in this mode, so abort and prevent globally below
	} else {
	  track[t]->trackPWM=false; // this track sure can not run with PWM
	  //DIAG(F("Track %c trackPWM 0 (not capable)"), t+'A');
	}
      } else if (track[t]->getMode() & (TRACK_MODE_MAIN |TRACK_MODE_PROG)) {
	track[t]->trackPWM = track[t]->isPWMCapable(); // trackPWM is still a guess here
	//DIAG(F("Track %c trackPWM %d"), t+'A', track[t]->trackPWM);
	canDo &= track[t]->trackPWM;
      }
    }
    if (!canDo) {
      // if we discover that HA mode was globally impossible
      // we must adjust the trackPWM capabilities
      FOR_EACH_TRACK(t) {
	track[t]->trackPWM=false;
	//DIAG(F("Track %c trackPWM 0 (global override)"), t+'A');
      }
      DCCTimer::clearPWM(); // has to be AFTER trackPWM changes because if trackPWM==true this is undone for  that track
    }
#else
    // For ESP32 we just reinitialize the DCC Waveform
    DCCWaveform::begin();
    // setMode() again AFTER Waveform::begin() of ESP32 fixes INVERTED signal
    track[trackToSet]->setMode(mode);
#endif

    // This block must be AFTER the PWM-Timer modifications
    if (mode & TRACK_MODE_DC) {
        // DC tracks need to be given speed of the throttle for that cab address
        // otherwise will not match other tracks on same cab.
        // This also needs to allow for inverted DCX
        applyDCSpeed(trackToSet);
    }

    // Turn off power if we changed the mode of this track
    if (mode != oldmode)
      track[trackToSet]->setPower(POWERMODE::OFF);
    streamTrackState(NULL,trackToSet);

    //DIAG(F("TrackMode=%d"),mode);
    return true; 
}

void TrackManager::applyDCSpeed(byte t) {
  uint8_t speedByte=DCC::getThrottleSpeedByte(trackDCAddr[t]);
  track[t]->setDCSignal(speedByte);
}

bool TrackManager::parseEqualSign(Print *stream, int16_t params, int16_t p[])
{
    
    if (params==0) { // <=>  List track assignments
        FOR_EACH_TRACK(t)
             streamTrackState(stream,t);
        return true;
        
    }
    
    p[0]-="A"_hk;  // convert A... to 0.... 

    if (params>1 && (p[0]<0 || p[0]>=MAX_TRACKS)) 
        return false;
    
    if (params==2  && p[1]=="MAIN"_hk) // <= id MAIN>
        return setTrackMode(p[0],TRACK_MODE_MAIN);
    
#ifndef DISABLE_PROG
    if (params==2  && p[1]=="PROG"_hk) // <= id PROG>
        return setTrackMode(p[0],TRACK_MODE_PROG);
#endif
    
    if (params==2  && (p[1]=="OFF"_hk || p[1]=="NONE"_hk)) // <= id OFF> <= id NONE>
        return setTrackMode(p[0],TRACK_MODE_NONE);

    if (params==2  && p[1]=="EXT"_hk) // <= id EXT>
        return setTrackMode(p[0],TRACK_MODE_EXT);
#ifdef BOOSTER_INPUT
    if (params==2  && p[1]=="BOOST"_hk) // <= id BOOST>
        return setTrackMode(p[0],TRACK_MODE_BOOST);
#endif
    if (params==2  && p[1]=="AUTO"_hk) // <= id AUTO>
      return setTrackMode(p[0], track[p[0]]->getMode() | TRACK_MODE_AUTOINV);

    if (params==2  && p[1]=="INV"_hk) // <= id AUTO>
      return setTrackMode(p[0], track[p[0]]->getMode() | TRACK_MODE_INV);

    if (params==3  && p[1]=="DC"_hk && p[2]>0) // <= id DC cab>
        return setTrackMode(p[0],TRACK_MODE_DC,p[2]);
    
    if (params==3  && p[1]=="DCX"_hk && p[2]>0) // <= id DCX cab>
        return setTrackMode(p[0],TRACK_MODE_DC|TRACK_MODE_INV,p[2]);

    return false;
}

const FSH* TrackManager::getModeName(TRACK_MODE tm) {
  const FSH *modename=F("---");
  
  if (tm & TRACK_MODE_MAIN) {
    if(tm & TRACK_MODE_AUTOINV)
      modename=F("MAIN A");
    else if (tm & TRACK_MODE_INV)
      modename=F("MAIN I>\n");
    else
      modename=F("MAIN");
  }
#ifndef DISABLE_PROG
  else if (tm & TRACK_MODE_PROG)
    modename=F("PROG");
#endif
  else if (tm & TRACK_MODE_NONE)
    modename=F("NONE");
  else if(tm & TRACK_MODE_EXT)
    modename=F("EXT");
  else if(tm & TRACK_MODE_BOOST) {
        if(tm & TRACK_MODE_AUTOINV)
      modename=F("B A");
    else if (tm & TRACK_MODE_INV)
      modename=F("B I");
    else
      modename=F("B");
  }
  else if (tm & TRACK_MODE_DC) {
    if (tm & TRACK_MODE_INV)
      modename=F("DCX");
    else
      modename=F("DC");
  }
  return modename;
}

// null stream means send to commandDistributor for broadcast
void TrackManager::streamTrackState(Print* stream, byte t) {
  const FSH *format;
  
  if (track[t]==NULL) return;
  TRACK_MODE tm = track[t]->getMode();
  if (tm & TRACK_MODE_DC)
    format=F("<= %c %S %d>\n");
  else
    format=F("<= %c %S>\n");

  const FSH *modename=getModeName(tm);
  if (stream) {  // null stream means send to commandDistributor for broadcast
    StringFormatter::send(stream,format,'A'+t, modename, trackDCAddr[t]);
  } else {
    CommandDistributor::broadcastTrackState(format,'A'+t, modename, trackDCAddr[t]);
    CommandDistributor::broadcastPower();
  }
  
}

byte TrackManager::nextCycleTrack=MAX_TRACKS;

void TrackManager::loop() {
    DCCWaveform::loop();
#ifndef DISABLE_PROG
    DCCACK::loop();
#endif
    bool dontLimitProg=DCCACK::isActive() || progTrackSyncMain || progTrackBoosted;
    nextCycleTrack++;
    if (nextCycleTrack>lastTrack) nextCycleTrack=0;
    if (track[nextCycleTrack]==NULL) return;
    MotorDriver * motorDriver=track[nextCycleTrack];
    bool useProgLimit=dontLimitProg ? false : (bool)(track[nextCycleTrack]->getMode() & TRACK_MODE_PROG);
    motorDriver->checkPowerOverload(useProgLimit, nextCycleTrack);   
}

MotorDriver * TrackManager::getProgDriver() {
    FOR_EACH_TRACK(t)
      if (track[t]->getMode() & TRACK_MODE_PROG) return track[t];
    return NULL;
} 

#ifdef ARDUINO_ARCH_ESP32
std::vector<MotorDriver *>TrackManager::getMainDrivers() {
  std::vector<MotorDriver *>  v;
  FOR_EACH_TRACK(t)
    if (track[t]->getMode() & TRACK_MODE_MAIN) v.push_back(track[t]);
  return v;
}
#endif

// Set track power for all tracks with this mode
void TrackManager::setTrackPower(TRACK_MODE trackmodeToMatch, POWERMODE powermode) {
  bool didChange=false;
  FOR_EACH_TRACK(t) {
    MotorDriver *driver=track[t];
    TRACK_MODE trackmodeOfTrack = driver->getMode();
    if (trackmodeToMatch & trackmodeOfTrack) {
      if (powermode != driver->getPower())
	didChange=true;
      if (powermode == POWERMODE::ON) {
	if (trackmodeOfTrack & TRACK_MODE_DC) {
	  driver->setBrake(true);   // DC starts with brake on
	  applyDCSpeed(t);          // speed match DCC throttles
	} else {
	  // toggle brake before turning power on - resets overcurrent error
	  // on the Pololu board if brake is wired to ^D2.
	  driver->setBrake(true);
	  driver->setBrake(false); // DCC runs with brake off
	}
      }
      driver->setPower(powermode);
    }
  }
  if (didChange)
    CommandDistributor::broadcastPower();
}

// Set track power for this track, inependent of mode
void TrackManager::setTrackPower(POWERMODE powermode, byte t) {
  MotorDriver *driver=track[t]; 
  TRACK_MODE trackmode = driver->getMode();
  POWERMODE oldpower = driver->getPower();
  if (trackmode & TRACK_MODE_NONE) {
    driver->setBrake(true);     // Track is unused. Brake is good to have.
    powermode = POWERMODE::OFF; // Track is unused. Force it to OFF
  } else if (trackmode & TRACK_MODE_DC) { // includes inverted DC (called DCX)
    if (powermode == POWERMODE::ON) {
      driver->setBrake(true);   // DC starts with brake on
      applyDCSpeed(t);          // speed match DCC throttles
    }
  } else /* MAIN PROG EXT BOOST */ {
    if (powermode == POWERMODE::ON) {
      // toggle brake before turning power on - resets overcurrent error
      // on the Pololu board if brake is wired to ^D2.
      driver->setBrake(true);
      driver->setBrake(false); // DCC runs with brake off
    }
  }
  driver->setPower(powermode);
  if (oldpower != driver->getPower())
    CommandDistributor::broadcastPower();
}

// returns state of the one and only prog track
POWERMODE TrackManager::getProgPower() {
  FOR_EACH_TRACK(t)
    if (track[t]->getMode() & TRACK_MODE_PROG)
      return track[t]->getPower(); // optimize: there is max one prog track
  return POWERMODE::OFF;
}

// returns on if all are on. returns off otherwise
POWERMODE TrackManager::getMainPower() {
  POWERMODE result = POWERMODE::OFF;
  FOR_EACH_TRACK(t) {
    if (track[t]->getMode() & TRACK_MODE_MAIN) {
      POWERMODE p = track[t]->getPower();
      if (p == POWERMODE::OFF)
	return POWERMODE::OFF; // done and out
      if (p == POWERMODE::ON)
	result = POWERMODE::ON;
    }
  }
  return result;
}

bool TrackManager::getPower(byte t, char s[]) {
  if (t > lastTrack)
    return false;
  if (track[t]) {
    s[0] = track[t]->getPower() == POWERMODE::ON ? '1' : '0';
    s[2] = t + 'A';
    return true;
  }
  return false;
}


void TrackManager::reportObsoleteCurrent(Print* stream) {
  // This function is for backward JMRI compatibility only
  // It reports the first track only, as main, regardless of track settings.
  //  <c MeterName value C/V unit min max res warn>
  int maxCurrent=track[0]->raw2mA(track[0]->getRawCurrentTripValue());
  StringFormatter::send(stream, F("<c CurrentMAIN %d C Milli 0 %d 1 %d>\n"), 
            track[0]->raw2mA(track[0]->getCurrentRaw(false)), maxCurrent, maxCurrent);                  
}

void TrackManager::reportCurrent(Print* stream) {
    StringFormatter::send(stream,F("<jI"));
    FOR_EACH_TRACK(t) {
         StringFormatter::send(stream, F(" %d"),
         (track[t]->getPower()==POWERMODE::OVERLOAD) ? -1 :
            track[t]->raw2mA(track[t]->getCurrentRaw(false)));
         }
    StringFormatter::send(stream,F(">\n"));    
}

void TrackManager::reportGauges(Print* stream) {
    StringFormatter::send(stream,F("<jG"));
    FOR_EACH_TRACK(t) {
         StringFormatter::send(stream, F(" %d"),
            track[t]->raw2mA(track[t]->getRawCurrentTripValue()));
         }
    StringFormatter::send(stream,F(">\n"));    
}

void TrackManager::setJoinRelayPin(byte joinRelayPin) {
  joinRelay=joinRelayPin;
  if (joinRelay!=UNUSED_PIN) {
    pinMode(joinRelay,OUTPUT);
    digitalWrite(joinRelay,LOW);  // LOW is relay disengaged
  }
}

void TrackManager::setJoin(bool joined) {
#ifdef ARDUINO_ARCH_ESP32
  if (joined) {
    FOR_EACH_TRACK(t) {
      if (track[t]->getMode() & TRACK_MODE_PROG) {
	tempProgTrack = t;
	setTrackMode(t, TRACK_MODE_MAIN);
	break;
      }
    }
  } else {
    if (tempProgTrack != MAX_TRACKS+1) {
      // as setTrackMode with TRACK_MODE_PROG defaults to
      // power off, we will take the current power state
      // of our track and then preserve that state.
      POWERMODE tPTmode = track[tempProgTrack]->getPower(); //get current power status of this track
      setTrackMode(tempProgTrack, TRACK_MODE_PROG);
      track[tempProgTrack]->setPower(tPTmode);              //set track status as it was before
      tempProgTrack = MAX_TRACKS+1;
    }
  }
#endif
  progTrackSyncMain=joined;
  if (joinRelay!=UNUSED_PIN) digitalWrite(joinRelay,joined?HIGH:LOW);
}

bool TrackManager::isPowerOn(byte t) {
      if (track[t]->getPower()!=POWERMODE::ON) 
	        return false;
    return true;   
  }

bool TrackManager::isProg(byte t) {
    if (track[t]->getMode() & TRACK_MODE_PROG)
        return true;
    return false;
}

TRACK_MODE TrackManager::getMode(byte t) {
    return (track[t]->getMode());
}

int16_t TrackManager::returnDCAddr(byte t) {
    return (trackDCAddr[t]);
}

