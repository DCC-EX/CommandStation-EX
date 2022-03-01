/*
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
#include <Arduino.h>
#include "MotorDriver.h"
#include "DCCTimer.h"
#include "DIAG.h"

#define setHIGH(fastpin)  *fastpin.inout |= fastpin.maskHIGH
#define setLOW(fastpin)   *fastpin.inout &= fastpin.maskLOW
#define isHIGH(fastpin)   (*fastpin.inout & fastpin.maskHIGH)
#define isLOW(fastpin)    (!isHIGH(fastpin))

bool MotorDriver::usePWM=false;
bool MotorDriver::commonFaultPin=false;
       
MotorDriver::MotorDriver(byte power_pin, byte signal_pin, byte signal_pin2, int8_t brake_pin,
                         byte current_pin, float sense_factor, unsigned int trip_milliamps, byte fault_pin) {
  powerPin=power_pin;
  getFastPin(F("POWER"),powerPin,fastPowerPin);
  pinMode(powerPin, OUTPUT);
  
  signalPin=signal_pin;
  getFastPin(F("SIG"),signalPin,fastSignalPin);
  pinMode(signalPin, OUTPUT);
  
  signalPin2=signal_pin2;
  if (signalPin2!=UNUSED_PIN) {
    dualSignal=true;
    getFastPin(F("SIG2"),signalPin2,fastSignalPin2);
    pinMode(signalPin2, OUTPUT);
  }
  else dualSignal=false; 
  
  brakePin=brake_pin;
  if (brake_pin!=UNUSED_PIN){
    invertBrake=brake_pin < 0;
    brakePin=invertBrake ? 0-brake_pin : brake_pin;
    getFastPin(F("BRAKE"),brakePin,fastBrakePin);
    pinMode(brakePin, OUTPUT);
    setBrake(false);
  }
  else brakePin=UNUSED_PIN;
  
  currentPin=current_pin;
  if (currentPin!=UNUSED_PIN) {
    pinMode(currentPin, INPUT);
    senseOffset=analogRead(currentPin); // value of sensor at zero current
  }

  faultPin=fault_pin;
  if (faultPin != UNUSED_PIN) {
    getFastPin(F("FAULT"),faultPin, 1 /*input*/, fastFaultPin);
    pinMode(faultPin, INPUT);
  }

  senseFactor=sense_factor;
  tripMilliamps=trip_milliamps;
  rawCurrentTripValue=(int)(trip_milliamps / sense_factor);
  
  if (currentPin==UNUSED_PIN) 
    DIAG(F("MotorDriver ** WARNING ** No current or short detection"));  
  else  
    DIAG(F("MotorDriver currentPin=A%d, senseOffset=%d, rawCurrentTripValue(relative to offset)=%d"),
    currentPin-A0, senseOffset,rawCurrentTripValue);

  // prepare values for current detection
  sampleDelay = 0;
  lastSampleTaken = millis();
  progTripValue = mA2raw(TRIP_CURRENT_PROG); 

}

bool MotorDriver::isPWMCapable() {
    return (!dualSignal) && DCCTimer::isPWMPin(signalPin); 
}


void MotorDriver::setPower(POWERMODE mode) {
  bool on=mode==POWERMODE::ON;
  if (on) {
    // toggle brake before turning power on - resets overcurrent error
    // on the Pololu board if brake is wired to ^D2.
    setBrake(true);
    setBrake(false);
    setHIGH(fastPowerPin);
  }
  else setLOW(fastPowerPin);
  powerMode=mode; 
}

// setBrake applies brake if on == true. So to get
// voltage from the motor bride one needs to do a
// setBrake(false).
// If the brakePin is negative that means the sense
// of the brake pin on the motor bridge is inverted
// (HIGH == release brake) and setBrake does
// compensate for that.
//
void MotorDriver::setBrake(bool on) {
  if (brakePin == UNUSED_PIN) return;
  if (on ^ invertBrake) setHIGH(fastBrakePin);
  else setLOW(fastBrakePin);
}

void MotorDriver::setSignal( bool high) {
   if (usePWM) {
    DCCTimer::setPWM(signalPin,high);
   }
   else {
     if (high) {
        setHIGH(fastSignalPin);
        if (dualSignal) setLOW(fastSignalPin2);
     }
     else {
        setLOW(fastSignalPin);
        if (dualSignal) setHIGH(fastSignalPin2);
     }
   }
}


bool MotorDriver::canMeasureCurrent() {
  return currentPin!=UNUSED_PIN;
}
/*
 * Return the current reading as pin reading 0 to 1023. If the fault
 * pin is activated return a negative current to show active fault pin.
 * As there is no -0, create a little and return -1 in that case.
 * 
 * senseOffset handles the case where a shield returns values above or below 
 * a central value depending on direction.
 */
int MotorDriver::getCurrentRaw() {
  if (currentPin==UNUSED_PIN) return 0; 
  int current;
  // This function should NOT be called in an interruot so we 
  // dont need to fart about saving and restoring CPU specific 
  // interrupt registers. 
  noInterrupts();
  current = analogRead(currentPin)-senseOffset;
  interrupts();
  if (current<0) current=0-current;
  if ((faultPin != UNUSED_PIN)  && isLOW(fastFaultPin) && isHIGH(fastPowerPin))
      return (current == 0 ? -1 : -current);
  return current;
   
}

void MotorDriver::setDCSignal(byte speedcode) {
  // spedcxode is a dcc speed /direction
  // TODO jiggle the DC speed pin PWMs 
  
}
int MotorDriver::getCurrentRawInInterrupt() {
  
  // IMPORTANT:  This function must be called in Interrupt() time within the 56uS timer
  //             The default analogRead takes ~100uS which is catastrphic
  //             so DCCTimer has set the sample time to be much faster.  

  if (currentPin==UNUSED_PIN) return 0; 
  return analogRead(currentPin)-senseOffset;
}  

unsigned int MotorDriver::raw2mA( int raw) {
  return (unsigned int)(raw * senseFactor);
}
int MotorDriver::mA2raw( unsigned int mA) {
  return (int)(mA / senseFactor);
}

void  MotorDriver::getFastPin(const FSH* type,int pin, bool input, FASTPIN & result) {
    // DIAG(F("MotorDriver %S Pin=%d,"),type,pin);
    (void) type; // avoid compiler warning if diag not used above. 
    uint8_t port = digitalPinToPort(pin);
    if (input)
      result.inout = portInputRegister(port);
    else
      result.inout = portOutputRegister(port);
    result.maskHIGH = digitalPinToBitMask(pin);
    result.maskLOW = ~result.maskHIGH;
    // DIAG(F(" port=0x%x, inoutpin=0x%x, isinput=%d, mask=0x%x"),port, result.inout,input,result.maskHIGH);
}

void MotorDriver::checkPowerOverload(bool useProgLimit, byte trackno) {
  if (millis() - lastSampleTaken  < sampleDelay) return;
  lastSampleTaken = millis();
  int tripValue= useProgLimit?progTripValue:getRawCurrentTripValue();
  
  // Trackname for diag messages later
  switch (powerMode) {
    case POWERMODE::OFF:
      sampleDelay = POWER_SAMPLE_OFF_WAIT;
      break;
    case POWERMODE::ON:
      // Check current
      lastCurrent=getCurrentRaw();
      if (lastCurrent < 0) {
	  // We have a fault pin condition to take care of
	  lastCurrent = -lastCurrent;
	  setPower(POWERMODE::OVERLOAD); // Turn off, decide later how fast to turn on again
	  if (commonFaultPin) {
	      if (lastCurrent <= tripValue) {
		      setPower(POWERMODE::ON); // maybe other track
	      }
	      // Write this after the fact as we want to turn on as fast as possible
	      // because we don't know which output actually triggered the fault pin
	      DIAG(F("COMMON FAULT PIN ACTIVE - TOGGLED POWER on %d"), trackno);
	  } else {
	    DIAG(F("TRACK %d FAULT PIN ACTIVE - OVERLOAD"), trackno);
	      if (lastCurrent < tripValue) {
		  lastCurrent = tripValue; // exaggerate
	      }
	  }
      }
      if (lastCurrent < tripValue) {
        sampleDelay = POWER_SAMPLE_ON_WAIT;
	if(power_good_counter<100)
	  power_good_counter++;
	else
	  if (power_sample_overload_wait>POWER_SAMPLE_OVERLOAD_WAIT) power_sample_overload_wait=POWER_SAMPLE_OVERLOAD_WAIT;
      } else {
        setPower(POWERMODE::OVERLOAD);
        unsigned int mA=raw2mA(lastCurrent);
        unsigned int maxmA=raw2mA(tripValue);
	      power_good_counter=0;
        sampleDelay = power_sample_overload_wait;
        DIAG(F("TRACK %d POWER OVERLOAD current=%d max=%d offtime=%d"), trackno, mA, maxmA, sampleDelay);
	if (power_sample_overload_wait >= 10000)
	    power_sample_overload_wait = 10000;
	else
	    power_sample_overload_wait *= 2;
      }
      break;
    case POWERMODE::OVERLOAD:
      // Try setting it back on after the OVERLOAD_WAIT
      setPower(POWERMODE::ON);
      sampleDelay = POWER_SAMPLE_ON_WAIT;
      // Debug code....
      DIAG(F("TRACK %d POWER RESET delay=%d"), trackno, sampleDelay);
      break;
    default:
      sampleDelay = 999; // cant get here..meaningless statement to avoid compiler warning.
  }
}
