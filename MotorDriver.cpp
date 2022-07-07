/*
 *  © 2022 Paul M Antoine
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

bool MotorDriver::commonFaultPin=false;

volatile portreg_t shadowPORTA;
volatile portreg_t shadowPORTB;
volatile portreg_t shadowPORTC;

MotorDriver::MotorDriver(VPIN power_pin, byte signal_pin, byte signal_pin2, int8_t brake_pin,
                         byte current_pin, float sense_factor, unsigned int trip_milliamps, byte fault_pin) {
  powerPin=power_pin;
  IODevice::write(powerPin,LOW);// set to OUTPUT and off 
  
  signalPin=signal_pin;
  getFastPin(F("SIG"),signalPin,fastSignalPin);
  pinMode(signalPin, OUTPUT);

  fastSignalPin.shadowinout = NULL;
  if (HAVE_PORTA(fastSignalPin.inout == &PORTA)) {
    DIAG(F("Found PORTA pin %d"),signalPin);
    fastSignalPin.shadowinout = fastSignalPin.inout;
    fastSignalPin.inout = &shadowPORTA;
  }
  if (HAVE_PORTB(fastSignalPin.inout == &PORTB)) {
    DIAG(F("Found PORTB pin %d"),signalPin);
    fastSignalPin.shadowinout = fastSignalPin.inout;
    fastSignalPin.inout = &shadowPORTB;
  }
  if (HAVE_PORTC(fastSignalPin.inout == &PORTC)) {
    DIAG(F("Found PORTC pin %d"),signalPin);
    fastSignalPin.shadowinout = fastSignalPin.inout;
    fastSignalPin.inout = &shadowPORTC;
  }

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
    // if brake is used for railcom  cutout we need to do PORTX register trick here as well
    pinMode(brakePin, OUTPUT);
    setBrake(true);  // start with brake on in case we hace DC stuff going on
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

  // This conversion performed at compile time so the remainder of the code never needs
  // float calculations or libraray code. 
  senseFactorInternal=sense_factor * senseScale; 
  tripMilliamps=trip_milliamps;
  rawCurrentTripValue=mA2raw(trip_milliamps);
  
  if (currentPin==UNUSED_PIN) 
    DIAG(F("MotorDriver ** WARNING ** No current or short detection"));  
  else  {
    DIAG(F("MotorDriver currentPin=A%d, senseOffset=%d, rawCurrentTripValue(relative to offset)=%d"),
    currentPin-A0, senseOffset,rawCurrentTripValue);

    // self testing diagnostic for the non-float converters... may be removed when happy
    //  DIAG(F("senseFactorInternal=%d raw2mA(1000)=%d mA2Raw(1000)=%d"),
    //   senseFactorInternal, raw2mA(1000),mA2raw(1000));
  }

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
    IODevice::write(powerPin,HIGH);
    if (resetsCounterP != NULL)
      *resetsCounterP = 0;
  }
  else IODevice::write(powerPin,LOW);
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
  if ((faultPin != UNUSED_PIN)  && isLOW(fastFaultPin) && powerMode==POWERMODE::ON)
      return (current == 0 ? -1 : -current);
  return current;
   
}

void MotorDriver::setDCSignal(byte speedcode) {
  if (brakePin == UNUSED_PIN)
    return;
  // spedcoode is a dcc speed & direction
  byte tSpeed=speedcode & 0x7F; // DCC Speed with 0,1 stop and speed steps 2 to 127
  byte tDir=speedcode & 0x80;
  byte brake;
  if (tSpeed <= 1) brake = 255;
  else if (tSpeed >= 127) brake = 0;
  else  brake = 2 * (128-tSpeed);
  if (invertBrake)
    brake=255-brake;
  analogWrite(brakePin,brake);
  // as the port registers can be shadowed to get syncronized DCC signals
  // we need to take care of that and we have to turn off interrupts during
  // that time as otherwise setDCCSignal() which is called from interrupt
  // contect can undo whatever we do here.
  if (fastSignalPin.shadowinout != NULL) {
    if (HAVE_PORTA(fastSignalPin.shadowinout == &PORTA)) {
      noInterrupts();
      HAVE_PORTA(shadowPORTA=PORTA);
      setSignal(tDir);
      HAVE_PORTA(PORTA=shadowPORTA);
      interrupts();
    } else if (HAVE_PORTB(fastSignalPin.shadowinout == &PORTB)) {
      noInterrupts();
      HAVE_PORTB(shadowPORTB=PORTB);
      setSignal(tDir);
      HAVE_PORTB(PORTB=shadowPORTB);
      interrupts();
    } else if (HAVE_PORTC(fastSignalPin.shadowinout == &PORTC)) {
      noInterrupts();
      HAVE_PORTC(shadowPORTC=PORTC);
      setSignal(tDir);
      HAVE_PORTC(PORTC=shadowPORTC);
      interrupts();
    }
  } else {
    setSignal(tDir);
  }
}

int MotorDriver::getCurrentRawInInterrupt() {
  
  // IMPORTANT:  This function must be called in Interrupt() time within the 56uS timer
  //             The default analogRead takes ~100uS which is catastrphic
  //             so DCCTimer has set the sample time to be much faster.  

  if (currentPin==UNUSED_PIN) return 0; 
  return analogRead(currentPin)-senseOffset;
}  

unsigned int MotorDriver::raw2mA( int raw) {
  return (int32_t)raw * senseFactorInternal / senseScale;
}
unsigned int MotorDriver::mA2raw( unsigned int mA) {
  return (int32_t)mA * senseScale / senseFactorInternal;
}

void  MotorDriver::getFastPin(const FSH* type,int pin, bool input, FASTPIN & result) {
    // DIAG(F("MotorDriver %S Pin=%d,"),type,pin);
    (void) type; // avoid compiler warning if diag not used above.
#if defined(ARDUINO_ARCH_SAMD)
    PortGroup *port = digitalPinToPort(pin);
#else
    uint8_t port = digitalPinToPort(pin);
#endif
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
