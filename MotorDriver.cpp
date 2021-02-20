/*
 *  © 2020, Chris Harlow. All rights reserved.
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
  pinMode(currentPin, INPUT);

  faultPin=fault_pin;
  if (faultPin != UNUSED_PIN) {
    getFastPin(F("FAULT"),faultPin, 1 /*input*/, fastFaultPin);
    pinMode(faultPin, INPUT);
  }

  senseFactor=sense_factor;
  tripMilliamps=trip_milliamps;
  rawCurrentTripValue=(int)(trip_milliamps / sense_factor);
}

bool MotorDriver::isPWMCapable() {
    return (!dualSignal) && DCCTimer::isPWMPin(signalPin); 
}


void MotorDriver::setPower(bool on) {
  if (on) {
    // toggle brake before turning power on - resets overcurrent error
    // on the Pololu board if brake is wired to ^D2.
    setBrake(true);
    setBrake(false);
    setHIGH(fastPowerPin);
  }
  else setLOW(fastPowerPin);
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

/*
 * Return the current reading as pin reading 0 to 1023. If the fault
 * pin is activated return a negative current to show active fault pin.
 * As there is no -0, ceat a little and return -1 in that case.
 */
int MotorDriver::getCurrentRaw() {
  int current = analogRead(currentPin);
  if (faultPin != UNUSED_PIN && isLOW(fastFaultPin) && isHIGH(fastPowerPin))
      return (current == 0 ? -1 : -current);
  return current;
  // IMPORTANT:  This function can be called in Interrupt() time within the 56uS timer
  //             The default analogRead takes ~100uS which is catastrphic
  //             so DCCTimer has set the sample time to be much faster.  
}

unsigned int MotorDriver::raw2mA( int raw) {
  return (unsigned int)(raw * senseFactor);
}
int MotorDriver::mA2raw( unsigned int mA) {
  return (int)(mA / senseFactor);
}

void  MotorDriver::getFastPin(const FSH* type,int pin, bool input, FASTPIN & result) {
    DIAG(F("\nMotorDriver %S Pin=%d,"),type,pin);
    uint8_t port = digitalPinToPort(pin);
    if (input)
      result.inout = portInputRegister(port);
    else
      result.inout = portOutputRegister(port);
    result.maskHIGH = digitalPinToBitMask(pin);
    result.maskLOW = ~result.maskHIGH;
    DIAG(F(" port=0x%x, inoutpin=0x%x, isinput=%d, mask=0x%x\n"),port, result.inout,input,result.maskHIGH);
}
