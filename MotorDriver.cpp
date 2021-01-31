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
#include "AnalogReadFast.h"
#include "DIAG.h"

#define setHIGH(fastpin)  *fastpin.out |= fastpin.maskHIGH
#define setLOW(fastpin)   *fastpin.out &= fastpin.maskLOW
    
MotorDriver::MotorDriver(byte power_pin, byte signal_pin, byte signal_pin2, int8_t brake_pin,
                         byte current_pin, float sense_factor, unsigned int trip_milliamps, byte fault_pin) {
  powerPin=power_pin;
  signalPin=signal_pin;
  getFastPin(signalPin,fastSignalPin);
  signalPin2=signal_pin2;
  if (signalPin2!=UNUSED_PIN) {
    dualSignal=true;
    getFastPin(signalPin2,fastSignalPin2);
  }
  else dualSignal=false; 
  
  brakePin=brake_pin;
  currentPin=current_pin;
  senseFactor=sense_factor;
  faultPin=fault_pin;
  tripMilliamps=trip_milliamps;
  rawCurrentTripValue=(int)(trip_milliamps / sense_factor);
  simulatedOverload=(int)(32000/senseFactor);
  pinMode(powerPin, OUTPUT);
  pinMode(brakePin < 0 ? -brakePin : brakePin, OUTPUT);
  setBrake(false);
  pinMode(signalPin, OUTPUT);
  if (signalPin2 != UNUSED_PIN) pinMode(signalPin2, OUTPUT);
  pinMode(currentPin, INPUT);
  if (faultPin != UNUSED_PIN) pinMode(faultPin, INPUT);
}

void MotorDriver::setPower(bool on) {
  if (brakePin == -4 && on) {
    // toggle brake before turning power on - resets overcurrent error
    // on the Pololu board if brake is wired to ^D2.
    setBrake(true);
    setBrake(false);
  }
  digitalWrite(powerPin, on ? HIGH : LOW);
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
    bool state = on;
    byte pin = brakePin;
    if (brakePin < 0) {
      pin=-pin;
      state=!state;
    }
    digitalWrite(pin, state ? HIGH : LOW);
}

void MotorDriver::setSignal( bool high) {
   if (high) {
      setHIGH(fastSignalPin);
      if (dualSignal) setLOW(fastSignalPin2);
   }
   else {
      setLOW(fastSignalPin);
      if (dualSignal) setHIGH(fastSignalPin2);
   }
}


int MotorDriver::getCurrentRaw() {
  if (faultPin != UNUSED_PIN && digitalRead(faultPin) == LOW && digitalRead(powerPin) == HIGH)
      return simulatedOverload;
  
  // IMPORTANT:  This function can be called in Interrupt() time within the 56uS timer
  //             The default analogRead takes ~100uS which is catastrphic
  //             so analogReadFast is used here. (-2uS) 
  return analogReadFast(currentPin);
}

unsigned int MotorDriver::raw2mA( int raw) {
  return (unsigned int)(raw * senseFactor);
}
int MotorDriver::mA2raw( unsigned int mA) {
  return (int)(mA / senseFactor);
}

void  MotorDriver::getFastPin(int pin, FASTPIN & result) {
    DIAG(F("\nMotorDriver Pin=%d,"),pin);
    uint8_t port = digitalPinToPort(pin);
    result.out = portOutputRegister(port);
    result.maskHIGH = digitalPinToBitMask(pin);
    result.maskLOW = ~result.maskHIGH;
    DIAG(F(" port=0x%x, out=0x%x, mask=0x%x\n"),port, result.out,result.maskHIGH);
}
