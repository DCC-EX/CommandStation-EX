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



#if defined(ARDUINO_ARCH_SAMD) || defined(ARDUINO_ARCH_SAMC) || defined(ARDUINO_ARCH_MEGAAVR)
    #define WritePin digitalWrite
    #define ReadPin digitalRead
#else
    // use the DIO2 libraray for much faster pin access
    #define GPIO2_PREFER_SPEED 1
    #include <DIO2.h>  // use IDE menu Tools..Manage Libraries to locate and  install DIO2
    #define WritePin digitalWrite2
    #define ReadPin digitalRead2
#endif
    
MotorDriver::MotorDriver(byte power_pin, byte signal_pin, byte signal_pin2, int8_t brake_pin,
                         byte current_pin, float sense_factor, unsigned int trip_milliamps, byte fault_pin) {
  powerPin=power_pin;
  signalPin=signal_pin;
  signalPin2=signal_pin2;
  brakePin=brake_pin;
  currentPin=current_pin;
  senseFactor=sense_factor;
  faultPin=fault_pin;
  tripMilliamps=trip_milliamps;
  rawCurrentTripValue=(int)(trip_milliamps / sense_factor);
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
  WritePin(powerPin, on ? HIGH : LOW);
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
    WritePin(pin, state ? HIGH : LOW);
    //DIAG(F("BrakePin: %d is %d\n"), pin, ReadPin(pin));
}

void MotorDriver::setSignal( bool high) {
  WritePin(signalPin, high ? HIGH : LOW);
  if (signalPin2 != UNUSED_PIN) WritePin(signalPin2, high ? LOW : HIGH);
}


int MotorDriver::getCurrentRaw() {
  if (faultPin != UNUSED_PIN && ReadPin(faultPin) == LOW && ReadPin(powerPin) == HIGH)
      return (int)(32000/senseFactor);
  
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
