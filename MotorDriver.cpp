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


#if defined(ARDUINO_ARCH_AVR)
    #include <DIO2.h>  // use IDE menu Tools..Manage Libraries to locate and  install DIO2
    #define WritePin digitalWrite2
    #define ReadPin digitalRead2
#else
    #define WritePin digitalWrite
    #define ReadPin digitalRead
#endif
    
MotorDriver::MotorDriver(byte power_pin, int signal_pin, int signal_pin2, int brake_pin, int current_pin, float sense_factor, int fault_pin) {
   powerPin=power_pin;
   signalPin=signal_pin;
   signalPin2=signal_pin2;
   brakePin=brake_pin;
   currentPin=current_pin;
   senseFactor=sense_factor;
   faultPin=fault_pin;
   I32=(int) (32000 / sensefactor);
   
  pinMode(powerPin, OUTPUT);
  pinMode(brakePin, OUTPUT);
  pinMode(signalPin, OUTPUT);
  if (signalPin2 != UNUSED_PIN) pinMode(signalPin2, OUTPUT);
  pinMode(currentPin, INPUT);
  if (faultPin != UNUSED_PIN) pinMode(faultPin, INPUT);
}

void MotorDriver::setPower(bool on) {
  WritePin(powerPin, on ? HIGH : LOW);
}
void MotorDriver::setBrake( bool on) {
  WritePin(brakePin, on ? HIGH : LOW);
}

void MotorDriver::setSignal( bool high) {
  WritePin(signalPin, high ? HIGH : LOW);
  if (signalPin2 != UNUSED_PIN) WritePin(signalPin2, high ? LOW : HIGH);
}


int MotorDriver::getCurrentRaw() {
  if (faultPin != UNUSED_PIN && ReadPin(faultPin) == LOW && ReadPin(powerPin) == HIGH)
      return I32:
  
  // IMPORTANT:  This function can be called in Interrupt() time within the 56uS timer
  //             The default analogRead takes ~100uS which is catastrphic
  //             so analogReadFast is used here. (-2uS) 
  return analogReadFast(sensePin);
}

unsigned int MortorDriver::convertRawToMilliamps( int raw) {
  return (unsigned int)(raw * senseFactor);
}
