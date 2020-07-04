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
#include <TimerOne.h>  // use IDE menu Tools..Manage Libraries to locate and  install TimerOne
#include "avdweb_AnalogReadFast.h"
#include "Hardware.h"
#include "Config.h"
#include "DIAG.h"


#if defined(ARDUINO_ARCH_AVR)
    #include <DIO2.h>  // use IDE menu Tools..Manage Libraries to locate and  install DIO2
    #define WritePin digitalWrite2
#else
    #define WritePin digitalWrite
#endif
    
void Hardware::init() {
  pinMode(MAIN_POWER_PIN, OUTPUT);
  pinMode(MAIN_SIGNAL_PIN, OUTPUT);
  if (MAIN_SIGNAL_PIN_ALT) pinMode(MAIN_SIGNAL_PIN_ALT, OUTPUT);
  pinMode(MAIN_SENSE_PIN, INPUT);

  pinMode(PROG_POWER_PIN, OUTPUT);
  pinMode(PROG_SIGNAL_PIN, OUTPUT);
  if (PROG_SIGNAL_PIN_ALT) pinMode(PROG_SIGNAL_PIN_ALT, OUTPUT);
  pinMode(PROG_SENSE_PIN, INPUT);
}

void Hardware::setPower(bool isMainTrack, bool on) {
  WritePin(isMainTrack ? MAIN_POWER_PIN : PROG_POWER_PIN, on ? HIGH : LOW);
}
void Hardware::setBrake(bool isMainTrack, bool on) {
  WritePin(isMainTrack ? MAIN_BRAKE_PIN : PROG_BRAKE_PIN, on ? LOW:HIGH);
}

void Hardware::setSignal(bool isMainTrack, bool high) {
  byte pin = isMainTrack ? MAIN_SIGNAL_PIN : PROG_SIGNAL_PIN;
  byte pin2 = isMainTrack ? MAIN_SIGNAL_PIN_ALT : PROG_SIGNAL_PIN_ALT;
  WritePin(pin, high ? HIGH : LOW);
  if (pin2) WritePin(pin2, high ? LOW : HIGH);
}

int Hardware::getCurrentRaw(bool isMainTrack) {
  // IMPORTANT:  This function can be called in Interrupt() time within the 56uS timer
  //             The default analogRead takes ~100uS which is catastrphic
  //             so analogReadFast is used here. (-2uS) 
  return analogReadFast(isMainTrack ? MAIN_SENSE_PIN : PROG_SENSE_PIN);
 
}

unsigned int Hardware::getCurrentMilliamps(bool isMainTrack, int raw) {
  return (int)(raw * (isMainTrack ? MAIN_SENSE_FACTOR : PROG_SENSE_FACTOR));
}

void Hardware::setCallback(int duration, void (*isr)()) {
  Timer1.initialize(duration);
  // We don't want the timer to set pins because these often clash with motor shields etc.
  Timer1.disablePwm(TIMER1_A_PIN);
  Timer1.disablePwm(TIMER1_B_PIN);
  Timer1.attachInterrupt(isr);
}


// Railcom support functions, not yet implemented
//void Hardware::setSingleCallback(int duration, void (*isr)()) {
//  Timer2.initialize(duration);
//  Timer2.disablePwm(TIMER1_A_PIN);
//  Timer2.disablePwm(TIMER1_B_PIN);
//  Timer2.attachInterrupt(isr);
//}

//void Hardware::resetSingleCallback(int duration) {
//  if (duration==0) Timer2.stop();
//  else Timer2.initialize(duration);
//}
