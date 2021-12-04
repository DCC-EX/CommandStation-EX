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
#include "config.h"
#include "defines.h"
#include "MotorDriver.h"
#include "DCCTimer.h"
#include "DIAG.h"
#if defined(ARDUINO_ARCH_ESP32)
#include <driver/adc.h>
#define pinToADC1Channel(X) (adc1_channel_t)(((X) > 35) ? (X)-36 : (X)-28)
#endif

bool MotorDriver::usePWM=false;
bool MotorDriver::commonFaultPin=false;
       
MotorDriver::MotorDriver(byte power_pin, byte signal_pin, byte signal_pin2, int8_t brake_pin,
                         byte current_pin, float sense_factor, unsigned int trip_milliamps, byte fault_pin,
			 driverType dt) {
  dtype = dt;
  powerPin=power_pin;
  getFastPin(F("POWER"),powerPin,fastPowerPin);
  pinMode(powerPin, OUTPUT);

  if (dtype == RMT_MAIN) {
    signalPin=signal_pin;
#if defined(ARDUINO_ARCH_ESP32)
    rmtChannel = new RMTChannel(signalPin, true); // true: isMain
#endif
    dualSignal=false;
  } else if (dtype & (TIMER_MAIN | TIMER_PROG)) {
    signalPin=signal_pin;
    getFastPin(F("SIG"),signalPin,fastSignalPin);
    pinMode(signalPin, OUTPUT);

    signalPin2=signal_pin2;
    if (signalPin2!=UNUSED_PIN) {
      dualSignal=true;
      getFastPin(F("SIG2"),signalPin2,fastSignalPin2);
      pinMode(signalPin2, OUTPUT);
    }  else {
      dualSignal=false;
    }
  }
  
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
#if defined(ARDUINO_ARCH_ESP32)
    pinMode(currentPin, ANALOG);
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(pinToADC1Channel(currentPin),ADC_ATTEN_DB_11);
    senseOffset = adc1_get_raw(pinToADC1Channel(currentPin));
#else
    pinMode(currentPin, INPUT);
    senseOffset=analogRead(currentPin); // value of sensor at zero current
#endif
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
    DIAG(F("MotorDriver currentPin=A%d, senseOffset=%d, rawCurentTripValue(relative to offset)=%d"),
    currentPin-A0, senseOffset,rawCurrentTripValue);
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

void IRAM_ATTR MotorDriver::setSignal( bool high) {
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

#if defined(ARDUINO_TEENSY32) || defined(ARDUINO_TEENSY35)|| defined(ARDUINO_TEENSY36)
volatile unsigned int overflow_count=0;
#endif

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
#if defined(ARDUINO_TEENSY40) || defined(ARDUINO_TEENSY41)
  bool irq = disableInterrupts();
  current = analogRead(currentPin)-senseOffset;
  enableInterrupts(irq);
#elif defined(ARDUINO_TEENSY32) || defined(ARDUINO_TEENSY35)|| defined(ARDUINO_TEENSY36)
  unsigned char sreg_backup;
  sreg_backup = SREG;   /* save interrupt enable/disable state */
  cli();
  current = analogRead(currentPin)-senseOffset;
  overflow_count = 0;
  SREG = sreg_backup;    /* restore interrupt state */
#elif defined(ARDUINO_ARCH_ESP32)
  current = adc1_get_raw(pinToADC1Channel(currentPin))-senseOffset;
#else
  current = analogRead(currentPin)-senseOffset;
#endif
  if (current<0) current=0-current;
  if ((faultPin != UNUSED_PIN)  && isLOW(fastFaultPin) && isHIGH(fastPowerPin))
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
    // DIAG(F("MotorDriver %S Pin=%d,"),type,pin);
    (void) type; // avoid compiler warning if diag not used above.
    PORTTYPE port = digitalPinToPort(pin);
    if (input)
      result.inout = portInputRegister(port);
    else
      result.inout = portOutputRegister(port);
    result.maskHIGH = digitalPinToBitMask(pin);
    result.maskLOW = ~result.maskHIGH;
    // DIAG(F(" port=0x%x, inoutpin=0x%x, isinput=%d, mask=0x%x"),port, result.inout,input,result.maskHIGH);
}

bool MotorDriver::schedulePacket(dccPacket packet) {
  if(!rmtChannel) return true; // fake success if functionality is not there

  outQueue.push(packet);
  uint16_t size = outQueue.size();
  if (size > 10) {
    DIAG(F("Warning: outQueue %d > 10"),size);
  }
  return true;
}

void MotorDriver::loop() {
  if (rmtChannel  && !outQueue.empty() && rmtChannel->RMTfillData(outQueue.front()))
    outQueue.pop();
}

MotorDriverContainer::MotorDriverContainer(const FSH * motorShieldName,
					   MotorDriver *m0,
					   MotorDriver *m1,
					   MotorDriver *m2,
					   MotorDriver *m3,
					   MotorDriver *m4,
					   MotorDriver *m5,
					   MotorDriver *m6,
					   MotorDriver *m7) {
  // THIS AUTOMATIC DOES NOT WORK YET. TIMER_MAIN AND TIMER_PROG required in CONSTRUCTOR
  // AND CAN NOT BE ADDED LATER
  if (m0) {
    if (m0->type() == TYPE_UNKNOWN)
      m0->setType(TIMER_MAIN);
    mD.push_back(m0);
  }
  if (m1) {
    if (m1->type() == TYPE_UNKNOWN)
      m1->setType(TIMER_PROG);
    mD.push_back(m1);
  }
  if (m2) mD.push_back(m2);
  if (m3) mD.push_back(m3);
  if (m4) mD.push_back(m4);
  if (m5) mD.push_back(m5);
  if (m6) mD.push_back(m6);
  if (m7) mD.push_back(m7);
  shieldName = (FSH *)motorShieldName;
}

void MotorDriverContainer::loop() {
  // loops over MotorDrivers which have loop tasks
  if (mD.empty())
    return;
  for(const auto& d: mD)
    if (d->type() & (RMT_MAIN | RMT_PROG))
      d->loop();
}

std::vector<MotorDriver*> MotorDriverContainer::getDriverType(driverType t) {
  std::vector<MotorDriver*> v;
  for(const auto& d: mD){
    if (d->type() & t)
      v.push_back(d);
  }
  return v;
}

MotorDriverContainer MotorDriverContainer::mDC(MOTOR_SHIELD_TYPE);
