/*
 *  © 2022-2023 Paul M Antoine
 *  © 2021 Mike S
 *  © 2021 Fred Decker
 *  © 2020-2023 Harald Barth
 *  © 2020-2021 Chris Harlow
 *  © 2023 Colin Murdoch
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
#include "DCCWaveform.h"
#include "DCCTimer.h"
#include "DIAG.h"
#include "EXRAIL2.h"

unsigned long MotorDriver::globalOverloadStart = 0;

volatile portreg_t shadowPORTA;
volatile portreg_t shadowPORTB;
volatile portreg_t shadowPORTC;
#if defined(ARDUINO_ARCH_STM32)
volatile portreg_t shadowPORTD;
volatile portreg_t shadowPORTE;
volatile portreg_t shadowPORTF;
#endif

MotorDriver::MotorDriver(int16_t power_pin, byte signal_pin, byte signal_pin2, int16_t brake_pin,
                         byte current_pin, float sense_factor, unsigned int trip_milliamps, int16_t fault_pin) {
  const FSH * warnString = F("** WARNING **");

  invertPower=power_pin < 0;
  if (invertPower) {
    powerPin = 0-power_pin;
    IODevice::write(powerPin,HIGH);// set to OUTPUT and off
  } else {
    powerPin = power_pin;
    IODevice::write(powerPin,LOW);// set to OUTPUT and off
  }
  
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
  if (HAVE_PORTD(fastSignalPin.inout == &PORTD)) {
    DIAG(F("Found PORTD pin %d"),signalPin);
    fastSignalPin.shadowinout = fastSignalPin.inout;
    fastSignalPin.inout = &shadowPORTD;
  }
  if (HAVE_PORTE(fastSignalPin.inout == &PORTE)) {
    DIAG(F("Found PORTE pin %d"),signalPin);
    fastSignalPin.shadowinout = fastSignalPin.inout;
    fastSignalPin.inout = &shadowPORTE;
  }
  if (HAVE_PORTF(fastSignalPin.inout == &PORTF)) {
    DIAG(F("Found PORTF pin %d"),signalPin);
    fastSignalPin.shadowinout = fastSignalPin.inout;
    fastSignalPin.inout = &shadowPORTF;
  }

  signalPin2=signal_pin2;
  if (signalPin2!=UNUSED_PIN) {
    dualSignal=true;
    getFastPin(F("SIG2"),signalPin2,fastSignalPin2);
    pinMode(signalPin2, OUTPUT);

    fastSignalPin2.shadowinout = NULL;
    if (HAVE_PORTA(fastSignalPin2.inout == &PORTA)) {
      DIAG(F("Found PORTA pin %d"),signalPin2);
      fastSignalPin2.shadowinout = fastSignalPin2.inout;
      fastSignalPin2.inout = &shadowPORTA;
    }
    if (HAVE_PORTB(fastSignalPin2.inout == &PORTB)) {
      DIAG(F("Found PORTB pin %d"),signalPin2);
      fastSignalPin2.shadowinout = fastSignalPin2.inout;
      fastSignalPin2.inout = &shadowPORTB;
    }
    if (HAVE_PORTC(fastSignalPin2.inout == &PORTC)) {
      DIAG(F("Found PORTC pin %d"),signalPin2);
      fastSignalPin2.shadowinout = fastSignalPin2.inout;
      fastSignalPin2.inout = &shadowPORTC;
    }
    if (HAVE_PORTD(fastSignalPin2.inout == &PORTD)) {
      DIAG(F("Found PORTD pin %d"),signalPin2);
      fastSignalPin2.shadowinout = fastSignalPin2.inout;
      fastSignalPin2.inout = &shadowPORTD;
    }
    if (HAVE_PORTE(fastSignalPin2.inout == &PORTE)) {
      DIAG(F("Found PORTE pin %d"),signalPin2);
      fastSignalPin2.shadowinout = fastSignalPin2.inout;
      fastSignalPin2.inout = &shadowPORTE;
    }
    if (HAVE_PORTF(fastSignalPin2.inout == &PORTF)) {
      DIAG(F("Found PORTF pin %d"),signalPin2);
      fastSignalPin2.shadowinout = fastSignalPin2.inout;
      fastSignalPin2.inout = &shadowPORTF;
    }
  }
  else dualSignal=false; 
  
  if (brake_pin!=UNUSED_PIN){
    invertBrake=brake_pin < 0;
    if (invertBrake)
      brake_pin = 0-brake_pin;
    if (brake_pin > MAX_PIN)
      DIAG(F("%S Brake pin %d > %d"), warnString, brake_pin, MAX_PIN);
    brakePin=(byte)brake_pin;
    getFastPin(F("BRAKE"),brakePin,fastBrakePin);
    // if brake is used for railcom  cutout we need to do PORTX register trick here as well
    pinMode(brakePin, OUTPUT);
    setBrake(true);  // start with brake on in case we hace DC stuff going on
  } else {
    brakePin=UNUSED_PIN;
  }
  
  currentPin=current_pin;
  if (currentPin!=UNUSED_PIN) {
    int ret = ADCee::init(currentPin);
    if (ret < -1010) { // XXX give value a name later
      DIAG(F("ADCee::init error %d, disable current pin %d"), ret, currentPin);
      currentPin = UNUSED_PIN;
    }
  }
  senseOffset=0; // value can not be obtained until waveform is activated

  if (fault_pin != UNUSED_PIN) {
    invertFault=fault_pin < 0;
    if (invertFault)
      fault_pin =  0-fault_pin;
    if (fault_pin > MAX_PIN)
      DIAG(F("%S Fault pin %d > %d"), warnString, fault_pin, MAX_PIN);
    faultPin=(byte)fault_pin;
    DIAG(F("Fault pin = %d invert %d"), faultPin, invertFault);
    getFastPin(F("FAULT"),faultPin, 1 /*input*/, fastFaultPin);
    pinMode(faultPin, INPUT);
  } else {
      faultPin=UNUSED_PIN;
  }

  // This conversion performed at compile time so the remainder of the code never needs
  // float calculations or libraray code. 
  senseFactorInternal=sense_factor * senseScale; 
  tripMilliamps=trip_milliamps;
#ifdef MAX_CURRENT
  if (MAX_CURRENT > 0 && MAX_CURRENT < tripMilliamps)
    tripMilliamps = MAX_CURRENT;
#endif
  rawCurrentTripValue=mA2raw(tripMilliamps);

  if (rawCurrentTripValue + senseOffset > ADCee::ADCmax()) {
    // This would mean that the values obtained from the ADC never
    // can reach the trip value. So independent of the current, the
    // short circuit protection would never trip. So we adjust the
    // trip value so that it is tiggered when the ADC reports it's
    // maximum value instead.

    //    DIAG(F("Changing short detection value from %d to %d mA"),
    // raw2mA(rawCurrentTripValue), raw2mA(ADCee::ADCmax()-senseOffset));
    rawCurrentTripValue=ADCee::ADCmax()-senseOffset;
  }

  if (currentPin==UNUSED_PIN) 
    DIAG(F("%S No current or short detection"), warnString);
  else  {
    DIAG(F("Pin %d Max %dmA (%d)"), currentPin, raw2mA(rawCurrentTripValue), rawCurrentTripValue);

    // self testing diagnostic for the non-float converters... may be removed when happy
    //  DIAG(F("senseFactorInternal=%d raw2mA(1000)=%d mA2Raw(1000)=%d"),
    //   senseFactorInternal, raw2mA(1000),mA2raw(1000));
  }

  progTripValue = mA2raw(TRIP_CURRENT_PROG); 
}

bool MotorDriver::isPWMCapable() {
    return (!dualSignal) && DCCTimer::isPWMPin(signalPin);
}


void MotorDriver::setPower(POWERMODE mode) {
  if (powerMode == mode) return;
  //DIAG(F("Track %c POWERMODE=%d"), trackLetter, (int)mode);
  lastPowerChange[(int)mode] = micros();
  if (mode == POWERMODE::OVERLOAD)
    globalOverloadStart = lastPowerChange[(int)mode];
  bool on=(mode==POWERMODE::ON || mode ==POWERMODE::ALERT);
  if (on) {
    // when switching a track On, we need to check the crrentOffset with the pin OFF
    if (powerMode==POWERMODE::OFF && currentPin!=UNUSED_PIN) {
        senseOffset = ADCee::read(currentPin);
        DIAG(F("Track %c sensOffset=%d"),trackLetter,senseOffset);
    }

    IODevice::write(powerPin,invertPower ? LOW : HIGH);
    if (isProgTrack)
      DCCWaveform::progTrack.clearResets();
  }
  else {
      IODevice::write(powerPin,invertPower ? HIGH : LOW);
  }
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
void MotorDriver::setBrake(bool on, bool interruptContext) {
  if (brakePin == UNUSED_PIN) return;
  if (!interruptContext) {noInterrupts();}
  if (on ^ invertBrake)
    setHIGH(fastBrakePin);
  else
    setLOW(fastBrakePin);
  if (!interruptContext) {interrupts();}
}

bool MotorDriver::canMeasureCurrent() {
  return currentPin!=UNUSED_PIN;
}
/*
 * Return the current reading as pin reading 0 to max resolution (1024 or 4096).
 * If the fault pin is activated return a negative current to show active fault pin.
 * As there is no -0, cheat a little and return -1 in that case.
 * 
 * senseOffset handles the case where a shield returns values above or below 
 * a central value depending on direction.
 *
 * Bool fromISR should be adjusted dependent how function is called
 */
int MotorDriver::getCurrentRaw(bool fromISR) {
  (void)fromISR;
  if (currentPin==UNUSED_PIN) return 0; 
  int current;
  current = ADCee::read(currentPin, fromISR);
  // here one can diag raw value
  // if (fromISR == false) DIAG(F("%c: %d"), trackLetter, current);
  current = current-senseOffset;     // adjust with offset
  if (current<0) current=0-current;
  // current >= 0 here, we use negative current as fault pin flag
  if ((faultPin != UNUSED_PIN) && powerPin) {
    if (invertFault ? isHIGH(fastFaultPin) : isLOW(fastFaultPin))
      return (current == 0 ? -1 : -current);
  }
  return current;
}

#ifdef ANALOG_READ_INTERRUPT
/*
 * This should only be called in interrupt context
 * Copies current value from HW to cached value in
 * Motordriver.
 */
#pragma GCC push_options
#pragma GCC optimize ("-O3")
bool MotorDriver::sampleCurrentFromHW() {
  byte low, high;
  //if (!bit_is_set(ADCSRA, ADIF))
  if (bit_is_set(ADCSRA, ADSC))
    return false;
  //  if ((ADMUX & mask) != (currentPin - A0))
  //    return false;
  low = ADCL; //must read low before high
  high = ADCH;
  bitSet(ADCSRA, ADIF);
  sampleCurrent = (high << 8) | low;
  sampleCurrentTimestamp = millis();
  return true;
}
void MotorDriver::startCurrentFromHW() {
#if defined(ARDUINO_AVR_MEGA) || defined(ARDUINO_AVR_MEGA2560)
  const byte mask = 7;
#else
  const byte mask = 31;
#endif
  ADMUX=(1<<REFS0)|((currentPin-A0) & mask); //select AVCC as reference and set MUX
  bitSet(ADCSRA,ADSC); // start conversion
}
#pragma GCC pop_options
#endif //ANALOG_READ_INTERRUPT

#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_STM32)
#ifdef VARIABLE_TONES
uint16_t taurustones[28] = { 165, 175, 196, 220,
			     247, 262, 294, 330,
			     349, 392, 440, 494,
			     523, 587, 659, 698,
			     494, 440, 392, 249,
			     330, 284, 262, 247,
			     220, 196, 175, 165 };
#endif
#endif
void MotorDriver::setDCSignal(byte speedcode) {
  if (brakePin == UNUSED_PIN)
    return;
  switch(brakePin) {
#if defined(ARDUINO_AVR_UNO)
    // Not worth doin something here as:
    // If we are on pin 9 or 10 we are on Timer1 and we can not touch Timer1 as that is our DCC source.
    // If we are on pin 5 or 6 we are on Timer 0 ad we can not touch Timer0 as that is millis() etc.
    // We are most likely not on pin 3 or 11 as no known motor shield has that as brake.
#endif
#if defined(ARDUINO_AVR_MEGA) || defined(ARDUINO_AVR_MEGA2560)
  case 9:
  case 10:
    // Timer2 (is differnet)
    TCCR2A = (TCCR2A & B11111100) | B00000001; // set WGM1=0 and WGM0=1 phase correct PWM
    TCCR2B = (TCCR2B & B11110000) | B00000110; // set WGM2=0 ; set divisor on timer 2 to 1/256 for 122.55Hz
    //DIAG(F("2 A=%x B=%x"), TCCR2A, TCCR2B);
    break;
  case 6:
  case 7:
  case 8:
    // Timer4
    TCCR4A = (TCCR4A & B11111100) | B00000001; // set WGM0=1 and WGM1=0 for normal PWM 8-bit
    TCCR4B = (TCCR4B & B11100000) | B00000100; // set WGM2=0 and WGM3=0 for normal PWM 8 bit and div 1/256 for 122.55Hz
    break;
  case 46:
  case 45:
  case 44:
    // Timer5
    TCCR5A = (TCCR5A & B11111100) | B00000001; // set WGM0=1 and WGM1=0 for normal PWM 8-bit
    TCCR5B = (TCCR5B & B11100000) | B00000100; // set WGM2=0 and WGM3=0 for normal PWM 8 bit and div 1/256 for 122.55Hz
    break;
#endif
  default:
    break;
  }
  // spedcoode is a dcc speed & direction
  byte tSpeed=speedcode & 0x7F; // DCC Speed with 0,1 stop and speed steps 2 to 127
  byte tDir=speedcode & 0x80;
  byte brake;
#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_STM32)
  {
    int f = 131;
#ifdef VARIABLE_TONES
    if (tSpeed > 2) {
      if (tSpeed <= 58) {
	f = taurustones[ (tSpeed-2)/2 ] ;
      }
    }
#endif
    DCCTimer::DCCEXanalogWriteFrequency(brakePin, f); // set DC PWM frequency to 100Hz XXX May move to setup
  }
#endif
  if (tSpeed <= 1) brake = 255;
  else if (tSpeed >= 127) brake = 0;
  else  brake = 2 * (128-tSpeed);
  if (invertBrake)
    brake=255-brake;
#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_STM32)
  DCCTimer::DCCEXanalogWrite(brakePin,brake);
#else
  analogWrite(brakePin,brake);
#endif
  //DIAG(F("DCSignal %d"), speedcode);
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
  } else if (HAVE_PORTD(fastSignalPin.shadowinout == &PORTD)) {
    noInterrupts();
    HAVE_PORTD(shadowPORTD=PORTD);
    setSignal(tDir);
    HAVE_PORTD(PORTD=shadowPORTD);
    interrupts();
  } else if (HAVE_PORTE(fastSignalPin.shadowinout == &PORTE)) {
    noInterrupts();
    HAVE_PORTE(shadowPORTE=PORTE);
    setSignal(tDir);
    HAVE_PORTE(PORTE=shadowPORTE);
    interrupts();
  } else if (HAVE_PORTF(fastSignalPin.shadowinout == &PORTF)) {
    noInterrupts();
    HAVE_PORTF(shadowPORTF=PORTF);
    setSignal(tDir);
    HAVE_PORTF(PORTF=shadowPORTF);
    interrupts();
  } else {
    noInterrupts();
    setSignal(tDir);
    interrupts();
  }
}
void MotorDriver::throttleInrush(bool on) {
  if (brakePin == UNUSED_PIN)
    return;
  if ( !(trackMode & (TRACK_MODE_MAIN | TRACK_MODE_PROG | TRACK_MODE_EXT)))
    return;
  byte duty = on ? 208 : 0;
  if (invertBrake)
    duty = 255-duty;
#if defined(ARDUINO_ARCH_ESP32)
  if(on) {
    DCCTimer::DCCEXanalogWrite(brakePin,duty);
    DCCTimer::DCCEXanalogWriteFrequency(brakePin, 62500);
  } else {
    ledcDetachPin(brakePin);
  }
#elif defined(ARDUINO_ARCH_STM32)
  if(on) {
    DCCTimer::DCCEXanalogWriteFrequency(brakePin, 62500);
    DCCTimer::DCCEXanalogWrite(brakePin,duty);
  } else {
    pinMode(brakePin, OUTPUT);
  }
#else
  if(on){
    switch(brakePin) {
#if defined(ARDUINO_AVR_UNO)
      // Not worth doin something here as:
      // If we are on pin 9 or 10 we are on Timer1 and we can not touch Timer1 as that is our DCC source.
      // If we are on pin 5 or 6 we are on Timer 0 ad we can not touch Timer0 as that is millis() etc.
      // We are most likely not on pin 3 or 11 as no known motor shield has that as brake.
#endif
#if defined(ARDUINO_AVR_MEGA) || defined(ARDUINO_AVR_MEGA2560)
    case 9:
    case 10:
      // Timer2 (is different)
      TCCR2A = (TCCR2A & B11111100) | B00000011; // set WGM0=1 and WGM1=1 for fast PWM
      TCCR2B = (TCCR2B & B11110000) | B00000001; // set WGM2=0 and prescaler div=1 (max)
      DIAG(F("2 A=%x B=%x"), TCCR2A, TCCR2B);
      break;
    case 6:
    case 7:
    case 8:
      // Timer4
      TCCR4A = (TCCR4A & B11111100) | B00000001; // set WGM0=1 and WGM1=0 for fast PWM 8-bit
      TCCR4B = (TCCR4B & B11100000) | B00001001; // set WGM2=1 and WGM3=0 for fast PWM 8 bit and div=1 (max)
      break;
    case 46:
    case 45:
    case 44:
      // Timer5
      TCCR5A = (TCCR5A & B11111100) | B00000001; // set WGM0=1 and WGM1=0 for fast PWM 8-bit
      TCCR5B = (TCCR5B & B11100000) | B00001001; // set WGM2=1 and WGM3=0 for fast PWM 8 bit and div=1 (max)
      break;
#endif
    default:
      break;
    }
  }
  analogWrite(brakePin,duty);
#endif
}
unsigned int MotorDriver::raw2mA( int raw) {
  //DIAG(F("%d = %d * %d / %d"), (int32_t)raw * senseFactorInternal / senseScale, raw, senseFactorInternal, senseScale);
  return (int32_t)raw * senseFactorInternal / senseScale;
}
unsigned int MotorDriver::mA2raw( unsigned int mA) {
  //DIAG(F("%d = %d * %d / %d"), (int32_t)mA * senseScale / senseFactorInternal, mA, senseScale, senseFactorInternal);
  return (int32_t)mA * senseScale / senseFactorInternal;
}

void  MotorDriver::getFastPin(const FSH* type,int pin, bool input, FASTPIN & result) {
    // DIAG(F("MotorDriver %S Pin=%d,"),type,pin);
    (void) type; // avoid compiler warning if diag not used above.
#if defined(ARDUINO_ARCH_SAMD)
    PortGroup *port = digitalPinToPort(pin);
#elif defined(ARDUINO_ARCH_STM32)
    GPIO_TypeDef *port = digitalPinToPort(pin);
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

///////////////////////////////////////////////////////////////////////////////////////////
// checkPowerOverload(useProgLimit, trackno)
// bool useProgLimit: Trackmanager knows if this track is in prog mode or in main mode
// byte trackno: trackmanager knows it's number (could be skipped?)
//
// Short ciruit handling strategy:
//
// There are the following power states: ON ALERT OVERLOAD OFF
// OFF state is only changed to/from manually. Power is on
// during ON and ALERT. Power is off during OVERLOAD and OFF.
// The overload mechanism changes between the other states like
//
// ON -1-> ALERT -2-> OVERLOAD -3-> ALERT -4-> ON
// or
// ON -1-> ALERT -4-> ON
//
// Times are in class MotorDriver (MotorDriver.h).
//
// 1. ON to ALERT:
// Transition on fault pin condition or current overload
//
// 2. ALERT to OVERLOAD:
// Transition happens if different timeouts have elapsed.
// If only the fault pin is active, timeout is
// POWER_SAMPLE_IGNORE_FAULT_LOW (100ms)
// If only overcurrent is detected, timeout is
// POWER_SAMPLE_IGNORE_CURRENT (100ms)
// If fault pin and overcurrent are active, timeout is
// POWER_SAMPLE_IGNORE_FAULT_HIGH (5ms)
// Transition to OVERLOAD turns off power to the affected
// output (unless fault pins are shared)
// If the transition conditions are not fullfilled,
// transition according to 4 is tested.
//
// 3. OVERLOAD to ALERT
// Transiton happens when timeout has elapsed, timeout
// is named power_sample_overload_wait. It is started
// at POWER_SAMPLE_OVERLOAD_WAIT (40ms) at first entry
// to OVERLOAD and then increased by a factor of 2
// at further entries to the OVERLOAD condition. This
// happens until POWER_SAMPLE_RETRY_MAX (10sec) is reached.
// power_sample_overload_wait is reset by a poweroff or
// a POWER_SAMPLE_ALL_GOOD (5sec) period during ON.
// After timeout power is turned on again and state
// goes back to ALERT.
//
// 4. ALERT to ON
// Transition happens by watching the current and fault pin
// samples during POWER_SAMPLE_ALERT_GOOD (20ms) time. If
// values have been good during that time, transition is
// made back to ON. Note that even if state is back to ON,
// the power_sample_overload_wait time is first reset
// later (see above).
//
// The time keeping is handled by timestamps lastPowerChange[]
// which are set by each power change and by lastBadSample which
// keeps track if conditions during ALERT have been good enough
// to go back to ON. The time differences are calculated by
// microsSinceLastPowerChange().
//

void MotorDriver::checkPowerOverload(bool useProgLimit, byte trackno) {

  switch (powerMode) {

  case POWERMODE::OFF: {
    lastPowerMode = POWERMODE::OFF;
    power_sample_overload_wait = POWER_SAMPLE_OVERLOAD_WAIT;
    break;
  }

  case POWERMODE::ON: {
    lastPowerMode = POWERMODE::ON;
    bool cF = checkFault();
    bool cC = checkCurrent(useProgLimit);
    if(cF || cC ) {
      if (cC) {
	unsigned int mA=raw2mA(lastCurrent);
	DIAG(F("TRACK %c ALERT %s %dmA"), trackno + 'A',
	     cF ? "FAULT" : "",
	     mA);
      } else {
	DIAG(F("TRACK %c ALERT FAULT"), trackno + 'A');
      }
      setPower(POWERMODE::ALERT);
      if ((trackMode & TRACK_MODE_AUTOINV) && (trackMode & (TRACK_MODE_MAIN|TRACK_MODE_EXT|TRACK_MODE_BOOST))){
	DIAG(F("TRACK %c INVERT"), trackno + 'A');
	invertOutput();
      }
      break;
    }
    // all well
    if (microsSinceLastPowerChange(POWERMODE::ON) > POWER_SAMPLE_ALL_GOOD) {
      power_sample_overload_wait = POWER_SAMPLE_OVERLOAD_WAIT;
    }
    break;
  }

  case POWERMODE::ALERT: {
    // set local flags that handle how much is output to diag (do not output duplicates)
    bool notFromOverload = (lastPowerMode != POWERMODE::OVERLOAD);
    bool powerModeChange = (powerMode != lastPowerMode);
    unsigned long now = micros();
    if (powerModeChange)
      lastBadSample = now;
    lastPowerMode = POWERMODE::ALERT;
    // check how long we have been in this state
    unsigned long mslpc = microsSinceLastPowerChange(POWERMODE::ALERT);
    if(checkFault()) {
      throttleInrush(true);
      lastBadSample = now;
      unsigned long timeout = checkCurrent(useProgLimit) ? POWER_SAMPLE_IGNORE_FAULT_HIGH : POWER_SAMPLE_IGNORE_FAULT_LOW;
      if ( mslpc < timeout) {
	if (powerModeChange)
	  DIAG(F("TRACK %c FAULT PIN (%M ignore)"), trackno + 'A', timeout);
	break;
      }
      DIAG(F("TRACK %c FAULT PIN detected after %4M. Pause %4M)"), trackno + 'A', mslpc, power_sample_overload_wait);
      throttleInrush(false);
      setPower(POWERMODE::OVERLOAD);
      break;
    }
    if (checkCurrent(useProgLimit)) {
      lastBadSample = now;
      if (mslpc < POWER_SAMPLE_IGNORE_CURRENT) {
	if (powerModeChange) {
	  unsigned int mA=raw2mA(lastCurrent);
	  DIAG(F("TRACK %c CURRENT (%M ignore) %dmA"), trackno + 'A', POWER_SAMPLE_IGNORE_CURRENT, mA);
	}
	break;
      }
      unsigned int mA=raw2mA(lastCurrent);
      unsigned int maxmA=raw2mA(tripValue);
      DIAG(F("TRACK %c POWER OVERLOAD %4dmA (max %4dmA) detected after %4M. Pause %4M"),
	   trackno + 'A', mA, maxmA, mslpc, power_sample_overload_wait);
      throttleInrush(false);
      setPower(POWERMODE::OVERLOAD);
      break;
    }
    // all well
    unsigned long goodtime = micros() - lastBadSample;
    if (goodtime > POWER_SAMPLE_ALERT_GOOD) {
      if (true || notFromOverload) { // we did a RESTORE message XXX
	unsigned int mA=raw2mA(lastCurrent);
	DIAG(F("TRACK %c NORMAL (after %M/%M) %dmA"), trackno + 'A', goodtime, mslpc, mA);
      }
      throttleInrush(false);
      setPower(POWERMODE::ON);
    }
    break;
  }

  case POWERMODE::OVERLOAD: {
    lastPowerMode = POWERMODE::OVERLOAD;
    unsigned long mslpc = (commonFaultPin ? (micros() - globalOverloadStart) : microsSinceLastPowerChange(POWERMODE::OVERLOAD));
    if (mslpc > power_sample_overload_wait) {
      // adjust next wait time
      power_sample_overload_wait *= 2;
      if (power_sample_overload_wait > POWER_SAMPLE_RETRY_MAX)
	      power_sample_overload_wait = POWER_SAMPLE_RETRY_MAX;
  #ifdef EXRAIL_ACTIVE
      DIAG(F("Calling EXRAIL"));
      RMFT2::powerEvent(trackno, true); // Tell EXRAIL we have an overload
  #endif
      // power on test
      DIAG(F("TRACK %c POWER RESTORE (after %4M)"), trackno + 'A', mslpc);
      setPower(POWERMODE::ALERT);
    }
    break;
  }

  default:
    break;
  }
}
