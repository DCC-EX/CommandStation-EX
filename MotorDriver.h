/*
 *  © 2022-2023 Paul M. Antoine
 *  © 2021 Mike S
 *  © 2021 Fred Decker
 *  © 2020 Chris Harlow
 *  © 2022,2023 Harald Barth
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
#ifndef MotorDriver_h
#define MotorDriver_h
#include "FSH.h"
#include "IODevice.h"
#include "DCCTimer.h"

// use powers of two so we can do logical and/or on the track modes in if clauses.
// RACK_MODE_DCX is (TRACK_MODE_DC|TRACK_MODE_INV)
template<class T> inline T operator~ (T a) { return (T)~(int)a; }
template<class T> inline T operator| (T a, T b) { return (T)((int)a | (int)b); }
template<class T> inline T operator& (T a, T b) { return (T)((int)a & (int)b); }
template<class T> inline T operator^ (T a, T b) { return (T)((int)a ^ (int)b); }
enum TRACK_MODE : byte {TRACK_MODE_NONE = 1, TRACK_MODE_MAIN = 2, TRACK_MODE_PROG = 4,
                        TRACK_MODE_DC = 8, TRACK_MODE_EXT = 16, TRACK_MODE_BOOST = 32,
                        TRACK_MODE_ALL = 62, // only to operate all tracks
                        TRACK_MODE_INV = 64, TRACK_MODE_DCX = 72 /*DC + INV*/, TRACK_MODE_AUTOINV = 128};

#define setHIGH(fastpin)  *fastpin.inout |= fastpin.maskHIGH
#define setLOW(fastpin)   *fastpin.inout &= fastpin.maskLOW
#define isHIGH(fastpin)   (*fastpin.inout & fastpin.maskHIGH)
#define isLOW(fastpin)    (!isHIGH(fastpin))

#define TOKENPASTE(x, y) x ## y
#define TOKENPASTE2(x, y) TOKENPASTE(x, y)

#if defined(ARDUINO_AVR_MEGA) || defined(ARDUINO_AVR_MEGA2560)
#define HAVE_PORTA(X) X
#define HAVE_PORTB(X) X
#define HAVE_PORTC(X) X
#endif
#if defined(ARDUINO_AVR_UNO)
#define HAVE_PORTB(X) X
#endif
#if defined(ARDUINO_ARCH_SAMD)
#define PORTA REG_PORT_OUT0
#define HAVE_PORTA(X) X
#define PORTB REG_PORT_OUT1
#define HAVE_PORTB(X) X
#endif
#if defined(ARDUINO_ARCH_STM32)
#define PORTA GPIOA->ODR
#define HAVE_PORTA(X) X
#define PORTB GPIOB->ODR
#define HAVE_PORTB(X) X
#define PORTC GPIOC->ODR
#define HAVE_PORTC(X) X
#define PORTD GPIOD->ODR
#define HAVE_PORTD(X) X
#if defined(GPIOE)
#define PORTE GPIOE->ODR
#define HAVE_PORTE(X) X
#endif
#if defined(GPIOF)
#define PORTF GPIOF->ODR
#define HAVE_PORTF(X) X
#endif
#endif

// if macros not defined as pass-through we define
// them here as someting that is valid as a
// statement and evaluates to false.
#ifndef HAVE_PORTA
#define HAVE_PORTA(X) byte TOKENPASTE2(Unique_, __LINE__) __attribute__((unused)) =0
#endif
#ifndef HAVE_PORTB
#define HAVE_PORTB(X) byte TOKENPASTE2(Unique_, __LINE__) __attribute__((unused)) =0
#endif
#ifndef HAVE_PORTC
#define HAVE_PORTC(X) byte TOKENPASTE2(Unique_, __LINE__) __attribute__((unused)) =0
#endif
#ifndef HAVE_PORTD
#define HAVE_PORTD(X) byte TOKENPASTE2(Unique_, __LINE__) __attribute__((unused)) =0
#endif
#ifndef HAVE_PORTE
#define HAVE_PORTE(X) byte TOKENPASTE2(Unique_, __LINE__) __attribute__((unused)) =0
#endif
#ifndef HAVE_PORTF
#define HAVE_PORTF(X) byte TOKENPASTE2(Unique_, __LINE__) __attribute__((unused)) =0
#endif

// Virtualised Motor shield 1-track hardware Interface

#ifndef UNUSED_PIN     // sync define with the one in MotorDrivers.h
#define UNUSED_PIN 255 // inside uint8_t
#endif
#define MAX_PIN 254

class pinpair {
public:
  pinpair(byte p1, byte p2) {
    pin = p1;
    invpin = p2;
  };
  byte pin = UNUSED_PIN;
  byte invpin = UNUSED_PIN;
};

#if defined(__IMXRT1062__) || defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_SAMD) || defined(ARDUINO_ARCH_STM32)
typedef uint32_t portreg_t;
#else
typedef uint8_t portreg_t;
#endif

struct FASTPIN {
  volatile portreg_t *inout;
  portreg_t maskHIGH;
  portreg_t maskLOW;
  volatile portreg_t *shadowinout;
};
// The port registers that are shadowing
// the real port registers. These are
// defined in Motordriver.cpp
extern volatile portreg_t shadowPORTA;
extern volatile portreg_t shadowPORTB;
extern volatile portreg_t shadowPORTC;
extern volatile portreg_t shadowPORTD;
extern volatile portreg_t shadowPORTE;
extern volatile portreg_t shadowPORTF;

enum class POWERMODE : byte { OFF, ON, OVERLOAD, ALERT };

class MotorDriver {
  public:
    
    MotorDriver(int16_t power_pin, byte signal_pin, byte signal_pin2, int16_t brake_pin, 
                byte current_pin, float senseFactor, unsigned int tripMilliamps, int16_t fault_pin);
    void setPower( POWERMODE mode);
    POWERMODE getPower() { return powerMode;}
    // as the port registers can be shadowed to get syncronized DCC signals
    // we need to take care of that and we have to turn off interrupts if
    // we setSignal() or setBrake() or setPower() during that time as
    // otherwise the call from interrupt context can undo whatever we do
    // from outside interrupt
    void setBrake( bool on, bool interruptContext=false);
    __attribute__((always_inline)) inline void setSignal( bool high) {
#ifndef ARDUINO_ARCH_ESP32
      if (invertPhase)
	high = !high;
#endif
      if (trackPWM) {
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
    };
    inline void enableSignal(bool on) {
      if (on)
	pinMode(signalPin, OUTPUT);
      else
	pinMode(signalPin, INPUT);
      if (signalPin2 != UNUSED_PIN) {
	if (on)
	  pinMode(signalPin2, OUTPUT);
	else
	  pinMode(signalPin2, INPUT);
      }
    };
    inline pinpair getSignalPin() { return pinpair(signalPin,signalPin2); };
    void setDCSignal(byte speedByte);
    void throttleInrush(bool on);
    inline void detachDCSignal() {
#if defined(__arm__)
      pinMode(brakePin, OUTPUT);
#elif defined(ARDUINO_ARCH_ESP32)
      ledcDetachPin(brakePin);
#else
      setDCSignal(128);
#endif
    };
    int  getCurrentRaw(bool fromISR=false);
    unsigned int raw2mA( int raw);
    unsigned int mA2raw( unsigned int mA);
    inline bool brakeCanPWM() {
#if defined(ARDUINO_ARCH_ESP32)
      return (brakePin != UNUSED_PIN); // This was just (true) but we probably do need to check for UNUSED_PIN!
#elif defined(__arm__)
      // On ARM we can use digitalPinHasPWM
      return ((brakePin!=UNUSED_PIN) && (digitalPinHasPWM(brakePin)));
#elif defined(digitalPinToTimer)
      return ((brakePin!=UNUSED_PIN) && (digitalPinToTimer(brakePin)));
#else
      return (brakePin<14 && brakePin >1);
#endif
    }
    inline int getRawCurrentTripValue() {
	    return rawCurrentTripValue;
    }
    bool isPWMCapable();
    bool canMeasureCurrent();
    bool trackPWM = false; // this track uses PWM timer to generate the DCC waveform
    bool commonFaultPin = false; // This is a stupid motor shield which has only a common fault pin for both outputs
    inline byte setCommonFaultPin() {
      return commonFaultPin = true;
    }
    inline byte getFaultPin() {
	return faultPin;
    }
    inline void makeProgTrack(bool on) {  // let this output know it's a prog track.
      isProgTrack = on;
    }
    void checkPowerOverload(bool useProgLimit, byte trackno);
    inline void setTrackLetter(char c) {
      trackLetter = c;
    };
    // this returns how much time has passed since the last power change. If it
    // was really long ago (approx > 52min) advance counter approx 35 min so that
    // we are at 18 minutes again. Times for 32 bit unsigned long.
    inline unsigned long microsSinceLastPowerChange(POWERMODE mode) {
      unsigned long now = micros();
      unsigned long diff = now - lastPowerChange[(int)mode];
      if (diff > (1UL << (7 *sizeof(unsigned long)))) // 2^(4*7)us = 268.4 seconds
        lastPowerChange[(int)mode] = now - 30000000UL;           // 30 seconds ago
      return diff;
    };
#ifdef ANALOG_READ_INTERRUPT
    bool sampleCurrentFromHW();
    void startCurrentFromHW();
#endif
  inline void setMode(TRACK_MODE m) {
    trackMode = m;
    invertOutput(trackMode & TRACK_MODE_INV);
  };
  inline void invertOutput() {               // toggles output inversion
    invertPhase = !invertPhase;
    invertOutput(invertPhase);
  };
  inline void invertOutput(bool b) {         // sets output inverted or not
    if (b)
      invertPhase = 1;
    else
      invertPhase = 0;
#if defined(ARDUINO_ARCH_ESP32)
    pinpair p = getSignalPin();
    uint32_t *outreg = (uint32_t *)(GPIO_FUNC0_OUT_SEL_CFG_REG + 4*p.pin);
    if (invertPhase) // set or clear the invert bit in the gpio out register
      *outreg |=  ((uint32_t)0x1 << GPIO_FUNC0_OUT_INV_SEL_S);
    else
      *outreg &= ~((uint32_t)0x1 << GPIO_FUNC0_OUT_INV_SEL_S);
    if (p.invpin != UNUSED_PIN) {
      outreg = (uint32_t *)(GPIO_FUNC0_OUT_SEL_CFG_REG + 4*p.invpin);
      if (invertPhase) // clear or set the invert bit in the gpio out register
	*outreg &= ~((uint32_t)0x1 << GPIO_FUNC0_OUT_INV_SEL_S);
      else
	*outreg |=  ((uint32_t)0x1 << GPIO_FUNC0_OUT_INV_SEL_S);
    }
#endif
  };
  inline TRACK_MODE getMode() {
    return trackMode;
  };
  private:
    char trackLetter = '?';
    bool isProgTrack = false; // tells us if this is a prog track
    void  getFastPin(const FSH* type,int pin, bool input, FASTPIN & result);
    inline void  getFastPin(const FSH* type,int pin, FASTPIN & result) {
	getFastPin(type, pin, 0, result);
    };
    // side effect sets lastCurrent and tripValue
    inline bool checkCurrent(bool useProgLimit) {
      tripValue= useProgLimit?progTripValue:getRawCurrentTripValue();
      lastCurrent = getCurrentRaw();
      if (lastCurrent < 0)
	lastCurrent = -lastCurrent;
      return lastCurrent >= tripValue;
    };
    // side effect sets lastCurrent
    inline bool checkFault() {
      lastCurrent = getCurrentRaw();
      return lastCurrent < 0;
    };
    VPIN powerPin;
    byte signalPin, signalPin2, currentPin, faultPin, brakePin;
    FASTPIN fastSignalPin, fastSignalPin2, fastBrakePin,fastFaultPin;
    bool dualSignal;       // true to use signalPin2
    bool invertBrake;       // brake pin passed as negative means pin is inverted
    bool invertPower;       // power pin passed as negative means pin is inverted
    bool invertFault;       // fault pin passed as negative means pin is inverted
    bool invertPhase = 0;   // phase of out pin is inverted
    // Raw to milliamp conversion factors avoiding float data types.
    // Milliamps=rawADCreading * sensefactorInternal / senseScale
    //
    // senseScale is chosen as 256 to give enough scale for 2 decimal place 
    // raw->mA conversion with an ultra fast optimised integer multiplication  
    int senseFactorInternal;  // set to senseFactor * senseScale
    static const int senseScale=256;
    int senseOffset;
    unsigned int tripMilliamps;
    int rawCurrentTripValue;
    // current sampling
    POWERMODE powerMode;
    POWERMODE lastPowerMode;
    unsigned long lastPowerChange[4];         // timestamp in microseconds
    unsigned long lastBadSample;              // timestamp in microseconds
    // used to sync restore time when common Fault pin detected
    static unsigned long globalOverloadStart; // timestamp in microseconds
    int progTripValue;
    int  lastCurrent; //temp value
    int  tripValue;   //temp value
#ifdef ANALOG_READ_INTERRUPT
    volatile unsigned long sampleCurrentTimestamp;
    volatile uint16_t sampleCurrent;
#endif
    int maxmA;
    int tripmA;

    // Times for overload management. Unit: microseconds.
    // Base for wait time until power is turned on again
    static const unsigned long POWER_SAMPLE_OVERLOAD_WAIT =     40000UL;
    // Time after we consider all faults old and forgotten
    static const unsigned long POWER_SAMPLE_ALL_GOOD =        5000000UL;
    // Time after which we consider a ALERT over 
    static const unsigned long POWER_SAMPLE_ALERT_GOOD =        20000UL;
    // How long to ignore fault pin if current is under limit
    static const unsigned long POWER_SAMPLE_IGNORE_FAULT_LOW = 100000UL;
    // How long to ignore fault pin if current is higher than limit
    static const unsigned long POWER_SAMPLE_IGNORE_FAULT_HIGH =  5000UL;
    // How long to wait between overcurrent and turning off
    static const unsigned long POWER_SAMPLE_IGNORE_CURRENT  =  100000UL;
    // Upper limit for retry period
    static const unsigned long POWER_SAMPLE_RETRY_MAX =      10000000UL;
    
    // Trip current for programming track, 250mA. Change only if you really
    // need to be non-NMRA-compliant because of decoders that are not either.
    static const int TRIP_CURRENT_PROG=250;
    unsigned long power_sample_overload_wait = POWER_SAMPLE_OVERLOAD_WAIT;
    unsigned int power_good_counter = 0;
    TRACK_MODE trackMode = TRACK_MODE_NONE; // we assume track not assigned at startup

};
#endif
