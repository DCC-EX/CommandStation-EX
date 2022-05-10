/*
 *  © 2021 Mike S
 *  © 2021 Fred Decker
 *  © 2020 Chris Harlow
 *  All rights reserved.
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
#ifndef MotorDriver_h
#define MotorDriver_h
#include "FSH.h"
#include "IODevice.h"
#include "DCCTimer.h"

#define setHIGH(fastpin)  *fastpin.inout |= fastpin.maskHIGH
#define setLOW(fastpin)   *fastpin.inout &= fastpin.maskLOW
#define isHIGH(fastpin)   (*fastpin.inout & fastpin.maskHIGH)
#define isLOW(fastpin)    (!isHIGH(fastpin))

// Virtualised Motor shield 1-track hardware Interface

#ifndef UNUSED_PIN     // sync define with the one in MotorDrivers.h
#define UNUSED_PIN 127 // inside int8_t
#endif

#if defined(__IMXRT1062__) || defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ARCH_ESP32)
struct FASTPIN {
  volatile uint32_t *inout;
  uint32_t maskHIGH;  
  uint32_t maskLOW;  
};
#else
struct FASTPIN {
  volatile uint8_t *inout;
  uint8_t maskHIGH;  
  uint8_t maskLOW;  
};
#endif

enum class POWERMODE : byte { OFF, ON, OVERLOAD };

class MotorDriver {
  public:
    
    MotorDriver(VPIN power_pin, byte signal_pin, byte signal_pin2, int8_t brake_pin, 
                byte current_pin, float senseFactor, unsigned int tripMilliamps, byte faultPin);
    virtual void setPower( POWERMODE mode);
    virtual POWERMODE getPower() { return powerMode;}
    __attribute__((always_inline)) inline void setSignal( bool high) {
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
    };
    virtual void setBrake( bool on);
    virtual void setDCSignal(byte speedByte);
    virtual int  getCurrentRaw();
    virtual int getCurrentRawInInterrupt();
    virtual unsigned int raw2mA( int raw);
    virtual int mA2raw( unsigned int mA);
    inline bool canBrake() {
      return brakePin!=UNUSED_PIN;
    }
    inline int getRawCurrentTripValue() {
	    return rawCurrentTripValue;
    }
    bool isPWMCapable();
    bool canMeasureCurrent();
    static bool usePWM;
    static bool commonFaultPin; // This is a stupid motor shield which has only a common fault pin for both outputs
    inline byte getFaultPin() {
	return faultPin;
    }
    inline void setResetCounterPointer(byte *bp) {  // load resetPacketCounter pointer
      resetsCounterP = bp;
    }
    void checkPowerOverload(bool useProgLimit, byte trackno);
  private:
    byte *resetsCounterP = NULL; // points to the resetPacketCounter if this is a prog track
    void  getFastPin(const FSH* type,int pin, bool input, FASTPIN & result);
    void  getFastPin(const FSH* type,int pin, FASTPIN & result) {
	getFastPin(type, pin, 0, result);
    }
    VPIN powerPin;
    byte signalPin, signalPin2, currentPin, faultPin, brakePin;
    FASTPIN fastSignalPin, fastSignalPin2, fastBrakePin,fastFaultPin;
    bool dualSignal;       // true to use signalPin2
    bool invertBrake;       // brake pin passed as negative means pin is inverted
    float senseFactor;
    int senseOffset;
    unsigned int tripMilliamps;
    int rawCurrentTripValue;
    // current sampling
    POWERMODE powerMode;
    unsigned long lastSampleTaken;
    unsigned int sampleDelay;
    int progTripValue;
    int  lastCurrent;
    int maxmA;
    int tripmA;

    // Wait times for power management. Unit: milliseconds
    static const int  POWER_SAMPLE_ON_WAIT = 100;
    static const int  POWER_SAMPLE_OFF_WAIT = 1000;
    static const int  POWER_SAMPLE_OVERLOAD_WAIT = 20;
    
    // Trip current for programming track, 250mA. Change only if you really
    // need to be non-NMRA-compliant because of decoders that are not either.
    static const int TRIP_CURRENT_PROG=250;
    unsigned long power_sample_overload_wait = POWER_SAMPLE_OVERLOAD_WAIT;
    unsigned int power_good_counter = 0;

};
#endif
