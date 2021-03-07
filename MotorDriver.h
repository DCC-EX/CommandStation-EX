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
#ifndef MotorDriver_h
#define MotorDriver_h
#include "FSH.h"

// Virtualised Motor shield 1-track hardware Interface

#ifndef UNUSED_PIN     // sync define with the one in MotorDrivers.h
#define UNUSED_PIN 127 // inside int8_t
#endif

struct FASTPIN {
  volatile uint8_t *inout;
  uint8_t maskHIGH;  
  uint8_t maskLOW;  
};

class MotorDriver {
  public:
    MotorDriver(byte power_pin, byte signal_pin, byte signal_pin2, int8_t brake_pin, byte current_pin, float senseFactor, unsigned int tripMilliamps, byte faultPin);
    virtual void setPower( bool on);
    virtual void setSignal( bool high);
    virtual void setBrake( bool on);
    virtual int  getCurrentRaw();
    virtual unsigned int raw2mA( int raw);
    virtual int mA2raw( unsigned int mA);
    inline int getRawCurrentTripValue() {
	    return rawCurrentTripValue;
    }
    bool isPWMCapable();
    static bool usePWM;
    static bool commonFaultPin; // This is a stupid motor shield which has only a common fault pin for both outputs
    inline byte getFaultPin() {
	return faultPin;
    }
    
  private:
    void  getFastPin(const FSH* type,int pin, bool input, FASTPIN & result);
    void  getFastPin(const FSH* type,int pin, FASTPIN & result) {
	getFastPin(type, pin, 0, result);
    }
    byte powerPin, signalPin, signalPin2, currentPin, faultPin, brakePin;
    FASTPIN fastPowerPin,fastSignalPin, fastSignalPin2, fastBrakePin,fastFaultPin;
    bool dualSignal;       // true to use signalPin2
    bool invertBrake;       // brake pin passed as negative means pin is inverted
    float senseFactor;
    unsigned int tripMilliamps;
    int rawCurrentTripValue;
};
#endif
