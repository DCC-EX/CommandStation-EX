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
#include "defines.h"
#include "FSH.h"

#if defined(ARDUINO_ARCH_ESP32)
#include <queue>
#include "DCCRMT.h"
#endif

// Number of preamble bits (moved here so MotorDriver and Waveform know)
const int   PREAMBLE_BITS_MAIN = 16;
const int   PREAMBLE_BITS_PROG = 22;

#ifndef UNUSED_PIN     // sync define with the one in MotorDrivers.h
#define UNUSED_PIN 127 // inside int8_t
#endif

#if defined(__IMXRT1062__) || defined(ESP_FAMILY)
typedef uint32_t PORTTYPE;
struct FASTPIN {
  volatile uint32_t *inout;
  uint32_t maskHIGH;  
  uint32_t maskLOW;  
};
#else
typedef uint8_t PORTTYPE;
struct FASTPIN {
  volatile uint8_t *inout;
  uint8_t maskHIGH;  
  uint8_t maskLOW;  
};
#endif

#define setHIGH(fastpin)  *fastpin.inout |= fastpin.maskHIGH
#define setLOW(fastpin)   *fastpin.inout &= fastpin.maskLOW
#define isHIGH(fastpin)   (*fastpin.inout & fastpin.maskHIGH)
#define isLOW(fastpin)    (!isHIGH(fastpin))

enum driverType { TIMERINTERRUPT, RMT_MAIN, RMT_PROG, DC_ENA, DC_BRAKE };
		  
class MotorDriver {
  public:
    MotorDriver(byte power_pin, byte signal_pin, byte signal_pin2, int8_t brake_pin, 
                byte current_pin, float senseFactor, unsigned int tripMilliamps, byte faultPin,
		driverType t=TIMERINTERRUPT);
    void setPower( bool on);
    void setSignal( bool high);
    void setBrake( bool on);
    int  getCurrentRaw();
    unsigned int raw2mA( int raw);
    int mA2raw( unsigned int mA);
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
#if defined(ARDUINO_ARCH_ESP32)
    void loop();
    inline driverType type() { return dtype; };
    bool schedulePacket(dccPacket packet);
#endif

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
    int senseOffset;
    unsigned int tripMilliamps;
    int rawCurrentTripValue;
#if defined(ARDUINO_TEENSY40) || defined(ARDUINO_TEENSY41)
    static bool disableInterrupts() {
      uint32_t primask;
      __asm__ volatile("mrs %0, primask\n" : "=r" (primask)::);
      __disable_irq();
      return (primask == 0) ? true : false;
    }
    static void enableInterrupts(bool doit) {
      if (doit) __enable_irq();
    }
#endif
#if defined(ARDUINO_ARCH_ESP32)
  RMTChannel* rmtChannel;
  std::queue<dccPacket> outQueue;
  driverType dtype;
#endif
};

class MotorDriverContainer {
public:
  MotorDriverContainer(const FSH * motorShieldName,
		       MotorDriver *m0=NULL,
		       MotorDriver *m1=NULL,
		       MotorDriver *m2=NULL,
		       MotorDriver *m3=NULL,
		       MotorDriver *m4=NULL,
		       MotorDriver *m5=NULL,
		       MotorDriver *m6=NULL,
		       MotorDriver *m7=NULL);
  inline void add(byte n, MotorDriver *m) {
    if (n>8) return;
    mD[n] = m;
  };
  //  void SetCapability(byte n, byte cap, char [] name);
  inline FSH *getMotorShieldName() { return shieldName; };
  inline MotorDriver *mainTrack() { return mD[0]; }; //start fixed
  inline MotorDriver *progTrack() { return mD[1]; };
  void loop();

private:
  MotorDriver *mD[8];
  FSH *shieldName;
};
#endif
