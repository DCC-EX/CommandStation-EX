/*
 *  © 2023, Neil McKechnie. All rights reserved.
 *  
 *  This file is part of DCC++EX API
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

/*
 * This device is a layered device which is designed to sit on top of another
 * device.  The underlying device class is expected to accept writeAnalogue calls 
 * which will normally cause some physical movement of something.  The device may be a servo, 
 * a motor or some other kind of positioner, and the something might be a turnout,
 * a semaphore signal or something else.  One user has used this capability for
 * moving a figure along the platform on their layout!
 * 
 * Example of use:
 *    In myHal.cpp, 
 * 
 * #include "IO_Servo.h"
 * ...
 * PCA9685::create(100,16,0x40);   // First create the hardware interface device
 * Servo::create(300,16,100);    // Then create the higher level device which 
 *                               // references pins 100-115 or a subset of them.
 * 
 * Then any reference to pins 300-315 will cause the servo driver to send output
 * PWM commands to the corresponding PCA9685 driver pins 100-115.  The PCA9685 driver may
 * be substituted with any other driver which provides analogue output 
 * capability, e.g. EX-IOExpander devices, as long as they are capable of interpreting 
 * the writeAnalogue() function calls.
 */

#include "IODevice.h"

#ifndef IO_SERVO_H
#define IO_SERVO_H

#include "I2CManager.h"
#include "DIAG.h"

class Servo : IODevice {

public: 
  enum ProfileType : uint8_t {
    Instant = 0,  // Moves immediately between positions (if duration not specified)
    UseDuration = 0, // Use specified duration
    Fast = 1,     // Takes around 500ms end-to-end
    Medium = 2,   // 1 second end-to-end
    Slow = 3,     // 2 seconds end-to-end
    Bounce = 4,   // For semaphores/turnouts with a bit of bounce!!
    NoPowerOff = 0x80, // Flag to be ORed in to suppress power off after move.
  };

  // Create device driver instance.
  static void create(VPIN firstVpin, int nPins, VPIN firstSlavePin=VPIN_NONE) {
    new Servo(firstVpin, nPins, firstSlavePin);
  }

private:
  VPIN _firstSlavePin;
  IODevice *_slaveDevice = NULL;

  struct ServoData {
    uint16_t activePosition : 12; // Config parameter
    uint16_t inactivePosition : 12; // Config parameter
    uint16_t currentPosition : 12;
    uint16_t fromPosition : 12;
    uint16_t toPosition : 12; 
    uint8_t profile;  // Config parameter
    uint16_t stepNumber; // Index of current step (starting from 0)
    uint16_t numSteps;  // Number of steps in animation, or 0 if none in progress.
    uint8_t currentProfile; // profile being used for current animation.
    uint16_t duration; // time (tenths of a second) for animation to complete.
  }; // 14 bytes per element, i.e. per pin in use
  
  struct ServoData *_servoData [16];

  static const uint8_t _catchupSteps = 5; // number of steps to wait before switching servo off
  static const uint8_t FLASH _bounceProfile[30];

  const unsigned int refreshInterval = 50; // refresh every 50ms


  // Configure a port on the Servo.
  bool _configure(VPIN vpin, ConfigTypeEnum configType, int paramCount, int params[]) {
    if (_deviceState == DEVSTATE_FAILED) return false;
    if (configType != CONFIGURE_SERVO) return false;
    if (paramCount != 5) return false;
    #ifdef DIAG_IO
    DIAG(F("Servo: Configure VPIN:%u Apos:%d Ipos:%d Profile:%d Duration:%d state:%d"), 
      vpin, params[0], params[1], params[2], params[3], params[4]);
    #endif

    int8_t pin = vpin - _firstVpin;
    struct ServoData *s = _servoData[pin];
    if (s == NULL) { 
      _servoData[pin] = (struct ServoData *)calloc(1, sizeof(struct ServoData));
      s = _servoData[pin];
      if (!s) return false; // Check for failed memory allocation
    }

    s->activePosition = params[0];
    s->inactivePosition = params[1];
    s->profile = params[2];
    s->duration = params[3];
    int state = params[4];

    if (state != -1) {
      // Position servo to initial state
      writeAnalogue(vpin, state ? s->activePosition : s->inactivePosition);
    } 
    return true;
  }

  // Constructor
  Servo(VPIN firstVpin, int nPins, VPIN firstSlavePin = VPIN_NONE) {
    _firstVpin = firstVpin;
    _nPins = (nPins > 16) ? 16 : nPins;
    if (firstSlavePin == VPIN_NONE)
      _firstSlavePin = firstVpin;
    else
      _firstSlavePin = firstSlavePin;

    // To save RAM, space for servo configuration is not allocated unless a pin is used.
    // Initialise the pointers to NULL.
    for (int i=0; i<_nPins; i++)
      _servoData[i] = NULL;

    // Get reference to slave device.
    _slaveDevice = findDevice(_firstSlavePin);
    if (!_slaveDevice) {
      DIAG(F("Servo: Slave device not found on Vpins %u-%u"), 
        _firstSlavePin, _firstSlavePin+_nPins-1);
      _deviceState = DEVSTATE_FAILED;
    }      
    if (_slaveDevice != findDevice(_firstSlavePin+_nPins-1)) {
      DIAG(F("Servo: Slave device does not cover all Vpins %u-%u"), 
        _firstSlavePin, _firstSlavePin+_nPins-1);
      _deviceState = DEVSTATE_FAILED;
    }

    addDevice(this, _slaveDevice); // Link device ahead of slave device to intercept requests
  }

  // Device-specific initialisation
  void _begin() override {
    #if defined(DIAG_IO)
    _display();
    #endif
  }

  // Device-specific write function, invoked from IODevice::write().  
  // For this function, the configured profile is used.
  void _write(VPIN vpin, int value) override {
    if (_deviceState == DEVSTATE_FAILED) return;
    #ifdef DIAG_IO
    DIAG(F("Servo Write VPIN:%u Value:%d"), vpin, value);
    #endif
    int pin = vpin - _firstVpin;
    if (value) value = 1;

    struct ServoData *s = _servoData[pin];
    if (s != NULL) {
      // Use configured parameters
      writeAnalogue(vpin, value ? s->activePosition : s->inactivePosition, s->profile, s->duration);
    }  else {
      /* simulate digital pin on PWM */
      writeAnalogue(vpin, value ? 4095 : 0, Instant | NoPowerOff, 0);     
    }
  }

  // Device-specific writeAnalogue function, invoked from IODevice::writeAnalogue().
  // Profile is as follows:
  //  Bit 7:     0=Set output to 0% to power off servo motor when finished
  //             1=Keep output at final position (better with LEDs, which will stay lit)
  //  Bits 6-0:  0           Use specified duration (defaults to 0 deciseconds)
  //             1 (Fast)    Move servo in 0.5 seconds
  //             2 (Medium)  Move servo in 1.0 seconds
  //             3 (Slow)    Move servo in 2.0 seconds
  //             4 (Bounce)  Servo 'bounces' at extremes.
  // Duration is in deciseconds (tenths of a second) and defaults to 0.  
  //            
  void _writeAnalogue(VPIN vpin, int value, uint8_t profile, uint16_t duration) override {
    #ifdef DIAG_IO
    DIAG(F("Servo: WriteAnalogue VPIN:%u Value:%d Profile:%d Duration:%d %S"), 
      vpin, value, profile, duration, _deviceState == DEVSTATE_FAILED?F("DEVSTATE_FAILED"):F(""));
    #endif
    if (_deviceState == DEVSTATE_FAILED) return;
    int pin = vpin - _firstVpin;
    if (value > 4095) value = 4095;
    else if (value < 0) value = 0;

    struct ServoData *s = _servoData[pin];
    if (s == NULL) {
      // Servo pin not configured, so configure now using defaults
      s = _servoData[pin] = (struct ServoData *) calloc(sizeof(struct ServoData), 1);
      if (s == NULL) return;  // Check for memory allocation failure
      s->activePosition = 4095;
      s->inactivePosition = 0;
      s->currentPosition = value;
      s->profile = Instant | NoPowerOff;  // Use instant profile (but not this time)
    }

    // Animated profile.  Initiate the appropriate action.
    s->currentProfile = profile;
    uint8_t profileValue = profile & ~NoPowerOff;  // Mask off 'don't-power-off' bit.
    s->numSteps = profileValue==Fast ? 10 :   // 0.5 seconds
                  profileValue==Medium ? 20 : // 1.0 seconds
                  profileValue==Slow ? 40 :   // 2.0 seconds
                  profileValue==Bounce ? sizeof(_bounceProfile)-1 : // ~ 1.5 seconds
                  duration * 2 + 1; // Convert from deciseconds (100ms) to refresh cycles (50ms)
    s->stepNumber = 0;
    s->toPosition = value;
    s->fromPosition = s->currentPosition;
  }

  // _read returns true if the device is currently in executing an animation, 
  //  changing the output over a period of time.
  int _read(VPIN vpin) override {
    if (_deviceState == DEVSTATE_FAILED) return 0;
    int pin = vpin - _firstVpin;
    struct ServoData *s = _servoData[pin];
    if (s == NULL) 
      return false; // No structure means no animation!
    else
      return (s->stepNumber < s->numSteps);
  }

  void _loop(unsigned long currentMicros) override {
    if (_deviceState == DEVSTATE_FAILED) return;
    for (int pin=0; pin<_nPins; pin++) {
      updatePosition(pin);
    }
    delayUntil(currentMicros + refreshInterval * 1000UL);
  }

  // Private function to reposition servo
  // TODO: Could calculate step number from elapsed time, to allow for erratic loop timing.
  void updatePosition(uint8_t pin) {
    struct ServoData *s = _servoData[pin];
    if (s == NULL) return; // No pin configuration/state data

    if (s->numSteps == 0) return; // No animation in progress

    if (s->stepNumber == 0 && s->fromPosition == s->toPosition) {
      // Go straight to end of sequence, output final position.
      s->stepNumber = s->numSteps-1;
    }

    if (s->stepNumber < s->numSteps) {
      // Animation in progress, reposition servo
      s->stepNumber++;
      if ((s->currentProfile & ~NoPowerOff) == Bounce) {
        // Retrieve step positions from array in flash
        uint8_t profileValue = GETFLASH(&_bounceProfile[s->stepNumber]);
        s->currentPosition = map(profileValue, 0, 100, s->fromPosition, s->toPosition);
      } else {
        // All other profiles - calculate step by linear interpolation between from and to positions.
        s->currentPosition = map(s->stepNumber, 0, s->numSteps, s->fromPosition, s->toPosition);
      }
      // Send servo command to output driver
      _slaveDevice->_writeAnalogue(_firstSlavePin+pin, s->currentPosition);
    } else if (s->stepNumber < s->numSteps + _catchupSteps) {
      // We've finished animation, wait a little to allow servo to catch up
      s->stepNumber++;
    } else if (s->stepNumber == s->numSteps + _catchupSteps 
              && s->currentPosition != 0) {
  #ifdef IO_SWITCH_OFF_SERVO
      if ((s->currentProfile & NoPowerOff) == 0) {
        // Wait has finished, so switch off output driver to avoid servo buzz.
        _slaveDevice->_writeAnalogue(_firstSlavePin+pin, 0);
      }
  #endif
      s->numSteps = 0;  // Done now.
    }
  }

  // Display details of this device.
  void _display() override {
    DIAG(F("Servo Configured on Vpins:%u-%u, slave pins:%d-%d %S"),
      (int)_firstVpin, (int)_firstVpin+_nPins-1,
      (int)_firstSlavePin, (int)_firstSlavePin+_nPins-1,
      (_deviceState==DEVSTATE_FAILED) ? F("OFFLINE") : F(""));
  }
};

#endif