/*
 *  © 2022, Peter Cole. All rights reserved.
 *
 *  This file is part of EX-CommandStation
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
* The IO_EXIOExpander.h device driver integrates with one or more EX-IOExpander devices.
* This device driver will configure the device on startup, along with
* interacting with the device for all input/output duties.
*
* To create EX-IOExpander devices, these are defined in myHal.cpp:
* (Note the device driver is included by default)
*
* void halSetup() {
*   // EXIOExpander::create(vpin, num_vpins, i2c_address);
*   EXIOExpander::create(800, 18, 0x65);
* }
* 
* All pins on an EX-IOExpander device are allocated according to the pin map for the specific
* device in use. There is no way for the device driver to sanity check pins are used for the
* correct purpose, however the EX-IOExpander device's pin map will prevent pins being used
* incorrectly (eg. A6/7 on Nano cannot be used for digital input/output).
*/

#ifndef IO_EX_IOEXPANDER_H
#define IO_EX_IOEXPANDER_H

#include "I2CManager.h"
#include "DIAG.h"
#include "FSH.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 * IODevice subclass for EX-IOExpander.
 */
class EXIOExpander : public IODevice {
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

  static void create(VPIN vpin, int nPins, uint8_t i2cAddress) {
    if (checkNoOverlap(vpin, nPins, i2cAddress)) new EXIOExpander(vpin, nPins, i2cAddress);
  }

private:  
  // Constructor
  EXIOExpander(VPIN firstVpin, int nPins, uint8_t i2cAddress) {
    _firstVpin = firstVpin;
    _nPins = nPins;
    _i2cAddress = i2cAddress;
    // To save RAM, space for servo configuration is not allocated unless a pin is used.
    // Initialise the pointers to NULL.
    for (int i=0; i<_nPins; i++) {
      _servoData[i] = NULL;
    }
    addDevice(this);
  }

  void _begin() {
    // Initialise EX-IOExander device
    I2CManager.begin();
    if (I2CManager.exists(_i2cAddress)) {
      _command4Buffer[0] = EXIOINIT;
      _command4Buffer[1] = _nPins;
      _command4Buffer[2] = _firstVpin & 0xFF;
      _command4Buffer[3] = _firstVpin >> 8;
      // Send config, if EXIOPINS returned, we're good, setup pin buffers, otherwise go offline
      I2CManager.read(_i2cAddress, _receive3Buffer, 3, _command4Buffer, 4);
      if (_receive3Buffer[0] == EXIOPINS) {
        _numDigitalPins = _receive3Buffer[1];
        _numAnaloguePins = _receive3Buffer[2];
        _digitalPinBytes = (_numDigitalPins + 7)/8;
        _digitalInputStates=(byte*) calloc(_digitalPinBytes,1);
        _analoguePinBytes = _numAnaloguePins * 2;
        _analogueInputStates = (byte*) calloc(_analoguePinBytes, 1);
        _analoguePinMap = (uint8_t*) calloc(_numAnaloguePins, 1);
      } else {
        DIAG(F("ERROR configuring EX-IOExpander device, I2C:x%x"), _i2cAddress);
        _deviceState = DEVSTATE_FAILED;
        return;
      }
      // We now need to retrieve the analogue pin map
      _command1Buffer[0] = EXIOINITA;
      I2CManager.read(_i2cAddress, _analoguePinMap, _numAnaloguePins, _command1Buffer, 1);
      // Attempt to get version, if we don't get it, we don't care, don't go offline
      _command1Buffer[0] = EXIOVER;
      I2CManager.read(_i2cAddress, _versionBuffer, 3, _command1Buffer, 1);
      _command1Buffer[0] = EXIOVER;
      I2CManager.read(_i2cAddress, _versionBuffer, 3, _command1Buffer, 1);
      _majorVer = _versionBuffer[0];
      _minorVer = _versionBuffer[1];
      _patchVer = _versionBuffer[2];
      DIAG(F("EX-IOExpander device found, I2C:x%x, Version v%d.%d.%d"),
          _i2cAddress, _versionBuffer[0], _versionBuffer[1], _versionBuffer[2]);
#ifdef DIAG_IO
      _display();
#endif
    } else {
      DIAG(F("EX-IOExpander device not found, I2C:x%x"), _i2cAddress);
      _deviceState = DEVSTATE_FAILED;
    }
  }

  // Digital input pin configuration, used to enable on EX-IOExpander device and set pullups if in use
  bool _configure(VPIN vpin, ConfigTypeEnum configType, int paramCount, int params[]) override {
    if (paramCount != 1) return false;
    int pin = vpin - _firstVpin;
    if (configType == CONFIGURE_INPUT) {
      bool pullup = params[0];
      _digitalOutBuffer[0] = EXIODPUP;
      _digitalOutBuffer[1] = pin;
      _digitalOutBuffer[2] = pullup;
      I2CManager.read(_i2cAddress, _command1Buffer, 1, _digitalOutBuffer, 3);
      if (_command1Buffer[0] == EXIORDY) {
        return true;
      } else {
        DIAG(F("Vpin %d cannot be used as a digital input pin"), (int)vpin);
        return false;
      }
    } else if (configType == CONFIGURE_SERVO) {
      if (paramCount != 5) return false;
#ifdef DIAG_IO
      DIAG(F("Servo: Configure VPIN:%d Apos:%d Ipos:%d Profile:%d Duration:%d state:%d"), 
        vpin, params[0], params[1], params[2], params[3], params[4]);
#endif
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
        IODevice::writeAnalogue(pin, state ? s->activePosition : s->inactivePosition, 0, 0);
      } 
      return true;
    } else {
      return false;
    }
  }

  // Analogue input pin configuration, used to enable on EX-IOExpander device
  int _configureAnalogIn(VPIN vpin) override {
    int pin = vpin - _firstVpin;
    _command2Buffer[0] = EXIOENAN;
    _command2Buffer[1] = pin;
    I2CManager.read(_i2cAddress, _command1Buffer, 1, _command2Buffer, 2);
    if (_command1Buffer[0] == EXIORDY) {
      return true;
    } else {
      DIAG(F("Vpin %d cannot be used as an analogue input pin"), (int)vpin);
      return false;
    }
    return true;
  }

  // Main loop, collect both digital and analogue pin states continuously (faster sensor/input reads)
  void _loop(unsigned long currentMicros) override {
    (void)currentMicros; // remove warning
    if (_deviceState == DEVSTATE_FAILED) return;
    _command1Buffer[0] = EXIORDD;
    I2CManager.read(_i2cAddress, _digitalInputStates, _digitalPinBytes, _command1Buffer, 1);
    _command1Buffer[0] = EXIORDAN;
    I2CManager.read(_i2cAddress, _analogueInputStates, _analoguePinBytes, _command1Buffer, 1);
    for (int pin=0; pin<_nPins; pin++) {
      updatePosition(pin);
    }
  }

  // Obtain the correct analogue input value
  int _readAnalogue(VPIN vpin) override {
    int pin = vpin - _firstVpin;
    uint8_t _pinLSBByte;
    for (uint8_t aPin = 0; aPin < _numAnaloguePins; aPin++) {
      if (_analoguePinMap[aPin] == pin) {
        _pinLSBByte = aPin * 2;
      }
    }
    uint8_t _pinMSBByte = _pinLSBByte + 1;
    return (_analogueInputStates[_pinMSBByte] << 8) + _analogueInputStates[_pinLSBByte];
  }

  // Obtain the correct digital input value
  int _read(VPIN vpin) override {
    if (_deviceState == DEVSTATE_FAILED) return 0;
    int pin = vpin - _firstVpin;
    if (_servoData[pin] == NULL) {
      uint8_t pinByte = pin / 8;
      bool value = bitRead(_digitalInputStates[pinByte], pin - pinByte * 8);
      return value;
    } else {
      struct ServoData *s = _servoData[pin];
      if (s == NULL) {
        return false; // No structure means no animation!
      } else {
        return (s->stepNumber < s->numSteps);
      }
    }
  }

  void _write(VPIN vpin, int value) override {
    if (_deviceState == DEVSTATE_FAILED) return;
    int pin = vpin - _firstVpin;
    if (_servoData[pin] == NULL) {
      _digitalOutBuffer[0] = EXIOWRD;
      _digitalOutBuffer[1] = pin;
      _digitalOutBuffer[2] = value;
      I2CManager.read(_i2cAddress, _command1Buffer, 1, _digitalOutBuffer, 3);
      if (_command1Buffer[0] != EXIORDY) {
        DIAG(F("Vpin %d cannot be used as a digital output pin"), (int)vpin);
      }
    } else {
      if (value) value = 1;
      struct ServoData *s = _servoData[pin];
      if (s != NULL) {
        // Use configured parameters
        this->_writeAnalogue(vpin, value ? s->activePosition : s->inactivePosition, s->profile, s->duration);
      }  else {
        /* simulate digital pin on PWM */
        this->_writeAnalogue(vpin, value ? 4095 : 0, Instant | NoPowerOff, 0);     
      }
    }
  }

  // void _writeAnalogue(VPIN vpin, int value, uint8_t param1, uint16_t param2) override {
  void _writeAnalogue(VPIN vpin, int value, uint8_t profile, uint16_t duration) override {
    int pin = vpin - _firstVpin;
    /* Initial _writeAnalogue here
    _command4Buffer[0] = EXIOWRAN;
    _command4Buffer[1] = pin;
    _command4Buffer[2] = value & 0xFF;
    _command4Buffer[3] = value >> 8;
    I2CManager.write(_i2cAddress, _command4Buffer, 4);
    */
#ifdef DIAG_IO
    DIAG(F("Servo: WriteAnalogue Vpin:%d Value:%d Profile:%d Duration:%d %S"), 
      vpin, value, profile, duration, _deviceState == DEVSTATE_FAILED?F("DEVSTATE_FAILED"):F(""));
#endif
    if (_deviceState == DEVSTATE_FAILED) return;
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
      // Send servo command
      this->writePWM(pin, s->currentPosition);
    } else if (s->stepNumber < s->numSteps + _catchupSteps) {
      // We've finished animation, wait a little to allow servo to catch up
      s->stepNumber++;
    } else if (s->stepNumber == s->numSteps + _catchupSteps 
              && s->currentPosition != 0) {
  // #ifdef IO_SWITCH_OFF_SERVO
  //     if ((s->currentProfile & NoPowerOff) == 0) {
  //       // Wait has finished, so switch off PWM to prevent annoying servo buzz
  //       // _slaveDevice->writeAnalogue(_firstSlavePin+pin, 0);
  //     }
  // #endif
      s->numSteps = 0;  // Done now.
    }
  }

  void writePWM(int pin, uint16_t value) {
    _command4Buffer[0] = EXIOWRAN;
    _command4Buffer[1] = pin;
    _command4Buffer[2] = value & 0xFF;
    _command4Buffer[3] = value >> 8;
    I2CManager.write(_i2cAddress, _command4Buffer, 4);
  }

  void _display() override {
    DIAG(F("EX-IOExpander I2C:x%x v%d.%d.%d Vpins %d-%d %S"),
              _i2cAddress, _majorVer, _minorVer, _patchVer,
              (int)_firstVpin, (int)_firstVpin+_nPins-1,
              _deviceState == DEVSTATE_FAILED ? F("OFFLINE") : F(""));
  }

  uint8_t _i2cAddress;
  uint8_t _numDigitalPins = 0;
  uint8_t _numAnaloguePins = 0;
  byte _digitalOutBuffer[3];
  uint8_t _versionBuffer[3];
  uint8_t _majorVer = 0;
  uint8_t _minorVer = 0;
  uint8_t _patchVer = 0;
  byte* _digitalInputStates;
  byte* _analogueInputStates;
  uint8_t _digitalPinBytes = 0;
  uint8_t _analoguePinBytes = 0;
  byte _command1Buffer[1];
  byte _command2Buffer[2];
  byte _command4Buffer[4];
  byte _receive3Buffer[3];
  uint8_t* _analoguePinMap;

  // Servo specific
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
  
  struct ServoData *_servoData[256];

  static const uint8_t _catchupSteps = 5; // number of steps to wait before switching servo off
  // static const uint8_t FLASH _bounceProfile[30];

  const unsigned int refreshInterval = 50; // refresh every 50ms

  // Profile for a bouncing signal or turnout
  // The profile below is in the range 0-100% and should be combined with the desired limits
  // of the servo set by _activePosition and _inactivePosition.  The profile is symmetrical here,
  // i.e. the bounce is the same on the down action as on the up action.  First entry isn't used.
  const byte FLASH _bounceProfile[30] = 
    {0,2,3,7,13,33,50,83,100,83,75,70,65,60,60,65,74,84,100,83,75,70,70,72,75,80,87,92,97,100};

  // EX-IOExpander protocol flags
  enum {
    EXIOINIT = 0xE0,    // Flag to initialise setup procedure
    EXIORDY = 0xE1,     // Flag we have completed setup procedure, also for EX-IO to ACK setup
    EXIODPUP = 0xE2,    // Flag we're sending digital pin pullup configuration
    EXIOVER = 0xE3,     // Flag to get version
    EXIORDAN = 0xE4,    // Flag to read an analogue input
    EXIOWRD = 0xE5,     // Flag for digital write
    EXIORDD = 0xE6,     // Flag to read digital input
    EXIOENAN = 0xE7,    // Flag to enable an analogue pin
    EXIOINITA = 0xE8,   // Flag we're receiving analogue pin mappings
    EXIOPINS = 0xE9,    // Flag we're receiving pin counts for buffers
    EXIOWRAN = 0xEA,   // Flag we're sending an analogue write (PWM)
    EXIOERR = 0xEF,     // Flag we've received an error
  };
};

#endif
