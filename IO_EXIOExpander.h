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

  static void create(VPIN vpin, int nPins, I2CAddress i2cAddress) {
    if (checkNoOverlap(vpin, nPins, i2cAddress)) new EXIOExpander(vpin, nPins, i2cAddress);
  }

private:  
  // Constructor
  EXIOExpander(VPIN firstVpin, int nPins, I2CAddress i2cAddress) {
    _firstVpin = firstVpin;
    _nPins = nPins;
    _i2cAddress = i2cAddress;
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
      I2CManager.read(_i2cAddress, _receive3Buffer, 3, _command4Buffer, 4, &_i2crb);
      if (_receive3Buffer[0] == EXIOPINS) {
        _numDigitalPins = _receive3Buffer[1];
        _numAnaloguePins = _receive3Buffer[2];
        _digitalPinBytes = (_numDigitalPins + 7)/8;
        _digitalInputStates=(byte*) calloc(_digitalPinBytes,1);
        _analoguePinBytes = _numAnaloguePins * 2;
        _analogueInputStates = (byte*) calloc(_analoguePinBytes, 1);
        _analoguePinMap = (uint8_t*) calloc(_numAnaloguePins, 1);
      } else {
        DIAG(F("ERROR configuring EX-IOExpander device, I2C:%s"), _i2cAddress.toString());
        _deviceState = DEVSTATE_FAILED;
        return;
      }
      // We now need to retrieve the analogue pin map
      _command1Buffer[0] = EXIOINITA;
      I2CManager.read(_i2cAddress, _analoguePinMap, _numAnaloguePins, _command1Buffer, 1, &_i2crb);
      // Attempt to get version, if we don't get it, we don't care, don't go offline
      _command1Buffer[0] = EXIOVER;
      I2CManager.read(_i2cAddress, _versionBuffer, 3, _command1Buffer, 1, &_i2crb);
      _majorVer = _versionBuffer[0];
      _minorVer = _versionBuffer[1];
      _patchVer = _versionBuffer[2];
      DIAG(F("EX-IOExpander device found, I2C:%s, Version v%d.%d.%d"),
          _i2cAddress.toString(), _versionBuffer[0], _versionBuffer[1], _versionBuffer[2]);
#ifdef DIAG_IO
      _display();
#endif
    } else {
      DIAG(F("EX-IOExpander device not found, I2C:%s"), _i2cAddress.toString());
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
      I2CManager.read(_i2cAddress, _command1Buffer, 1, _digitalOutBuffer, 3, &_i2crb);
      if (_command1Buffer[0] == EXIORDY) {
        return true;
      } else {
        DIAG(F("Vpin %d cannot be used as a digital input pin"), (int)vpin);
        return false;
      }
    } else {
      return false;
    }
  }

  // Analogue input pin configuration, used to enable on EX-IOExpander device
  int _configureAnalogIn(VPIN vpin) override {
    int pin = vpin - _firstVpin;
    _command2Buffer[0] = EXIOENAN;
    _command2Buffer[1] = pin;
    I2CManager.read(_i2cAddress, _command1Buffer, 1, _command2Buffer, 2, &_i2crb);
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
    if (_deviceState == DEVSTATE_FAILED) return;    // If device failed, return
    uint8_t status = _i2crb.status;
    if (status == I2C_STATUS_PENDING) return;  // If device busy, return
    if (status == I2C_STATUS_OK) {             // If device ok, read input data
      if (_commandFlag) {
        _command1Buffer[0] = EXIORDD;
        I2CManager.read(_i2cAddress, _digitalInputStates, _digitalPinBytes, _command1Buffer, 1, &_i2crb);
      } else {
        _command1Buffer[0] = EXIORDAN;
        byte _tempAnalogue[_analoguePinBytes];      // Setup temp buffer so reads come from known state
        I2CManager.read(_i2cAddress, _tempAnalogue, _analoguePinBytes, _command1Buffer, 1, &_i2crb);
        memcpy(_analogueInputStates, _tempAnalogue, _analoguePinBytes); // Copy temp buffer to states
      }
      _commandFlag = !_commandFlag;
      // Need to delay here: digital in IO_Base 4000UL, analogue in IO_AnalogueInputs 10000UL (fast) or 1000000UL(slow)
    } else {
      DIAG(F("EX-IOExpander I2C:%s Error:%d %S"), _I2CAddress.toString(), status, I2CManager.getErrorMessage(status));
      _deviceState = DEVSTATE_FAILED;
    }
  }

  // Obtain the correct analogue input value
  int _readAnalogue(VPIN vpin) override {
    if (_deviceState == DEVSTATE_FAILED) return 0;
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
    uint8_t pinByte = pin / 8;
    bool value = bitRead(_digitalInputStates[pinByte], pin - pinByte * 8);
    return value;
  }

  void _write(VPIN vpin, int value) override {
    if (_deviceState == DEVSTATE_FAILED) return;
    int pin = vpin - _firstVpin;
    _digitalOutBuffer[0] = EXIOWRD;
    _digitalOutBuffer[1] = pin;
    _digitalOutBuffer[2] = value;
    uint8_t status = I2CManager.read(_i2cAddress, _command1Buffer, 1, _digitalOutBuffer, 3);
    if (status != I2C_STATUS_OK) {
      DIAG(F("EX-IOExpander I2C:%s Error:%d %S"), _I2CAddress.toString(), status, I2CManager.getErrorMessage(status));
      _deviceState = DEVSTATE_FAILED;
    } else {
      if (_command1Buffer[0] != EXIORDY) {
        DIAG(F("Vpin %d cannot be used as a digital output pin"), (int)vpin);
      }
    }
  }

  void _writeAnalogue(VPIN vpin, int value, uint8_t profile, uint16_t duration) override {
    if (_deviceState == DEVSTATE_FAILED) return;
    int pin = vpin - _firstVpin;
#ifdef DIAG_IO
    DIAG(F("Servo: WriteAnalogue Vpin:%d Value:%d Profile:%d Duration:%d %S"), 
      vpin, value, profile, duration, _deviceState == DEVSTATE_FAILED?F("DEVSTATE_FAILED"):F(""));
#endif
    _servoBuffer[0] = EXIOWRAN;
    _servoBuffer[1] = pin;
    _servoBuffer[2] = value & 0xFF;
    _servoBuffer[3] = value >> 8;
    _servoBuffer[4] = profile;
    _servoBuffer[5] = duration & 0xFF;
    _servoBuffer[6] = duration >> 8;
    uint8_t status = I2CManager.read(_i2cAddress, _command1Buffer, 1, _servoBuffer, 7);
    if (status != I2C_STATUS_OK) {
      DIAG(F("EX-IOExpander I2C:%s Error:%d %S"), _I2CAddress.toString(), status, I2CManager.getErrorMessage(status));
      _deviceState = DEVSTATE_FAILED;
    } else {
      if (_command1Buffer[0] != EXIORDY) {
        DIAG(F("Vpin %d cannot be used as a servo/PWM pin"), (int)vpin);
      }
    }
  }

  void _display() override {
    DIAG(F("EX-IOExpander I2C:%s v%d.%d.%d Vpins %d-%d %S"),
              _i2cAddress.toString(), _majorVer, _minorVer, _patchVer,
              (int)_firstVpin, (int)_firstVpin+_nPins-1,
              _deviceState == DEVSTATE_FAILED ? F("OFFLINE") : F(""));
  }

  I2CAddress _i2cAddress;
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
  byte _servoBuffer[7];
  uint8_t* _analoguePinMap;
  I2CRB _i2crb;
  bool _commandFlag = 1;

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
