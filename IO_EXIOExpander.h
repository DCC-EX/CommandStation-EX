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
  static void create(VPIN vpin, int nPins, uint8_t i2cAddress) {
    if (checkNoOverlap(vpin, nPins, i2cAddress)) new EXIOExpander(vpin, nPins, i2cAddress);
  }

private:  
  // Constructor
  EXIOExpander(VPIN firstVpin, int nPins, uint8_t i2cAddress) {
    _firstVpin = firstVpin;
    _nPins = nPins;
    _i2cAddress = i2cAddress;
    _digitalPinBytes = (nPins+7)/8;
    _digitalInputStates=(byte*) calloc(_digitalPinBytes,1);
    addDevice(this);
  }

  void _begin() {
    // Initialise EX-IOExander device
    I2CManager.begin();
    if (I2CManager.exists(_i2cAddress)) {
      _command2Buffer[0] = EXIOINIT;
      _command2Buffer[1] = _nPins;
      // Send config, if EXIOINITA returned, we're good, setup analogue input buffer, otherwise go offline
      I2CManager.read(_i2cAddress, _receive3Buffer, 3, _command2Buffer, 2);
      if (_receive3Buffer[0] == EXIOINITA) {
        _numAnaloguePins = _receive3Buffer[1];
        _numPWMPins = _receive3Buffer[2];
        _analoguePinBytes = _numAnaloguePins * 2;
        _analogueInputStates = (byte*) calloc(_analoguePinBytes, 1);
        _analoguePinMap = (uint8_t*) calloc(_numAnaloguePins, 1);
        _servoData = (struct ServoData*) calloc(_numPWMPins, 14);

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
      I2CManager.write(_i2cAddress, _digitalOutBuffer, 3);
      return true;
    } else if (configType == CONFIGURE_SERVO) {
      DIAG(F("Configure servo at pin %d"), (int)pin);
      for (int i = 0; i < paramCount; i++) {
        DIAG(F("Param %d is %x"), (int)i, params[i]);
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
    I2CManager.write(_i2cAddress, _command2Buffer, 2);
    return true;
  }

  // Main loop, collect both digital and analogue pin states continuously (faster sensor/input reads)
  void _loop(unsigned long currentMicros) override {
    (void)currentMicros; // remove warning
    _command1Buffer[0] = EXIORDD;
    I2CManager.read(_i2cAddress, _digitalInputStates, _digitalPinBytes, _command1Buffer, 1);
    _command1Buffer[0] = EXIORDAN;
    I2CManager.read(_i2cAddress, _analogueInputStates, _analoguePinBytes, _command1Buffer, 1);
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
    int pin = vpin - _firstVpin;
    uint8_t pinByte = pin / 8;
    bool value = _digitalInputStates[pinByte] >> (pin - pinByte * 8);
    return value;
  }

  void _write(VPIN vpin, int value) override {
    int pin = vpin - _firstVpin;
    _digitalOutBuffer[0] = EXIOWRD;
    _digitalOutBuffer[1] = pin;
    _digitalOutBuffer[2] = value;
    I2CManager.write(_i2cAddress, _digitalOutBuffer, 3);
  }

  void _display() override {
    DIAG(F("EX-IOExpander I2C:x%x v%d.%d.%d Vpins %d-%d %S"),
              _i2cAddress, _majorVer, _minorVer, _patchVer,
              (int)_firstVpin, (int)_firstVpin+_nPins-1,
              _deviceState == DEVSTATE_FAILED ? F("OFFLINE") : F(""));
  }

  uint8_t _i2cAddress;
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
  byte _receive3Buffer[3];
  uint8_t* _analoguePinMap;
  uint8_t _numPWMPins = 0;

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
  } ServoData; // 14 bytes per element, i.e. per pin in use
  
  struct ServoData* _servoData;

  static const uint8_t _catchupSteps = 5; // number of steps to wait before switching servo off
  static const byte FLASH _bounceProfile[30];

  const unsigned int refreshInterval = 50; // refresh every 50ms

  enum {
    EXIOINIT = 0xE0,    // Flag to initialise setup procedure
    EXIORDY = 0xE1,     // Flag we have completed setup procedure, also for EX-IO to ACK setup
    EXIODPUP = 0xE2,    // Flag we're sending digital pin pullup configuration
    EXIOVER = 0xE3,     // Flag to get version
    EXIORDAN = 0xE4,    // Flag to read an analogue input
    EXIOWRD = 0xE5,     // Flag for digital write
    EXIORDD = 0xE6,     // Flag to read digital input
    EXIOENAN = 0xE7,    // Flag eo enable an analogue pin
    EXIOINITA = 0xE8,   // Flag we're receiving analogue pin info
  };
};

#endif
