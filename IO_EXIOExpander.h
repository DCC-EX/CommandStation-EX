/*
 *  © 2021, Peter Cole. All rights reserved.
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
*   // EXIOExpander::create(vpin, num_vpins, i2c_address, digitalPinCount, analoguePinCount);
*   EXIOExpander::create(800, 18, 0x65, 12, 8);
* }
* 
* Note when defining the number of digital and analogue pins, there is no way to sanity check
* this from the device driver, and it is up to the user to define the correct values here.
*
* All pins available on the EX-IOExpander device must be accounted for.
* 
* Vpins are allocated to digital pins first, and then analogue pins, so digital pins will
* populate the first part of the specified vpin range, with the analogue pins populating the
* last part of the vpin range.
* Eg. for a default Nano, 800 - 811 are digital (D2 - D13), 812 to 817 are analogue (A0 - A3, A6/A7).
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
  static void create(VPIN vpin, int nPins, I2CAddress i2cAddress, int numDigitalPins, int numAnaloguePins) {
    if (checkNoOverlap(vpin, nPins, i2cAddress)) new EXIOExpander(vpin, nPins, i2cAddress, numDigitalPins, numAnaloguePins);
  }

private:  
  // Constructor
  EXIOExpander(VPIN firstVpin, int nPins, I2CAddress i2cAddress, int numDigitalPins, int numAnaloguePins) {
    _firstVpin = firstVpin;
    _nPins = nPins;
    _i2cAddress = i2cAddress;
    _numDigitalPins = numDigitalPins;
    _numAnaloguePins = numAnaloguePins;
    _digitalPinBytes = (numDigitalPins+7)/8;
    _analoguePinBytes = numAnaloguePins * 2;
    _digitalInputStates=(byte*) calloc(_digitalPinBytes,1);
    _analogueInputStates=(byte*) calloc(_analoguePinBytes,1);
    addDevice(this);
  }

  void _begin() {
    // Initialise EX-IOExander device
    I2CManager.begin();
    if (I2CManager.exists(_i2cAddress)) {
      _digitalOutBuffer[0] = EXIOINIT;
      _digitalOutBuffer[1] = _numDigitalPins;
      _digitalOutBuffer[2] = _numAnaloguePins;
      // Send config, if EXIORDY returned, we're good, otherwise go offline
      I2CManager.read(_i2cAddress, _commandBuffer, 1, _digitalOutBuffer, 3);
      if (_commandBuffer[0] != EXIORDY) {
        DIAG(F("ERROR configuring EX-IOExpander device, I2C:x%x"), (int)_i2cAddress);
        _deviceState = DEVSTATE_FAILED;
        return;
      }
      // Attempt to get version, if we don't get it, we don't care, don't go offline
      // Using digital in buffer in reverse to save RAM
      _commandBuffer[0] = EXIOVER;
      I2CManager.read(_i2cAddress, _versionBuffer, 3, _commandBuffer, 1);
      _majorVer = _versionBuffer[0];
      _minorVer = _versionBuffer[1];
      _patchVer = _versionBuffer[2];
      DIAG(F("EX-IOExpander device found, I2C:x%x, Version v%d.%d.%d"),
          (int)_i2cAddress, _versionBuffer[0], _versionBuffer[1], _versionBuffer[2]);
#ifdef DIAG_IO
      _display();
#endif
    } else {
      DIAG(F("EX-IOExpander device not found, I2C:x%x"), (int)_i2cAddress);
      _deviceState = DEVSTATE_FAILED;
    }
  }

  bool _configure(VPIN vpin, ConfigTypeEnum configType, int paramCount, int params[]) override {
    if (configType != CONFIGURE_INPUT) return false;
    if (paramCount != 1) return false;
    if (vpin >= _firstVpin + _numDigitalPins) {
      DIAG(F("EX-IOExpander ERROR: Vpin %d is an analogue pin, cannot use as a digital pin"), vpin);
      return false;
    }
    bool pullup = params[0];
    int pin = vpin - _firstVpin;
    _digitalOutBuffer[0] = EXIODPUP;
    _digitalOutBuffer[1] = pin;
    _digitalOutBuffer[2] = pullup;
    I2CManager.write(_i2cAddress, _digitalOutBuffer, 3);
    return true;
  }

  // We only use this to detect incorrect use of analogue pins
  int _configureAnalogIn(VPIN vpin) override {
    if (vpin < _firstVpin + _numDigitalPins) {
      DIAG(F("EX-IOExpander ERROR: Vpin %d is a digital pin, cannot use as an analogue pin"), vpin);
      return false;
    }
    int pin = vpin - _firstVpin;
    _analogueOutBuffer[0] = EXIOENAN;
    _analogueOutBuffer[1] = pin;
    I2CManager.write(_i2cAddress, _analogueOutBuffer, 2);
    return true;
  }

  void _loop(unsigned long currentMicros) override {
    (void)currentMicros; // remove warning
    _commandBuffer[0] = EXIORDD;
    I2CManager.read(_i2cAddress, _digitalInputStates, _digitalPinBytes, _commandBuffer, 1);
    _commandBuffer[0] = EXIORDAN;
    I2CManager.read(_i2cAddress, _analogueInputStates, _analoguePinBytes, _commandBuffer, 1);
  }

  int _readAnalogue(VPIN vpin) override {
    if (vpin < _firstVpin + _numDigitalPins) return false;
    int pin = vpin - _firstVpin - _numDigitalPins;
    uint8_t _pinLSBByte = pin * 2;
    uint8_t _pinMSBByte = _pinLSBByte + 1;
    return (_analogueInputStates[_pinMSBByte] << 8) + _analogueInputStates[_pinLSBByte];
  }

  int _read(VPIN vpin) override {
    if (vpin >= _firstVpin + _numDigitalPins) return false;
    int pin = vpin - _firstVpin;
    uint8_t pinByte = pin / 8;
    bool value = _digitalInputStates[pinByte] >> (pin - pinByte * 8);
    return value;
  }

  void _write(VPIN vpin, int value) override {
    if (vpin >= _firstVpin + _numDigitalPins) return;
    int pin = vpin - _firstVpin;
    _digitalOutBuffer[0] = EXIOWRD;
    _digitalOutBuffer[1] = pin;
    _digitalOutBuffer[2] = value;
    I2CManager.write(_i2cAddress, _digitalOutBuffer, 3);
  }

  void _display() override {
    int _firstAnalogue, _lastAnalogue;
    if (_numAnaloguePins == 0) {
      _firstAnalogue = 0;
      _lastAnalogue = 0;
    } else {
      _firstAnalogue = _firstVpin + _numDigitalPins;
      _lastAnalogue = _firstVpin + _nPins - 1;
    }
    DIAG(F("EX-IOExpander I2C:x%x v%d.%d.%d: %d Digital Vpins %d-%d, %d Analogue Vpins %d-%d %S"),
              (int)_i2cAddress, _majorVer, _minorVer, _patchVer,
              _numDigitalPins, _firstVpin, _firstVpin + _numDigitalPins - 1,
              _numAnaloguePins, _firstAnalogue, _lastAnalogue,
              _deviceState == DEVSTATE_FAILED ? F("OFFLINE") : F(""));
  }

  uint8_t _i2cAddress;
  uint8_t _numDigitalPins;
  uint8_t _numAnaloguePins;
  byte _analogueOutBuffer[2];
  byte _digitalOutBuffer[3];
  uint8_t _versionBuffer[3];
  uint8_t _majorVer = 0;
  uint8_t _minorVer = 0;
  uint8_t _patchVer = 0;
  byte* _digitalInputStates;
  byte* _analogueInputStates;
  uint8_t _digitalPinBytes = 0;
  uint8_t _analoguePinBytes = 0;
  byte _commandBuffer[1];

  enum {
    EXIOINIT = 0xE0,    // Flag to initialise setup procedure
    EXIORDY = 0xE1,     // Flag we have completed setup procedure, also for EX-IO to ACK setup
    EXIODPUP = 0xE2,    // Flag we're sending digital pin pullup configuration
    EXIOVER = 0xE3,     // Flag to get version
    EXIORDAN = 0xE4,    // Flag to read an analogue input
    EXIOWRD = 0xE5,     // Flag for digital write
    EXIORDD = 0xE6,     // Flag to read digital input
    EXIOENAN = 0xE7,    // Flag eo enable an analogue pin
  };
};

#endif
