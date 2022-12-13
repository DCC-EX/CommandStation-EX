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
* The IO_EX-IOExpander.h device driver integrates with one or more EX-IOExpander devices.
* This device driver will configure the device and all I/O ports on startup, along with
* interacting with the device for all input/output duties.
*
* To create EX-IOExpander devices, these are defined in myHal.cpp:
*
* #include "IO_EX-IOExpander.h"
*
* void halSetup() {
*   // EXIOExpander::create(vpin, num_vpins, i2c_address);
*   EXIOExpander::create(800, 18, 0x90);
}
*/

#ifndef IO_EX_IOEXPANDER_H
#define IO_EX_IOEXPANDER_H

#include "IODevice.h"
#include "I2CManager.h"
#include "DIAG.h"
#include "FSH.h"
#include "EX-IOExpanderPins.h"

// Include user defined pin maps in myEX-IOExpander if defined
#if __has_include ("myEX-IOExpander.h")
  #include "myEX-IOExpander.h"
#endif

/////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 * IODevice subclass for EX-IOExpander.
 */
class EXIOExpander : public IODevice {
public:
  static void create(VPIN vpin, int nPins, uint8_t i2cAddress, uint8_t numDigitalPins, uint8_t numAnaloguePins) {
    if (checkNoOverlap(vpin, nPins, i2cAddress)) new EXIOExpander(vpin, nPins, i2cAddress, numDigitalPins, numAnaloguePins);
  }

private:  
  // Constructor
  EXIOExpander(VPIN firstVpin, int nPins, uint8_t i2cAddress, uint8_t numDigitalPins, uint8_t numAnaloguePins) {
    _firstVpin = firstVpin;
    _nPins = nPins;
    _i2cAddress = i2cAddress;
    _numDigitalPins = numDigitalPins;
    _numAnaloguePins = numAnaloguePins;
    addDevice(this);
  }

  void _begin() {
    // Initialise EX-IOExander device
    if (I2CManager.exists(_i2cAddress)) {
#ifdef DIAG_IO
      _display();
#endif
      _setupDevice();
    } else {
      DIAG(F("EX-IOExpander device not found, I2C:%x"), _i2cAddress);
      _deviceState = DEVSTATE_FAILED;
    }
  }

  void _setupDevice() {
    // Send digital and analogue pin counts
    uint8_t _setupBuffer[3] = {EXIOINIT, _numDigitalPins, _numAnaloguePins};
    // I2CManager.write(_i2cAddress, 3, EXIOINIT, _numDigitalPins, _numAnaloguePins);
    I2CManager.write(_i2cAddress, _setupBuffer, 3, &_i2crb);
    _activity = EXIODPIN;
  }

  void _loop(unsigned long currentMicros) override {
    if (_i2crb.status == I2C_STATUS_PENDING) return;
    if (_i2crb.status == I2C_STATUS_OK) {
      switch(_activity) {
        case EXIODPIN:
          // Enable digital ports
          _digitalPinBytes = (_numDigitalPins + 7) / 8 + 1;
          // uint8_t _enableDigitalPins[_digitalPinBytes];
          // _enableDigitalPins[0] = EXIODPIN;
          _digitalOutBuffer[0] = EXIODPIN;
          for (uint8_t byte = 1; byte < _digitalPinBytes; byte++) {
            // _enableDigitalPins[byte] = 0;
            _digitalOutBuffer[byte] = 0;
          }
          for (uint8_t pin = 0; pin < _numDigitalPins; pin++) {
            int pinByte = pin / 8;
            // bitSet(_enableDigitalPins[pinByte + 1], pin - pinByte * 8);
            bitSet(_digitalOutBuffer[pinByte + 1], pin - pinByte * 8);
          }
          // I2CManager.write(_i2cAddress, _enableDigitalPins, _digitalPinBytes, &_i2crb);
          I2CManager.write(_i2cAddress, _digitalOutBuffer, _digitalPinBytes, &_i2crb);
          _activity = EXIOAPIN;
          break;
        case EXIOAPIN:
          // Enable analogue ports
          _analoguePinBytes = (_numAnaloguePins + 7) / 8 + 1;
          // uint8_t _enableAnaloguePins[_analoguePinBytes];
          // _enableAnaloguePins[0] = EXIOAPIN;
          _analogueOutBuffer[0] = EXIOAPIN;
          for (uint8_t byte = 1; byte < _analoguePinBytes; byte++) {
            // _enableAnaloguePins[byte] = 0;
            _analogueOutBuffer[byte] = 0;
          }
          for (uint8_t pin = 0; pin < _numAnaloguePins; pin++) {
            int pinByte = pin / 8;
            // bitSet(_enableAnaloguePins[pinByte + 1], pin - pinByte * 8);
            bitSet(_analogueOutBuffer[pinByte + 1], pin - pinByte * 8);
          }
          // I2CManager.write(_i2cAddress, _enableAnaloguePins, _analoguePinBytes, &_i2crb);
          I2CManager.write(_i2cAddress, _analogueOutBuffer, _analoguePinBytes, &_i2crb);
          _activity = EXIORDY;
          break;
        default:
          break;
      }
    }
  }

  void _display() override {
    DIAG(F("EX-IOExpander I2C:x%x Configured on Vpins:%d-%d %S"), _i2cAddress, _firstVpin, _firstVpin+_nPins-1,
      _deviceState == DEVSTATE_FAILED ? F("OFFLINE") : F(""));
  }

  uint8_t _i2cAddress;
  uint8_t _numDigitalPins;
  uint8_t _numAnaloguePins;
  int _digitalPinBytes;
  int _analoguePinBytes;
  uint8_t _digitalOutBuffer[EXIO_NANO_DIGITAL_PINS + 1];
  uint8_t _digitalInBufer[EXIO_NANO_DIGITAL_PINS];
  uint8_t _analogueOutBuffer[EXIO_NANO_ANALOGUE_PINS + 1];
  uint8_t _analogueInBuffer[EXIO_NANO_ANALOGUE_PINS];
  uint8_t _activity;
  I2CRB _i2crb;

  enum {
    EXIOINIT = 0xE0,    // Flag to initialise setup procedure
    EXIODPIN = 0xE1,    // Flag we're sending digital pin assignments
    EXIOAPIN = 0xE2,    // Flag we're sending analogue pin assignments
    EXIORDY = 0xE3,     // Flag we have completed setup procedure, also for EX-IO to ACK setup
    EXIODDIR = 0xE4,    // Flag we're sending digital pin direction configuration
    EXIODPUP = 0xE5,    // Flag we're sending digital pin pullup configuration
  };
};

#endif