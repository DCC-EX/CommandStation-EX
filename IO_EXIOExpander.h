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
*   // EXIOExpander::create(vpin, num_vpins, i2c_address, digitalPinCount, analoguePinCount);
*   EXIOExpander::create(800, 18, 0x65, EXIO_NANO_DIGITAL_PINS, EXIO_NANO_ANALOGUE_PINS);
* }
* 
* Note when defining the number of digital and analogue pins, there is no way to sanity check
* this from the device driver, and it is up to the user to define the correct values here.
* 
* Vpins are allocated to digital pins first, and then analogue pins, so digital pins will
* populate the first part of the specified vpin range, with the analogue pins populating the
* last part of the vpin range.
* Eg. for a default Nano, 800 - 811 are digital (D2 - D13), 812 to 817 are analogue (A0 - A3, A6/A7).
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
  static void create(VPIN vpin, int nPins, uint8_t i2cAddress, byte numDigitalPins, byte numAnaloguePins) {
    if (checkNoOverlap(vpin, nPins, i2cAddress)) new EXIOExpander(vpin, nPins, i2cAddress, numDigitalPins, numAnaloguePins);
  }

private:  
  // Constructor
  EXIOExpander(VPIN firstVpin, int nPins, uint8_t i2cAddress, byte numDigitalPins, byte numAnaloguePins) {
    _firstVpin = firstVpin;
    _nPins = nPins;
    _i2cAddress = i2cAddress;
    _numDigitalPins = numDigitalPins;
    _numAnaloguePins = numAnaloguePins;
    _digitalOutBuffer = (byte *)calloc(_numDigitalPins + 1, 1);
    _digitalInBuffer = (byte *)calloc(_numDigitalPins, 1);
    _analogueOutBuffer = (byte *)calloc(_numAnaloguePins + 1, 1);
    _analogueInBuffer = (byte *)calloc(_numAnaloguePins, 1);
    addDevice(this);
  }

  void _begin() {
    // Initialise EX-IOExander device
    uint8_t _check = I2CManager.checkAddress(_i2cAddress);
    if (I2CManager.exists(_i2cAddress)) {
      _activity = EXIOINIT;   // First thing to do is configure EX-IOExpander device
#ifdef DIAG_IO
      _display();
#endif
    } else {
      DIAG(F("EX-IOExpander device not found, I2C:x%x"), _i2cAddress);
      _deviceState = DEVSTATE_FAILED;
    }
  }

  void _loop(unsigned long currentMicros) override {
    if (_i2crb.status == I2C_STATUS_PENDING) return;  // Do nothing if I2C isn't ready yet
    if (_i2crb.status == I2C_STATUS_OK) {
      switch(_activity) {
        case EXIOINIT:
          // Send digital and analogue pin counts to configure EX-IOExpander
          _setupBuffer[0] = EXIOINIT;
          _setupBuffer[1] = _numDigitalPins;
          _setupBuffer[2] = _numAnaloguePins;
          I2CManager.write(_i2cAddress, _setupBuffer, 3, &_i2crb);
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
  uint8_t _setupBuffer[3];
  byte * _digitalOutBuffer = NULL;
  byte * _digitalInBuffer = NULL;
  byte * _analogueOutBuffer = NULL;
  byte * _analogueInBuffer = NULL;
  uint8_t _activity;
  I2CRB _i2crb;

  enum {
    EXIOINIT = 0xE0,    // Flag to initialise setup procedure
    EXIORDY = 0xE1,     // Flag we have completed setup procedure, also for EX-IO to ACK setup
    EXIODDIR = 0xE2,    // Flag we're sending digital pin direction configuration
    EXIODPUP = 0xE3,    // Flag we're sending digital pin pullup configuration
  };
};

#endif