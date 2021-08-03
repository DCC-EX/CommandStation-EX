/*
 *  © 2021, Neil McKechnie. All rights reserved.
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

#ifndef IO_PCF8574_H
#define IO_PCF8574_H

#include "IO_GPIOBase.h"

class PCF8574 : public GPIOBase<uint8_t> {
public:
  static void create(VPIN firstVpin, uint8_t nPins, uint8_t I2CAddress, int interruptPin=-1) {
    new PCF8574(firstVpin, nPins, I2CAddress, interruptPin);
  }

private:
  PCF8574(VPIN firstVpin, uint8_t nPins, uint8_t I2CAddress, int interruptPin=-1)
    : GPIOBase<uint8_t>((FSH *)F("PCF8574"), firstVpin, min(nPins, 8), I2CAddress, interruptPin) 
  {
    requestBlock.setReadParams(_I2CAddress, inputBuffer, 1);
  }
  
  // The pin state is '1' if the pin is an input or if it is an output set to 1.  Zero otherwise. 
  void _writeGpioPort() override {
    I2CManager.write(_I2CAddress, 1, _portOutputState | ~_portMode);
  }

  // The PCF8574 handles inputs by applying a weak pull-up when output is driven to '1'.
  // Therefore, writing '1' in _writePortModes is enough to set the module to input mode 
  // and enable pull-up.
  void _writePullups() override { }

  // The pin state is '1' if the pin is an input or if it is an output set to 1.  Zero otherwise. 
  void _writePortModes() override {
    I2CManager.write(_I2CAddress, 1, _portOutputState | ~_portMode);
  }

  // In immediate mode, _readGpioPort reads the device GPIO port and updates _portInputState accordingly.
  //  When not in immediate mode, it initiates a request using the request block and returns.
  //  When the request completes, _processCompletion finishes the operation.
  void _readGpioPort(bool immediate) override {
    if (immediate) {
      uint8_t buffer[1];
      I2CManager.read(_I2CAddress, buffer, 1);
      _portInputState = ((uint16_t)buffer) & 0xff;
    } else {
      requestBlock.wait(); // Wait for preceding operation to complete
      // Issue new request to read GPIO register
      I2CManager.queueRequest(&requestBlock);
    }
  }

  // This function is invoked when an I/O operation on the requestBlock completes.
  void _processCompletion(uint8_t status) override {
    if (status == I2C_STATUS_OK) 
      _portInputState = ((uint16_t)inputBuffer[0]) & 0xff;
    else  
      _portInputState = 0xff;
  }

  void _setupDevice() override { }
 
  uint8_t inputBuffer[1];
};

#endif