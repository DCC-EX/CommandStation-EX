/*
 *  © 2022 Paul M Antoine
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

/* 
 * The PCF8574 is a simple device; it only has one register.  The device 
 * input/output mode and pullup are configured through this, and the 
 * output state is written and the input state read through it too.
 * 
 * This is accomplished by having a weak resistor in series with the output,
 * and a read-back of the other end of the resistor.  As an output, the 
 * pin state is set to 1 or 0, and the output voltage goes to +5V or 0V
 * (through the weak resistor).
 * 
 * In order to use the pin as an input, the output is written as
 * a '1' in order to pull up the resistor.  Therefore the input will be
 * 1 unless the pin is pulled down externally, in which case it will be 0.
 * 
 * As a consequence of this approach, it is not possible to use the device for
 * inputs without pullups.
 */

#ifndef IO_PCF8574_H
#define IO_PCF8574_H

#include "IO_GPIOBase.h"

class PCF8574 : public GPIOBase<uint8_t> {
public:
  static void create(VPIN firstVpin, uint8_t nPins, I2CAddress i2cAddress, int interruptPin=-1) {
    if (checkNoOverlap(firstVpin, nPins, i2cAddress)) new PCF8574(firstVpin, nPins, i2cAddress, interruptPin);
  }

private:
  PCF8574(VPIN firstVpin, uint8_t nPins, I2CAddress i2cAddress, int interruptPin=-1)
    : GPIOBase<uint8_t>((FSH *)F("PCF8574"), firstVpin, nPins, i2cAddress, interruptPin) 
  {
    requestBlock.setReadParams(_I2CAddress, inputBuffer, 1);
  }
  
  // The PCF8574 handles inputs by applying a weak pull-up when output is driven to '1'.
  // The pin state is driven '1' if the pin is an input, or if it is an output set to 1.
  // Unused pins are driven '0'.
  void _writeGpioPort() override {
    I2CManager.write(_I2CAddress, 1, (_portOutputState | ~_portMode) & _portInUse);
  }

  // The PCF8574 handles inputs by applying a weak pull-up when output is driven to '1'.
  // Therefore, writing '1' in _writePortModes is enough to set the module to input mode 
  // and enable pull-up.
  void _writePullups() override { }

  void _writePortModes() override {
    _writeGpioPort();
  }

  // In immediate mode, _readGpioPort reads the device GPIO port and updates _portInputState accordingly.
  //  When not in immediate mode, it initiates a request using the request block and returns.
  //  When the request completes, _processCompletion finishes the operation.
  void _readGpioPort(bool immediate) override {
    if (immediate) {
      uint8_t buffer[1];
      I2CManager.read(_I2CAddress, buffer, 1);
      _portInputState = buffer[0] | _portMode;
    } else {
      requestBlock.wait(); // Wait for preceding operation to complete
      // Issue new request to read GPIO register
      I2CManager.queueRequest(&requestBlock);
    }
  }

  // This function is invoked when an I/O operation on the requestBlock completes.
  void _processCompletion(uint8_t status) override {
    if (status == I2C_STATUS_OK) 
      _portInputState = inputBuffer[0] | _portMode;
    else  
      _portInputState = 0xff; 
  }

  // Set up device ports
  void _setupDevice() override { 
    _writePortModes();
  }
 
  uint8_t inputBuffer[1];
};

#endif