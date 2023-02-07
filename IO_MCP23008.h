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

#ifndef IO_MCP23008_H
#define IO_MCP23008_H

#include "IO_GPIOBase.h"

class MCP23008 : public GPIOBase<uint8_t> {
public:
  static void create(VPIN firstVpin, uint8_t nPins, I2CAddress i2cAddress, int interruptPin=-1) {
    if (checkNoOverlap(firstVpin, nPins,i2cAddress)) new MCP23008(firstVpin, nPins, i2cAddress, interruptPin);
  }

private:
  // Constructor
  MCP23008(VPIN firstVpin, uint8_t nPins, I2CAddress i2cAddress, int interruptPin=-1)
    : GPIOBase<uint8_t>((FSH *)F("MCP23008"), firstVpin, nPins, i2cAddress, interruptPin) {

    requestBlock.setRequestParams(_I2CAddress, inputBuffer, sizeof(inputBuffer),
      outputBuffer, sizeof(outputBuffer));
    outputBuffer[0] = REG_GPIO;
  }
  
  void _writeGpioPort() override {
    I2CManager.write(_I2CAddress, 2, REG_GPIO, _portOutputState);
  }
  void _writePullups() override {
    // Set pullups only for in-use pins.  This prevents pullup being set for a pin that
    //  is intended for use as an output but hasn't been written to yet.
    I2CManager.write(_I2CAddress, 2, REG_GPPU, _portPullup & _portInUse);  
  }
  void _writePortModes() override {
    // Write 0 to IODIR for in-use pins that are outputs, 1 for others.
    uint8_t temp = ~(_portMode & _portInUse);
    I2CManager.write(_I2CAddress, 2, REG_IODIR, temp);
    // Enable interrupt-on-change for in-use pins that are inputs (_portMode=0)
    temp = ~_portMode & _portInUse;
    I2CManager.write(_I2CAddress, 2, REG_INTCON, 0x00);
    I2CManager.write(_I2CAddress, 2, REG_GPINTEN, temp);
  }
  void _readGpioPort(bool immediate) override {
    if (immediate) {
      uint8_t buffer;
      I2CManager.read(_I2CAddress, &buffer, 1, 1, REG_GPIO);
      _portInputState = buffer | _portMode;
    } else {
      // Queue new request
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
  void _setupDevice() override {
    // IOCON is set ODR=1 (open drain shared interrupt pin), INTPOL=0 (active-Low)
    I2CManager.write(_I2CAddress, 2, REG_IOCON, 0x04);
    _writePortModes();
    _writePullups();
    _writeGpioPort();
  }
 
  uint8_t inputBuffer[1];
  uint8_t outputBuffer[1];

  enum {
    // Register definitions for MCP23008
    REG_IODIR=0x00,
    REG_GPINTEN=0x02,
    REG_INTCON=0x04,
    REG_IOCON=0x05,
    REG_GPPU=0x06,
    REG_GPIO=0x09,
  };

};

#endif