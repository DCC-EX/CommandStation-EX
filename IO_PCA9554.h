/*
 *  © 2025, Paul M. Antoine
 *  © 2021, Neil McKechnie. All rights reserved.
 *  
 *  This file is part of DCC-EX API
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

#ifndef io_pca9554_h
#define io_pca9554_h

#include "IO_GPIOBase.h"
#include "FSH.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 * IODevice subclass for PCA9554/TCA9554 8-bit I/O expander (NXP & Texas Instruments).
 */
 
class PCA9554 : public GPIOBase<uint8_t> {
public:
  static void create(VPIN vpin, uint8_t nPins, I2CAddress i2cAddress, int interruptPin=-1) {
    if (checkNoOverlap(vpin, nPins, i2cAddress)) new PCA9554(vpin,nPins, i2cAddress, interruptPin);
  }

private:  
  // Constructor
  PCA9554(VPIN vpin, uint8_t nPins, I2CAddress I2CAddress, int interruptPin=-1) 
    : GPIOBase<uint8_t>((FSH *)F("PCA9554"), vpin, nPins, I2CAddress, interruptPin) 
  {
    requestBlock.setRequestParams(_I2CAddress, inputBuffer, sizeof(inputBuffer),
      outputBuffer, sizeof(outputBuffer));
    outputBuffer[0] = REG_INPUT_P0;
  }
  void _writeGpioPort() override {
    I2CManager.write(_I2CAddress, 2, REG_OUTPUT_P0, _portOutputState);
  }
  void _writePullups() override {
    // Do nothing, pull-ups are always in place for input ports
    // This function is here for HAL GPIOBase API compatibilitiy
      
  }
  void _writePortModes() override {
    // Write 0 to REG_CONF_P0 for in-use pins that are outputs, 1 for others.
    // PCA9554 & TCA9554, Interrupt is always enabled for raising and falling edge
    uint8_t temp = ~(_portMode & _portInUse);
    I2CManager.write(_I2CAddress, 2, REG_CONF_P0, temp);    
  }
  void _readGpioPort(bool immediate) override {
    if (immediate) {
      uint8_t buffer[1];
      I2CManager.read(_I2CAddress, buffer, 1, 1, REG_INPUT_P0);
      _portInputState = buffer[0];
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
      _portInputState = inputBuffer[0];
    else  
      _portInputState = 0xff;
  }

  void _setupDevice() override {
    // HAL API calls
    _writePortModes();
    _writePullups();
    _writeGpioPort();
  }
 
  uint8_t inputBuffer[1];
  uint8_t outputBuffer[1];
 

  enum {
    REG_INPUT_P0 = 0x00,
    REG_OUTPUT_P0 = 0x01,
    REG_POL_INV_P0 = 0x02,
    REG_CONF_P0 = 0x03,
  };

};

#endif
