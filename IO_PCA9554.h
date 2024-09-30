/*
 *  © 2024, Paul M. Antoine
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

#ifndef io_pca9554_h
#define io_pca9554_h

#include "IO_GPIOBase.h"
#include "FSH.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 * IODevice subclass for PCA9554 and TCA9554/9534 8-bit I/O expanders (NXP & Texas Instruments).
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
    if (nPins > 8) {
      DIAG(F("PCA9554 nPins %d larger than allowed!"));
      nPins = 8;
    }
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
    // Write 0 to REG_CONF_P0 for in-use pins that are outputs, 1 for inputs.
    // PCA9554 & TCA9555, Interrupt is always enabled for raising and falling edge
    uint8_t temp = ~(_portMode & _portInUse);
    I2CManager.write(_I2CAddress, 2, REG_CONF_P0, temp);    
  }
  void _readGpioPort(bool immediate) override {
    if (immediate) {
      uint8_t buffer[2];
      I2CManager.read(_I2CAddress, buffer, 1, 1, REG_INPUT_P0);
      _portInputState = buffer[0] | _portMode;
      /* PCA9554 Int bug fix, from PCA9554 datasheet: "must change command byte to something besides 00h 
       * after a Read operation to the PCA9554 device or before reading from 
       * another device"
       * Recommended solution, read from REG_OUTPUT_P0, then do nothing with the received data
       * Issue not seen during testing, uncomment if needed 
       */
      //I2CManager.read(_I2CAddress, buffer, 1, 1, REG_OUTPUT_P0);
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
    {
      if (_portInputState != (inputBuffer[0] | _portMode))
        DIAG(F("PCA9554 inputs changed, value now %x"), inputBuffer[0]);
      _portInputState = inputBuffer[0] | _portMode;
    }
    else  
      _portInputState = 0xff;
  }

  void _setupDevice() override {
    // HAL API calls
    _writePortModes();
    _writePullups();
    _writeGpioPort();
  }
 
  uint8_t inputBuffer[2];
  uint8_t outputBuffer[1];
 

  enum {
    REG_INPUT_P0 = 0x00,
    REG_OUTPUT_P0 = 0x01,
    REG_POL_INV_P0 = 0x02,
    REG_CONF_P0 = 0x03,
  };

};

#endif
