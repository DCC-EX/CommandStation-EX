/*
 *  © 2026, Terry Wheatcroft & Chris Harlow
 *  All rights reserved.
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

#ifndef io_xl9535_h
#define io_xl9535_h

#include "IO_GPIOBase.h"
#include "FSH.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 * IODevice subclass for XL9535 16-bit I/O expander relay board.
 * Since this board is effectively an output-only board, it has no I2C read code.
 */
 
class XL9535 : public GPIOBase<uint16_t> {
public:
  static void create(VPIN vpin, uint8_t nPins, I2CAddress i2cAddress) {
    if (checkNoOverlap(vpin, nPins, i2cAddress)) new XL9535(vpin,nPins, i2cAddress);
  }

private:  
  // Constructor
  XL9535(VPIN vpin, uint8_t nPins, I2CAddress I2CAddress) 
    : GPIOBase<uint16_t>((FSH *)F("XL9535"), vpin, nPins, I2CAddress,-1) 
  {
  }

  void _writeGpioPort() override {
    I2CManager.write(_I2CAddress, 3, REG_OUTPUT_P0, _portOutputState, _portOutputState>>8);
  }

  void _writePullups() override {}

  void _writePortModes() override {
    // Write 0 to REG_CONF_P0 & REG_CONF_P1 as all pins are output
    I2CManager.write(_I2CAddress, 3, REG_CONF_P0, 0, 0);    
  }

  void _readGpioPort(bool immediate) override {
    (void) immediate; 
    _portInputState = _portOutputState;
  }

  void _setupDevice() override {
    // HAL API calls
    _writeGpioPort();
  }
 
  
  enum {
    REG_OUTPUT_P0 = 0x02,
    REG_OUTPUT_P1 = 0x03,
    REG_POL_INV_P0 = 0x04,
    REG_POL_INV_P1 = 0x05,
    REG_CONF_P0 = 0x06,
    REG_CONF_P1 = 0x07,    
  };

};

#endif
