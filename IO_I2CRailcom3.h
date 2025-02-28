   /*
 *  © 2024, Henk Kruisbrink & Chris Harlow. All rights reserved.
 *  © 2023, Neil McKechnie. All rights reserved.
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
 * 
 * Dec 2023, Added NXP SC16IS752 I2C Dual UART
 * The SC16IS752 has 64 bytes TX & RX FIFO buffer
 * First version without interrupts from I2C UART and only RX/TX are used, interrupts may not be
 * needed as the RX Fifo holds the reply 
 * 
 * Jan 2024, Issue with using both UARTs simultaniously, the secod uart seems to work  but the first transmit 
 * corrupt data. This need more analysis and experimenatation. 
 * Will push this driver to the dev branch with the uart fixed to 0 
 * Both SC16IS750 (single uart) and SC16IS752 (dual uart, but only uart 0 is enable)
 * 
 * myHall.cpp configuration syntax:
 * 
 * I2CRailcom::create(1st vPin, vPins, I2C address);
 * 
 * myAutomation configuration
 *  HAL(I2CRailcom, 1st vPin, vPins, I2C address)
 * Parameters:
 * 1st vPin     : First virtual pin that EX-Rail can control to play a sound, use PLAYSOUND command (alias of ANOUT)
 * vPins        : Total number of virtual pins allocated (2 vPins are supported, one for each UART)
 *                1st vPin for UART 0, 2nd for UART 1
 * I2C Address  : I2C address of the serial controller, in 0x format
 */

#ifndef IO_I2CRailcom3_h
#define IO_I2CRailcom3_h
#include "Arduino.h"
#include "IODevice.h"

// Debug and diagnostic defines, enable too many will result in slowing the driver
#define DIAG_I2CRailcom

class I2CRailcom3 : public IODevice {
private: 
  byte cutoutCounter;
  public:
  // Constructor
  I2CRailcom3(VPIN firstVpin, int nPins, I2CAddress i2cAddress);
  
  static void create(VPIN firstVpin, int nPins, I2CAddress i2cAddress) ;
  
  void _begin() ;
  void _loop(unsigned long currentMicros) override ;
  void _display() override ;
  
private: 


  
};

#endif // IO_I2CRailcom_h
