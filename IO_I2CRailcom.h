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
 * This polls the RailcomCollecter device once per dcc packet
 * and obtains an abbreviated list of block occupancy changes which
 * are fortunately very rare compared with Railcom raw data. 
 *
 * myAutomation configuration
 *  HAL(I2CRailcom, 1st vPin, vPins, I2C address)
 * Parameters:
 * 1st vPin     : First virtual pin that EX-Rail can control to play a sound, use PLAYSOUND command (alias of ANOUT)
 * vPins        : Total number of virtual pins allocated 
 * I2C Address  : I2C address of the Railcom Collector, in 0x format
 */

#ifndef IO_I2CRailcom_h
#define IO_I2CRailcom_h
#include "Arduino.h"
#include "IODevice.h"

class I2CRailcom : public IODevice {
private: 
  byte cutoutCounter;
  public:
  // Constructor
  I2CRailcom(VPIN firstVpin, int nPins, I2CAddress i2cAddress);
  
  static void create(VPIN firstVpin, int nPins, I2CAddress i2cAddress) ;
  
  void _begin() ;
  void _loop(unsigned long currentMicros) override ;
  void _display() override ;
  
private: 


  
};

#endif // IO_I2CRailcom_h
