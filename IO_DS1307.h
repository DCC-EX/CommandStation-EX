/*
 *  © 2024, Chris Harlow. All rights reserved.
 *
 *  This file is part of CommandStation-EX
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
* The IO_DS1307 device driver is used to interface a standalone realtime clock. 
* The clock will announce every minute (which will trigger EXRAIL ONTIME events).
* Seconds, and Day/date info is ignored, except that the announced hhmm time
* will attempt to synchronize with the 0 seconds of the clock. 
* An analog read in EXRAIL (IFGTE(vpin, value) etc will check against the hh*60+mm time.
* The clock can be easily set by an analog write to the vpin using 24 hr clock time
* with the command <z vpin hh mm ss> 
*/

#ifndef IO_DS1307_h
#define IO_DS1307_h


#include "IODevice.h"

class DS1307 : public IODevice {
public: 
  static const bool debug=false; 
  static void create(VPIN vpin, I2CAddress i2cAddress);
 
    
private:
  
  // Constructor
    DS1307(VPIN vpin,I2CAddress i2cAddress);
    uint32_t getTime();
    void _begin() override;
    void _display() override;
    void _loop(unsigned long currentMicros) override;
    int _readAnalogue(VPIN vpin) override;
    void _writeAnalogue(VPIN vpin, int hh, uint8_t mm, uint16_t ss)  override;
};
 
#endif
