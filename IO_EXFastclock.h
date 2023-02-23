/*
 *  © 2022, Colin Murdoch. All rights reserved.
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
* The IO_EXFastclock device driver is used to interface the standalone fast clock and receive time data.
*
* The EX-fastClock code lives in a separate repo (https://github.com/DCC-EX/EX-Fastclock) and contains the clock logic.
*
*
*/

#ifndef IO_EXFastclock_h
#define IO_EXFastclock_h


#include "IODevice.h"
#include "I2CManager.h"
#include "DIAG.h"
#include "EXRAIL2.h"
#include "CommandDistributor.h"

bool FAST_CLOCK_EXISTS = true;

class EXFastClock : public IODevice {
public:
  // Constructor
    EXFastClock(I2CAddress i2cAddress){
    _I2CAddress = i2cAddress;
    addDevice(this);
  }

static void create(I2CAddress i2cAddress) {

  DIAG(F("Checking for Clock"));
  // Start by assuming we will find the clock
  // Check if specified I2C address is responding (blocking operation)
  // Returns I2C_STATUS_OK (0) if OK, or error code.
  uint8_t _checkforclock = I2CManager.checkAddress(i2cAddress);
  DIAG(F("Clock check result - %d"), _checkforclock);
  // XXXX change thistosave2 bytes
  if (_checkforclock == 0) {
      FAST_CLOCK_EXISTS = true;
      //DIAG(F("I2C Fast Clock found at %s"), i2cAddress.toString());
      new EXFastClock(i2cAddress); 
    }
    else {
      FAST_CLOCK_EXISTS = false;
      //DIAG(F("No Fast Clock found"));
      LCD(6,F("CLOCK NOT FOUND"));
    }
    
  }
    
private:
  

// Initialisation of Fastclock
void _begin() override {
  
  if (FAST_CLOCK_EXISTS == true) {
    I2CManager.begin();
    if (I2CManager.exists(_I2CAddress)) {
      _deviceState = DEVSTATE_NORMAL;
      #ifdef DIAG_IO
        _display();
      #endif
    } else {
    _deviceState = DEVSTATE_FAILED;
    //LCD(6,F("CLOCK NOT FOUND")); 
    DIAG(F("Fast Clock Not Found at address %s"), _I2CAddress.toString());
    }
  }
}

// Processing loop to obtain clock time

void _loop(unsigned long currentMicros) override{ 
  
  if (FAST_CLOCK_EXISTS==true) {
      uint8_t readBuffer[3];
      byte a,b;
      #ifdef EXRAIL_ACTIVE
        I2CManager.read(_I2CAddress, readBuffer, 3);
        // XXXX change this to save a few bytes
        a = readBuffer[0];
        b = readBuffer[1];
        //_clocktime = (a << 8) + b;
        //_clockrate = readBuffer[2];

        CommandDistributor::setClockTime(((a << 8) + b), readBuffer[2], 1);
        //setClockTime(int16_t clocktime, int8_t clockrate, byte opt);
        
        // As the minimum clock increment is 2 seconds delay a bit - say 1 sec.
        // Clock interval is 60/ clockspeed i.e 60/b seconds
        delayUntil(currentMicros + ((60/b) * 1000000));  
     
      #endif
    
  }
}

  // Display EX-FastClock device driver info.
  void _display() override {
    DIAG(F("FastCLock on I2C:%s - %S"), _I2CAddress.toString(),  (_deviceState==DEVSTATE_FAILED) ? F("OFFLINE") : F(""));
  }
  
};

#endif
