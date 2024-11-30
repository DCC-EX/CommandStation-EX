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

#include "IO_DS1307.h"
#include "I2CManager.h"
#include "DIAG.h"
#include "CommandDistributor.h"

uint8_t d2b(uint8_t d) {
     return (d >> 4)*10 + (d & 0x0F);  
}

void DS1307::create(VPIN vpin, I2CAddress i2cAddress) {
    if (checkNoOverlap(vpin, 1, i2cAddress)) new DS1307(vpin, i2cAddress);
  }
 
    
  // Constructor
    DS1307::DS1307(VPIN vpin,I2CAddress i2cAddress){
      _firstVpin = vpin;
      _nPins = 1;
     _I2CAddress = i2cAddress;
     addDevice(this);
    }

uint32_t DS1307::getTime() {
    // Obtain ss,mm,hh buffers from device
    uint8_t readBuffer[3];
    const uint8_t writeBuffer[1]={0};

    // address register 0 for read. 
    I2CManager.write(_I2CAddress, writeBuffer, 1);
    if (I2CManager.read(_I2CAddress, readBuffer, 3) != I2C_STATUS_OK) {
       _deviceState=DEVSTATE_FAILED;
       return 0;
    } 
    _deviceState=DEVSTATE_NORMAL;

    if (debug) {
      static const char hexchars[]="0123456789ABCDEF";
      USB_SERIAL.print(F("<*RTC"));
        for (int i=2;i>=0;i--) {
          USB_SERIAL.write(' ');  
          USB_SERIAL.write(hexchars[readBuffer[i]>>4]);
          USB_SERIAL.write(hexchars[readBuffer[i]& 0x0F ]);
        }
        StringFormatter::send(&USB_SERIAL,F(" %d *>\n"),_deviceState);
      }
    
    if (readBuffer[0] & 0x80) {
        _deviceState=DEVSTATE_INITIALISING;
        DIAG(F("DS1307 clock in standby"));
        return 0; // clock is not running
    }
    // convert device format to seconds since midnight
    uint8_t ss=d2b(readBuffer[0]  & 0x7F);
    uint8_t mm=d2b(readBuffer[1]);
    uint8_t hh=d2b(readBuffer[2] & 0x3F);
    return (hh*60ul +mm)*60ul +ss;     
}

void DS1307::_begin()  {
  // Initialise  device and sync loop() to zero seconds
    I2CManager.begin(); 
    auto tstamp=getTime();
    if (_deviceState==DEVSTATE_NORMAL) {
        byte seconds=tstamp%60;
        delayUntil(micros() + ((60-seconds) * 1000000));
    }
    _display(); 
}

// Processing loop to obtain clock time.
// This self-synchronizes to the next minute tickover
void DS1307::_loop(unsigned long currentMicros) { 
    auto time=getTime();
    if (_deviceState==DEVSTATE_NORMAL) {
       byte ss=time%60;
       CommandDistributor::setClockTime(time/60, 1, 1);      
       delayUntil(currentMicros + ((60-ss) * 1000000));  
    }
}


  // Display device driver info.
  void DS1307::_display()  {
    auto tstamp=getTime();
    byte ss=tstamp%60;
    tstamp/=60;
    byte mm=tstamp%60;
    byte hh=tstamp/60;
    DIAG(F("DS1307 on I2C:%s vpin %d %d:%d:%d %S"), 
      _I2CAddress.toString(), _firstVpin,
      hh,mm,ss,
     (_deviceState==DEVSTATE_FAILED) ? F("OFFLINE") : F(""));
  }

  // allow user to set the clock 
  void DS1307::_writeAnalogue(VPIN vpin, int hh, uint8_t mm, uint16_t ss)  {  
    (void) vpin;
    uint8_t writeBuffer[3];
    writeBuffer[0]=1; // write mm,hh first 
    writeBuffer[1]=((mm/10)<<4) + (mm % 10);  
    writeBuffer[2]=((hh/10)<<4) + (hh % 10);  
    I2CManager.write(_I2CAddress, writeBuffer, 3);
    writeBuffer[0]=0; // write ss  
    writeBuffer[1]=((ss/10)<<4) + (ss % 10);  
    I2CManager.write(_I2CAddress, writeBuffer, 2);
    _loop(micros()); // resync with seconds rollover
  }
   
  // Method to read analogue hh*60+mm time 
   int DS1307::_readAnalogue(VPIN vpin) { 
    (void)vpin; 
    return getTime()/60;
  };

