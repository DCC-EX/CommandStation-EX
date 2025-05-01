/*
 *  © 2025, Chris Harlow. All rights reserved.
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
#ifndef IO_Bitmap_h
 #define IO_Bitmap_h
#include <Arduino.h>
#include "defines.h"
#include "IODevice.h"

/* 
Bitmap provides a set of virtual pins with no hardware.
Bitmap pins are able to be output and input and may be set and tested 
as digital or analogue values.
When writing a digital value, the analogue value is set to 0 or 1. 
When reading a digital value, the return is LOW for value 0 or HIGH for any other value
or analogue.

Bitmap pins may be used for any purpose, this is easier to manage than LATCH in EXRAIL
as they can be explicitely set and tested without interfering with underlying hardware.
Bitmap pins may be set, reset and tested in the same way as any other pin.
They are not persistent across reboots, but are retained in the current session.
Bitmap pins may also be monitored by JMRI_SENSOR() and <S> as for any other pin.
   
*/
class Bitmap : public IODevice {

public:
  static void create(VPIN firstVpin, int nPins) {   
    if (IODevice::checkNoOverlap(firstVpin,nPins))
         new Bitmap( firstVpin,  nPins);
  }

  Bitmap(VPIN firstVpin, int nPins) : IODevice(firstVpin, nPins) {
    _pinValues=(int16_t *) calloc(nPins,sizeof(int16_t));  
    // Connect to HAL so my _write, _read and _loop will be called as required.
    IODevice::addDevice(this);  
  }

// Called by HAL to start handling this device
  void _begin() override {
    _deviceState = DEVSTATE_NORMAL;
    _display();
  }

  int _read(VPIN vpin) override {
    int pin=vpin - _firstVpin;
    return _pinValues[pin]?1:0;
  }

  void _write(VPIN vpin, int value) override {
    int pin = vpin - _firstVpin;
    _pinValues[pin]=value!=0; // this is digital write  
  }

  int _readAnalogue(VPIN vpin) override {
    int pin=vpin - _firstVpin;
    return _pinValues[pin]; // this is analog read  
  }

  void _writeAnalogue(VPIN vpin, int value, uint8_t profile, uint16_t duration) override {
    int pin=vpin - _firstVpin;
    _pinValues[pin]=value; // this is analog write  
  }

  void _display() override {
      DIAG(F("Bitmap Configured on Vpins:%u-%u"), 
      (int)_firstVpin, 
      (int)_firstVpin+_nPins-1);
  }

private:
  int16_t* _pinValues;
};
#endif
