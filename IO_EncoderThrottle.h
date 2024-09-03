/*
 *  © 2024, Chris Harlow. All rights reserved.
 *
 *  This file is part of EX-CommandStation
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
* The IO_EncoderThrottle device driver uses a rotary encoder connected to vpins
* to drive a loco.
*  Loco id is selected by writeAnalog.
*/

#ifndef IO_EncoderThrottle_H
#define IO_EncoderThrottle_H
#include "IODevice.h"

class EncoderThrottle : public IODevice {
public:
  
  static void create(VPIN firstVpin, int dtPin, int clkPin, int clickPin, byte notch=10);
 
private:
  int _dtPin,_clkPin,_clickPin, _locoid, _notch,_prevpinstate; 
  enum {xrSTOP,xrFWD,xrREV} _stopState;
  byte _rocoState;  

  // Constructor
  EncoderThrottle(VPIN firstVpin, int dtPin, int clkPin, int clickPin, byte notch);
  
  void _loop(unsigned long currentMicros) override ;

  // Selocoid as analog value to start drive
  // use <z vpin locoid [notch]>
  void _writeAnalogue(VPIN vpin, int value, uint8_t param1, uint16_t param2) override;
  
  void _display() override ;

 };

#endif
