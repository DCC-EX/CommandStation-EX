/*
 *  © 2021, Peter Cole. All rights reserved.
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
* The IO_RotaryEncoder device driver is used to receive positions from a rotary encoder connected to an Arduino via I2C.
*
* There is separate code required for the Arduino the rotary encoder is connected to, which is located here:
* *TBA*
*
* This device driver receives the rotary encoder position when the rotary encoder button is pushed, and these positions
* can be tested in EX-RAIL with:
* ATRE(vpin, position) - trigger for when the specified rotary encoder position is received
* IFRE(vpin, position) - test to see if specified rotary encoder position has been received
*
* Refer to the documentation for further information including the valid activities.
*/

#ifndef IO_ROTARYENCODER_H
#define IO_ROTARYENCODER_H

#include "IODevice.h"
#include "I2CManager.h"
#include "DIAG.h"

class RotaryEncoder : public IODevice {
public:
  static void create(VPIN firstVpin, uint8_t I2CAddress) {
    new RotaryEncoder(firstVpin, I2CAddress);
  }
  // Constructor
  RotaryEncoder(VPIN firstVpin, uint8_t I2CAddress){
    _firstVpin = firstVpin;
    _I2CAddress = I2CAddress;
    addDevice(this);
  }

private:
  VPIN _firstVpin;
  uint8_t _I2CAddress;
  int8_t _position;

  // Device specific read function
  void _begin() {
    I2CManager.begin();
    I2CManager.setClock(1000000);
    if (I2CManager.exists(_I2CAddress)) {
#ifdef DIAG_IO
      _display();
#endif
    } else {
        _deviceState = DEVSTATE_FAILED;
    }
  }

  void _loop(unsigned long currentMicros)  {
    uint8_t readBuffer[1];
    I2CManager.read(_I2CAddress, readBuffer, 1);
    _position = readBuffer[0];
  }

  int _read(VPIN vpin) override {
    if (_deviceState == DEVSTATE_FAILED) return 0;
#ifdef DIAG_IO
    DIAG(F("Received position %d"), _position);
#endif
    return _position;
  }
  
  void _display() {
    DIAG(F("Rotary Encoder I2C:x%x Configured on Vpin:%d %S"), _I2CAddress, (int)_firstVpin, 
      (_deviceState==DEVSTATE_FAILED) ? F("OFFLINE") : F(""));
  }

};

#endif
