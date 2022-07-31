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

#ifndef IO_RotaryEncoder_h
#define IO_RotaryEncoder_h

#include "IODevice.h"
#include "I2CManager.h"
#include "DIAG.h"

class RotaryEncoder : public IODevice {
public:
  static void create(VPIN firstVpin, uint8_t I2CAddress);
  // Constructor
  RotaryEncoder(VPIN firstVpin, uint8_t I2CAddress);

private:
  // Device specific read function
  void _begin() override;
  void _loop(unsigned long currentMicros) override;
  int _read(VPIN vpin) override;
  void _display() override;
  uint8_t _I2CAddress;

};

#endif
