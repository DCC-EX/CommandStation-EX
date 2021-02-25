/*
 *  © 2021, Neil McKechnie. All rights reserved.
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

#include "I2CManager.h"

// If not already initialised, initialise I2C (wire).
void I2CManagerClass::begin(void) {
  if (!_beginCompleted) {
    Wire.begin();
    _beginCompleted = true;
  }
}

// Set clock speed to the lowest requested one. If none requested,
//  the Wire default is 100kHz.
void I2CManagerClass::setClock(uint32_t speed) {
  if (speed < _clockSpeed) {
    _clockSpeed = speed;
    Wire.setClock(_clockSpeed);
  }
}

// Check if specified I2C address is responding.
// Returns 0 if OK, or error code.
uint8_t I2CManagerClass::exists(uint8_t address) {
  begin();
  Wire.beginTransmission(address);
  return Wire.endTransmission();
}

I2CManagerClass I2CManager = I2CManagerClass();