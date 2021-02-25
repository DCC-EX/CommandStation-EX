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

#ifndef I2CManager_h
#define I2CManager_h

#include <Wire.h>

/* 
 * Helper class to manage access to the I2C 'Wire' subsystem.
 * 
 * Helps to avoid calling Wire.begin() multiple times (which is not)
 * entirely benign as it reinitialises).
 * 
 * Also helps to avoid the Wire clock from being set, by another device
 * driver, to a speed which is higher than a device supports.
 * 
 * Thirdly, it provides a convenient way to check whether there is a 
 * device on a particular I2C address.
 */

class I2CManagerClass {

public:

  I2CManagerClass() {}

  // If not already initialised, initialise I2C (wire).
  void begin(void);
  // Set clock speed to the lowest requested one.
  void setClock(uint32_t speed);
  // Check if specified I2C address is responding.
  uint8_t exists(uint8_t address);

private:
  bool _beginCompleted = false;
  uint32_t _clockSpeed = 1000000L; // 1MHz max on Arduino.
};

extern I2CManagerClass I2CManager;

#endif