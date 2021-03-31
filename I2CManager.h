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

#include "FSH.h"

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
  // Force clock speed 
  void forceClock(uint32_t speed);
  // Check if specified I2C address is responding.
  uint8_t checkAddress(uint8_t address);
  bool exists(uint8_t address);
  // Write a complete transmission to I2C from an array in RAM
  uint8_t write(uint8_t address, const uint8_t buffer[], uint8_t size);
  // Write a complete transmission to I2C from an array in Flash
  uint8_t write_P(uint8_t address, const uint8_t buffer[], uint8_t size);
  // Write a transmission to I2C from a list of bytes.
  uint8_t write(uint8_t address, int nBytes, ...);
  // Write a command from an array in RAM and read response
  uint8_t read(uint8_t address, uint8_t writeBuffer[], uint8_t writeSize, 
    uint8_t readBuffer[], uint8_t readSize);
  // Write a command from an arbitrary list of bytes and read response
  uint8_t read(uint8_t address, uint8_t readBuffer[], uint8_t readSize, 
    uint8_t writeSize, ...);
  // Write a null command and read the response.
  uint8_t read(uint8_t address, uint8_t readBuffer[], uint8_t readSize);

private:
  bool _beginCompleted = false;
  bool _clockSpeedFixed = false;
  uint32_t _clockSpeed = 400000L;  // 400kHz max on Arduino.
};

extern I2CManagerClass I2CManager;

#endif