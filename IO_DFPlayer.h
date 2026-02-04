/*
 * © 2026, Nicola Malavasi. All rights reserved.
 * © 2025-26, Chris Harlow. All rights reserved.
 * © 2023, Neil McKechnie. All rights reserved.
 * * This file is part of DCC-EX API
 *
 * This is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * It is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with CommandStation.  If not, see <https://www.gnu.org/licenses/>.
 */

/*
 * This file acts as the main wrapper for the DFPlayer driver suite.
 * It provides a unified interface to create Serial or I2C instances.
 * * KEY FEATURES:
 * 1. Unified Interface: Simplified 'create' method for all transport layers, 
 * making the driver more user-friendly in myHAL.cpp.
 * 2. Plug-and-Play (I2C): Eliminates the need for manual crystal frequency 
 * parameters by leveraging the auto-detection logic in the I2C layer.
 * 3. Platform Abstraction: Automatically handles hardware differences between 
 * standard AVR (Mega) and ESP32 (customizable serial pins).
 * 4. Resource Protection: Prevents VPIN and address conflicts by integrating 
 * with the DCC-EX IODevice registry.
 */


#ifndef IO_DFPlayer_h
#define IO_DFPlayer_h
#include "IO_DFPlayerSerial.h"
#include "IO_DFPlayerI2C.h"

class DFPlayer : public IODevice {
public:
  static void create(VPIN firstVpin, uint8_t nPins, I2CAddress i2cAddress) {
    if (nPins>2) nPins=2;
    if (checkNoOverlap(firstVpin, nPins, i2cAddress)) {
      new DFPlayerI2C(firstVpin, i2cAddress, 0);
      if (nPins >= 2) {
        new DFPlayerI2C(firstVpin + 1, i2cAddress, 1);
      }
    }
  }

  #ifdef ESP32
  static void create(VPIN f, HardwareSerial &s, int8_t rxPin, int8_t txPin) { 
      create(f, 1, s, rxPin, txPin);
  }
  static void create(VPIN firstVpin, int nPins, HardwareSerial &serial, int8_t rxPin, int8_t txPin) {
    if (checkNoOverlap(firstVpin, nPins)) {
      serial.begin(9600, SERIAL_8N1, rxPin, txPin);
      new DFPlayerSerial(firstVpin, nPins, serial);
    }
  }
 #else
  static void create(VPIN f, HardwareSerial &s) { create(f, 1, s); }
  static void create(VPIN firstVpin, int nPins, HardwareSerial &serial) {
    if (checkNoOverlap(firstVpin, nPins)) {
      serial.begin(9600, SERIAL_8N1);
      new DFPlayerSerial(firstVpin, nPins, serial);
    }
  }
  #endif
};
#endif