/*
 * © 2026, Nicola Malavasi (NicMal). All rights reserved.
 * © 2025-26, Chris Harlow. All rights reserved.
 * © 2023, Neil McKechnie. All rights reserved.
 * * This file is part of DCC++EX API
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
 * This file defines the Abstract Base Class (DFPlayerBase) for DFPlayer devices.
 * It manages the high-level logic, state machine, and command queuing, 
 * independent of the physical transport layer.
 * * KEY FEATURES:
 * 1. Command Queue (FIFO): Manages a 4-slot queue to buffer commands, ensuring 
 * a 120ms delay between transmissions to prevent module lock-ups.
 * 2. Protocol Handling: Automatically calculates the 16-bit checksum and 
 * constructs the 10-byte serial frames required by the DFPlayer hardware.
 * 3. State Management: Tracks playback status (_playing), current volume, 
 * active folder, and loop flags.
 * 4. Bidirectional Feedback: Parses incoming bytes from the module to detect 
 * events like "Track Finished" (0x3D), enabling WAITFOR() synchronization.
 * 5. EX-RAIL Integration: Maps DCC++EX internal opcodes (DF_PLAY, DF_VOL, etc.) 
 * to physical hardware commands via the _writeAnalogue override.
 */




#ifndef IO_DFPlayer_h
#define IO_DFPlayer_h
#include "IO_DFPlayerSerial.h"
#include "IO_DFPlayerI2C.h"

class DFPlayer : public IODevice {
public:
  static void create(VPIN firstVpin, uint8_t nPins, I2CAddress i2cAddress, uint8_t xtal) {
    if (nPins>2) nPins=2;
    if (checkNoOverlap(firstVpin, nPins, i2cAddress)) {
      // Istanza Canale A
      new DFPlayerI2C(firstVpin, i2cAddress, xtal, 0);
      if (nPins >= 2) {
        // Istanza Canale B
        new DFPlayerI2C(firstVpin + 1, i2cAddress, xtal, 1);
      }
    }
  }

  #ifdef ESP32
  // ESP32 user must provide serial pins
  static void create(VPIN f, HardwareSerial &s,
    int8_t rxPin, int8_t txPin) { 
      create(f, 1, s,rxPin,txPin);
     }
  
  static void create(VPIN firstVpin, int nPins, HardwareSerial &serial,
    int8_t rxPin, int8_t txPin) {
    if (checkNoOverlap(firstVpin,nPins)) {
      serial.begin(9600, SERIAL_8N1,rxPin,txPin); // 9600baud, no parity, 1 stop bit
      new DFPlayerSerial(firstVpin, nPins, serial);
    }
  }
 #else
  // NON-ESP32 knows about serial pins
  static void create(VPIN f, HardwareSerial &s) { create(f, 1, s); }
  static void create(VPIN firstVpin, int nPins, HardwareSerial &serial) {
    if (checkNoOverlap(firstVpin,nPins)) {
      serial.begin(9600, SERIAL_8N1); // 9600baud, no parity, 1 stop bit
      new DFPlayerSerial(firstVpin, nPins, serial);
    }
  }
  #endif

};
#endif