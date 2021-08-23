/*
 *  © 2021, Neil McKechnie. All rights reserved.
 *  
 *  This file is part of DCC++EX API
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
 * To declare a device instance, 
 *    IO_ExampleSerial myDevice(1000, 10, Serial3, 9600);
 * or to create programmatically,
 *    IO_ExampleSerial::create(1000, 10, Serial3, 9600);
 * 
 * (uses VPINs 1000-1009, talke on Serial 3 at 9600 baud.)
 * 
 * See IO_ExampleSerial.cpp for the protocol used over the serial line.
 * 
 */

#ifndef IO_EXAMPLESERIAL_H
#define IO_EXAMPLESERIAL_H

#include "IODevice.h"

class IO_ExampleSerial : public IODevice {
public:
  IO_ExampleSerial(VPIN firstVpin, int nPins, HardwareSerial *serial, unsigned long baud);
  static void create(VPIN firstVpin, int nPins, HardwareSerial *serial, unsigned long baud);  

protected:
  void _begin() override;
  void _loop(unsigned long currentMicros) override;
  void _write(VPIN vpin, int value) override;
  int _read(VPIN vpin) override;
  void _display() override;

private:
  HardwareSerial *_serial;
  uint8_t _inputState = 0;
  int _inputIndex = 0;
  int _inputValue = 0;
  uint16_t *_pinValues; // Pointer to block of memory containing pin values
  unsigned long _baud;
};

#endif // IO_EXAMPLESERIAL_H