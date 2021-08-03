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

#ifndef IO_EXAMPLESERIAL_H
#define IO_EXAMPLESERIAL_H

#include "IODevice.h"

class IO_ExampleSerial : public IODevice {
public:
  IO_ExampleSerial(VPIN firstVpin, int nPins, HardwareSerial *serial, unsigned long baud);
  static void create(VPIN firstVpin, int nPins, HardwareSerial *serial, unsigned long baud);  

  void _begin() override;
  void _loop(unsigned long currentMicros) override;
  void _write(VPIN vpin, int value) override;
  int _read(VPIN vpin) override;
  void _display() override;

private:
  HardwareSerial *_serial;
  uint8_t inputState = 0;
  int inputIndex = 0;
  int inputValue = 0;
};

#endif // IO_EXAMPLESERIAL_H