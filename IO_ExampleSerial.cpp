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

#include <Arduino.h>
#include "IO_ExampleSerial.h"
#include "FSH.h"

// Constructor
IO_ExampleSerial::IO_ExampleSerial(VPIN firstVpin, int nPins, HardwareSerial *serial, unsigned long baud) {
  _firstVpin = firstVpin;
  _nPins = nPins;
  
  // Save reference to serial port driver
  _serial = serial;
  _serial->begin(baud);
  DIAG(F("ExampleSerial configured Vpins:%d-%d"), _firstVpin, _firstVpin+_nPins-1);
}

// Static create method for one module.
void IO_ExampleSerial::create(VPIN firstVpin, int nPins, HardwareSerial *serial, unsigned long baud) {
  IO_ExampleSerial *dev = new IO_ExampleSerial(firstVpin, nPins, serial, baud);
  addDevice(dev);
}

// Device-specific initialisation
void IO_ExampleSerial::_begin() {
  // Send a few # characters to the output
  for (uint8_t i=0; i<3; i++)
    _serial->write('#');
}

// Device-specific write function.  Write a string in the form "#Wm,n#"
//  where m is the vpin number, and n is the value.
void IO_ExampleSerial::_write(VPIN vpin, int value) {
  int pin = vpin -_firstVpin;
  #ifdef DIAG_IO
  DIAG(F("IO_ExampleSerial::_write Pin:%d Value:%d"), (int)vpin, value);
  #endif
  // Send a command string over the serial line
  _serial->print('#');
  _serial->print('W');
  _serial->print(pin);
  _serial->print(',');
  _serial->print(value);
  _serial->println('#');
  DIAG(F("ExampleSerial Sent command, p1=%d, p2=%d"), vpin, value);
 }

// Device-specific read function.
int IO_ExampleSerial::_read(VPIN vpin) {

  // Return a value for the specified vpin.  For illustration, return 
  // a value indicating whether the pin number is odd.
  int result = (vpin & 1);

  return result;
}

// Loop function to do background scanning of the input port.  State 
//  machine parses the incoming command as it is received.  Command
//  is in the form "#Nm,n#" where m is the index and n is the value.
void IO_ExampleSerial::_loop(unsigned long currentMicros) {
  (void)currentMicros;  // Suppress compiler warnings
  if (_serial->available()) {
    // Input data available to read.  Read a character.
    char c = _serial->read();
    switch (inputState) {
      case 0: // Waiting for start of command
        if (c == '#')  // Start of command received.
          inputState = 1;
        break;
      case 1: // Expecting command character
        if (c == 'N') { // 'Notify' character received
          inputState = 2;
          inputValue = inputIndex = 0;
        } else
          inputState = 0; // Unexpected char, reset
        break;
      case 2: // reading first parameter (index)
        if (isdigit(c))
          inputIndex = inputIndex * 10 + (c-'0');
        else if (c==',') 
          inputState = 3;
        else
          inputState = 0; // Unexpected char, reset
        break;
      case 3: // reading reading second parameter (value)
        if (isdigit(c)) 
          inputValue = inputValue * 10 - (c-'0');
        else if (c=='#') { // End of command
          // Complete command received, do something with it.
          DIAG(F("ExampleSerial Received command, p1=%d, p2=%d"), inputIndex, inputValue);
          inputState = 0; // Done, start again.
        } else
          inputState = 0; // Unexpected char, reset
        break;
    }
  }
}

void IO_ExampleSerial::_display() {
  DIAG(F("IO_ExampleSerial VPins:%d-%d"), (int)_firstVpin, 
    (int)_firstVpin+_nPins-1);
}

