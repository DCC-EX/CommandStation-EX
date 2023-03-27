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
private:
  // Here we define the device-specific variables.  
  HardwareSerial *_serial;
  uint8_t _inputState = 0;
  int _inputIndex = 0;
  int _inputValue = 0;
  uint16_t *_pinValues; // Pointer to block of memory containing pin values
  unsigned long _baud;

public:
  //  Static function to handle "IO_ExampleSerial::create(...)" calls.
  static void create(VPIN firstVpin, int nPins, HardwareSerial *serial, unsigned long baud) {
    if (checkNoOverlap(firstVpin,nPins)) new IO_ExampleSerial(firstVpin, nPins, serial, baud);
  } 

protected:
  // Constructor.  This should initialise variables etc. but not call other objects yet
  // (e.g. Serial, I2CManager, and other parts of the CS functionality).
  // defer those until the _begin() function.  The 'addDevice' call is required unless
  // the device is not to be added (e.g. because of incorrect parameters).
  IO_ExampleSerial(VPIN firstVpin, int nPins, HardwareSerial *serial, unsigned long baud) {
    _firstVpin = firstVpin;
    _nPins = nPins;
    _pinValues = (uint16_t *)calloc(_nPins, sizeof(uint16_t));
    _baud = baud;
    
    // Save reference to serial port driver
    _serial = serial;

    addDevice(this);
  }

  // Device-specific initialisation
  void _begin() override {
    _serial->begin(_baud);
#if defined(DIAG_IO)
    _display();
#endif

    // Send a few # characters to the output
    for (uint8_t i=0; i<3; i++)
      _serial->write('#');
  }
  
  // Device-specific write function.  Write a string in the form "#Wm,n#"
  //  where m is the vpin number, and n is the value.
  void _write(VPIN vpin, int value) {
    int pin = vpin -_firstVpin;
    #ifdef DIAG_IO
    DIAG(F("IO_ExampleSerial::_write VPIN:%u Value:%d"), (int)vpin, value);
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
  int _read(VPIN vpin) {

    // Return a value for the specified vpin.
    int result = _pinValues[vpin-_firstVpin];

    return result;
  }

  // Loop function to do background scanning of the input port.  State 
  //  machine parses the incoming command as it is received.  Command
  //  is in the form "#Nm,n#" where m is the index and n is the value.
  void _loop(unsigned long currentMicros) {
    (void)currentMicros;  // Suppress compiler warnings
    if (_serial->available()) {
      // Input data available to read.  Read a character.
      char c = _serial->read();
      switch (_inputState) {
        case 0: // Waiting for start of command
          if (c == '#')  // Start of command received.
            _inputState = 1;
          break;
        case 1: // Expecting command character
          if (c == 'N') { // 'Notify' character received
            _inputState = 2;
            _inputValue = _inputIndex = 0;
          } else
            _inputState = 0; // Unexpected char, reset
          break;
        case 2: // reading first parameter (index)
          if (isdigit(c))
            _inputIndex = _inputIndex * 10 + (c-'0');
          else if (c==',') 
            _inputState = 3;
          else
            _inputState = 0; // Unexpected char, reset
          break;
        case 3: // reading reading second parameter (value)
          if (isdigit(c)) 
            _inputValue = _inputValue * 10 - (c-'0');
          else if (c=='#') { // End of command
            // Complete command received, do something with it.
            DIAG(F("ExampleSerial Received command, p1=%d, p2=%d"), _inputIndex, _inputValue);
            if (_inputIndex >= 0 && _inputIndex < _nPins) { // Store value
              _pinValues[_inputIndex] = _inputValue;
            }
            _inputState = 0; // Done, start again.
          } else
            _inputState = 0; // Unexpected char, reset
          break;
      }
    }
  }

  // Display information about the device, and perhaps its current condition (e.g. active, disabled etc).
  // Here we display the current values held for the pins.
  void _display() {
    DIAG(F("IO_ExampleSerial Configured on Vpins:%u-%u"), (int)_firstVpin, 
      (int)_firstVpin+_nPins-1);
    for (int i=0; i<_nPins; i++)
      DIAG(F("  VPin %2u: %d"), _firstVpin+i, _pinValues[i]);
  }


};

#endif // IO_EXAMPLESERIAL_H