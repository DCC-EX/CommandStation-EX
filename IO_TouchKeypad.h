/*
 *  © 2023, Neil McKechnie. All rights reserved.
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
 * Driver for capacitative touch-pad based on the TTP229-B chip with serial 
 * (not I2C) output.  The touchpad has 16 separate pads in a 4x4 matrix, 
 * numbered 1-16.  The communications with the pad are via a clock signal sent
 * from the controller to the device, and a data signal sent back by the device.
 * The pins clockPin and dataPin must be local pins, not external (GPIO Expander)
 * pins.
 * 
 * To use, 
 *    TouchKeypad::create(firstVpin, 16, clockPin, dataPin);
 * 
 * NOTE: Most of these keypads ship with only 8 pads enabled.  To enable all
 * sixteen pads, locate the area of the board labelled P1 (four pairs of 
 * holes labelled 1 to 4 from the left); solder a jumper link between the pair 
 * labelled 3 (connected to pin TP2 on the chip).  When this link is connected,
 * the pins OUT1 to OUT8 are not used but all sixteen touch pads are operational.
 * 
 * TODO: Allow a list of datapins to be provided so that multiple keypads can
 * be read simultaneously by the one device driver and the one shared clock signal.
 * As it stands, we can configure multiple driver instances, one for each keypad, 
 * and it will work fine.  The clock will be driven to all devices but only one 
 * driver will be reading the responses from its corresponding device at a time.
 */

#ifndef IO_TOUCHKEYPAD_H
#define IO_TOUCHKEYPAD_H

#include "IODevice.h"

class TouchKeypad : public IODevice {
private:
  // Here we define the device-specific variables.  
  uint16_t _inputStates = 0;
  VPIN _clockPin;
  VPIN _dataPin;

public:
  //  Static function to handle create calls.
  static void create(VPIN firstVpin, int nPins, VPIN clockPin, VPIN dataPin) {
    if (checkNoOverlap(firstVpin,nPins)) new TouchKeypad(firstVpin, nPins, clockPin, dataPin);
  } 

protected:
  // Constructor. 
  TouchKeypad(VPIN firstVpin, int nPins, VPIN clockPin, VPIN dataPin) {
    _firstVpin = firstVpin;
    _nPins = (nPins > 16) ? 16 : nPins;  // Maximum of 16 pads per device
    _clockPin = clockPin;
    _dataPin = dataPin;

    addDevice(this);
  }

  // Device-specific initialisation
  void _begin() override {
#if defined(DIAG_IO)
    _display();
#endif
    // Set clock pin as output, initially high, and data pin as input.  
    // Enable pullup on the input so that the default (not connected) state is 
    // 'keypad not pressed'.
    ArduinoPins::fastWriteDigital(_clockPin, 1);
    pinMode(_clockPin, OUTPUT);
    pinMode(_dataPin, INPUT_PULLUP); // Force defined state when no connection
  }
  
  // Device-specific read function.
  int _read(VPIN vpin) {
    if (vpin < _firstVpin || vpin >= _firstVpin + _nPins) return 0;

    // Return a value for the specified vpin.
    return _inputStates & (1<<(vpin-_firstVpin)) ? 1 : 0;
  }

  // Loop function to do background scanning of the keyboard.
  // The TTP229 device requires clock pulses to be sent to it,
  // and the data bits can be read on the rising edge of the clock.
  // By default the clock and data are inverted (active-low).
  // A gap of more  than 2ms is advised between successive read
  // cycles, we wait for 100ms between reads of the keyboard as this
  // provide a good enough response time.
  // Maximum clock frequency is 512kHz, so put a 1us delay
  // between clock transitions.
  //
  void _loop(unsigned long currentMicros) {

    // Clock 16 bits from the device
    uint16_t data = 0, maskBit = 0x01;
    for (uint8_t pad=0; pad<16; pad++) {
      ArduinoPins::fastWriteDigital(_clockPin, 0);
      delayMicroseconds(1);
      ArduinoPins::fastWriteDigital(_clockPin, 1);
      data |= (ArduinoPins::fastReadDigital(_dataPin) ? 0 : maskBit);
      maskBit <<= 1;
      delayMicroseconds(1);
    }
    _inputStates = data;
#ifdef DIAG_IO
    static uint16_t lastData = 0;
    if (data != lastData) DIAG(F("KeyPad: %x"), data);
    lastData = data;
#endif
    delayUntil(currentMicros + 100000); // read every 100ms
  }

  // Display information about the device, and perhaps its current condition (e.g. active, disabled etc).
  void _display() {
    DIAG(F("TouchKeypad Configured on Vpins:%u-%u SCL=%d SDO=%d"), (int)_firstVpin, 
      (int)_firstVpin+_nPins-1, _clockPin, _dataPin);
  }


};

#endif // IO_TOUCHKEYPAD_H