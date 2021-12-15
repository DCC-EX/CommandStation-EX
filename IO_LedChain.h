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
 * This driver is designed for the LED Driver devices which are characteristically
 * driven with DATAIN, CLOCK and LATCH signals, and chained one to another by connecting
 * the DATAOUT pin of one device to the next device's DATAIN pin.  The devices act like a
 * shift register, so the data bits are sent to the first device's DATAIN pin and clocked through
 * the shift register (bit by bit).  Once the shift register is loaded, the data is latched
 * into the devices when the LATCH signal is pulsed.
 * 
 * Some devices drive on/off outputs, so one bit is written to the shift register for each 
 * output to be driven.  For example, with 16 x 1-bit device, 16 bits are sent, one for each 
 * LED output.
 * 
 * Other devices allow the outputs to be driven by grey-scale, with up to 16 bit resolution.
 * In this case, the number of bits sent to the shift register increases drastically; for 
 * 12 LEDs at 16-bit resolution, a total of 172 bits must be sent for each device in the chain.
 * for 24 LEDs at 12-bit resolution, 268 bits must be sent for each device.
 * The most significant bit of each value is sent first.
 * 
 * RGB LEDs may be driven by connecting an output to each of the R/G/B wires of the LED, but
 * most of the driver devices sink current to ground through the LED, so an RGB LED with a 
 * common anode (+ve terminal) must be used with these devices.  Check the datasheet for details.
 * 
 * Device variants known:
 *  TLC5947:    24 x 12-bit
 *  TLC5940:    16 x 12-bit
 *  TLC6C598:   8 x 1-bit
 *  TLC6C5912:  12 x 1-bit
 *  STP24DP05:  8 x 3-bit (RGB)
 *  MAX6979:    16 x 1-bit
 *  74HC595:    8 x 1-bit
 * 
 * All of these devices are able to support clock pulses of 30ns or shorter with a clock rate of
 * 10MHz or faster; however, the Arduino isn't capable of running this fast, and the shortest pulse 
 * length that is generated is 750ns, and a peak clock rate of 186kHz.  Faster rates could be
 * achieved by using the SPI interface, but if any other SPI device is in use then additional
 * device select circuitry would be required.
 * 
 */

#ifndef IO_LEDCHAIN_H
#define IO_LEDCHAIN_H

#include "IODevice.h"

class LedChain : public IODevice {

private:

#ifdef ARDUINO_ARCH_AVR
  class DigPin {
    private:
      volatile uint8_t *ptr;
      uint8_t mask;
    public:
      DigPin() { ptr = &mask; }
      DigPin(int pinNumber) {
        if (pinNumber >= 0 && pinNumber <= NUM_DIGITAL_PINS) {
          int port = digitalPinToPort(pinNumber);
          if (port != NOT_A_PORT) {
            pinMode(pinNumber, OUTPUT);
            mask = digitalPinToBitMask(pinNumber);
            ptr = portOutputRegister(port);
            return;
          }
        }
        // Pin not valid, set pointer to somewhere benign
        ptr = &mask;
      }
      void setValue(bool value) {
        noInterrupts();
        if (value) 
          *ptr |= mask;
        else
          *ptr &= ~mask;
        interrupts();
      }
      void pulse() {
        noInterrupts();
        *ptr |= mask;
        *ptr &= ~mask;
        interrupts();
      }
  };
#else
  // Fall back to digitalWrite calls.
  class DigPin {
    private: 
      int _pinNumber = 0;
    public:
      DigPin() {};
      DigPin(int pinNumber) {
        pinMode(pinNumber, OUTPUT);
        _pinNumber = pinNumber;
      }
      void setValue(bool value) {
        digitalWrite(_pinNumber, value);
      }
      void pulse() {
        digitalWrite(_pinNumber, 1);
        digitalWrite(_pinNumber, 0);
      }
  };
#endif

  // pins must be arduino GPIO pins, not extender pins or HAL pins.
  DigPin _dataPin;
  DigPin _clockPin;
  DigPin _latchPin;
  int _bitsPerPin = 0;
  byte *_values;
  bool _changed = true;
  int _nextRegisterPin = 0;
  unsigned long _lastEntryTime = 0;

public:
  // Constructor performs static initialisation of the device object
  LedChain (VPIN vpin, int nPins, int dataPin, int clockPin, int latchPin, int bitsPerPin=1) {
    _firstVpin = vpin;
    _nPins = nPins;
    _dataPin = DigPin(dataPin);
    _clockPin = DigPin(clockPin);
    _latchPin = DigPin(latchPin);
    _bitsPerPin = bitsPerPin;
    if (_bitsPerPin == 1)
      _values = (byte *)calloc((_nPins+7)/8, 1);  // 1 byte per 8 pins (rounded up)
    else
      _values = (byte *)calloc(_nPins, 2);   // 2 bytes per pin.
    addDevice(this);
  }

  // Static create function provides alternative way to create object
  static void create(VPIN vpin, int nPins, int dataPin, int clockPin, int latchPin, int bitsPerPin=1) {
    new LedChain (vpin, nPins, dataPin, clockPin, latchPin, bitsPerPin);
  }

protected:
  // _begin function called to perform dynamic initialisation of the device
  void _begin() override {
    _dataPin.setValue(0);
    _clockPin.setValue(0);
    _latchPin.setValue(0);
#if defined(DIAG_IO)
    _display();
#endif
  }

  // Digital write - write on or off.
  void _write(VPIN vpin, int value) {
    if (_bitsPerPin == 1) {
      int pin = vpin - _firstVpin;
      uint8_t *ptr = _values + pin/8;
      uint8_t mask = 1 << (pin % 8);
      if (value) 
        *ptr |= mask;
      else
        *ptr &= ~mask;
    } else {
      // Write maximum positive value (will be truncated if too large)
      writeAnalogue(vpin, value ? 0x7fff : 0);
    }
    _changed = true;
  }

  // Analogue write - write the supplied value
  void _writeAnalogue(VPIN vpin, int value) {
    if (_bitsPerPin == 1 ) {
      _write(vpin, value);
    } else {
      int pin = vpin - _firstVpin;
      uint16_t *ptr = (uint16_t *)(_values + pin*2);
      *ptr = value;
    }
    _changed = true;
  }

  // _loop function - refresh device every 100ms if anything has changed.
  void _loop(unsigned long currentMicros) override {
    int count = 0;
    if (_changed) {
      // Remember the time that this output cycle started.
      if (_nextRegisterPin == 0) _lastEntryTime = currentMicros;
      if (_bitsPerPin == 1) {
        int pin=_nextRegisterPin;
        uint8_t *ptr = _values + pin/8;
        uint8_t mask = 1;
        while (true) {
          // For each pin, write one bit to the shift register
          uint8_t value = (*ptr & mask) ? 1 : 0;
          _dataPin.setValue(value);
          _clockPin.pulse();
          mask <<= 1;
          if (mask == 0) {
            if (++count >= 2) {  // max of 16 pins per loop entry
              _nextRegisterPin = pin;
              return;  // Resume on next loop entry
            }
            // Move to next byte
            ptr++;
            mask = 1;
          }
          if (++pin >= _nPins) break;
        }
      } else {
        // Multiple bits per pin - up to 16 bits stored in two bytes.
        int pin=_nextRegisterPin;
        uint16_t *ptr = (uint16_t *)_values + pin;
        while (true) {
          uint16_t value = *ptr++;
          // For each pin, write the requisite number of bits to the shift register
          uint16_t mask = 1 << (_bitsPerPin-1);
          while (mask) {
            _dataPin.setValue((value & mask) ? 1 : 0);
            _clockPin.pulse();
            mask >>= 1;
          }
          if (++pin >= _nPins) break; // finished.
          if (++count >= 1) {   // max of 1 pin per loop entry
            _nextRegisterPin = pin;
            return;  // Resume on next loop entry
          }
        }
      }
      // Pulse latch pin to transfer data from shift register to outputs.
      _latchPin.pulse();
      //_changed = false;
    }
    _nextRegisterPin = 0; // Restart from the beginning on next entry
    delayUntil(_lastEntryTime+100000UL);  // At most one update cycle per 100ms
  }

  void _display() override {
    DIAG(F("LedChain Configured on Vpins:%d-%d DataPin:%d ClockPin:%d LatchPin:%d BitsPerOutput:%d"),
      _firstVpin, _firstVpin+_nPins-1, _dataPin, _clockPin, _latchPin, _bitsPerPin);
  }

};
#endif //IO_LEDCHAIN_H