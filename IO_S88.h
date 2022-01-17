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
 * The S88 Bus is essentially a shift register consisting of one or more
 * S88 modules, connected one to another in one long chain.  The register
 * is read by the following steps:
 *   1) The LOAD/PS line goes to HIGH, then the CLK line is pulsed HIGH.  This
 * tells the registers to acquire data from the latched parallel inputs. 2) With
 * LOAD/PS still high, the RESET signal is pulsed HIGH, which clears the
 * parallel input upstream latches. 3) With LOAD/PS LOW, the shift is performed
 * by pulsing the CLK line high.  On each pulse, the data in the shift register
 * is presented, bit by bit, to the DATA-IN line.
 *
 * Example configuration in mySetup.cpp:
 *    #include "IO_S88.h"
 *    void mySetup() {
 *      S88bus::create(2500, 16, 40,41,42,43, 10);
 *    }
 *
 * This creates an S88 bus instance with 16 inputs (VPINs 2500-2515).
 * The LOAD pin is 40, RESET is 41, CLK is 42 and DATA is 43.  These four pins
 * must be local GPIO pins (not on an I/O expander).
 * The last parameter (bitTime=10) means that at least 10us is allowed for
 * reading each bit in the shift register.
 *
 * Depending on the S88 modules being used and the cabling, the timing of the
 * interface may have to be adjusted.  This is done by the bit time parameter.
 * The original S88 concept was based on 4014 shift register devices, which are
 * easily capable of moderate speed transfers and would cope with a bit time
 * around 2us.  However, commercial S88 devices appear to be extremely slow in
 * comparison and may need the order of tens or hundreds of microseconds.  The
 * symptom of a bit time that is too small is that, when you activate one input to
 * the S88 module, the command station sees something other than the correct
 * activation; e.g. no activations, multiple activations, the wrong pin
 * activating etc.
 *
 * _acquireCycleTime determines the minimum time between successive acquire
 * cycles of the inputs on the S88 bus.  Bear in mind that the Sensor code
 * includes anti-bounce logic which means that fleeting state changes may be
 * ignored, so reducing the acquireCycleTime may not have the desired effect.
 * Also, if the combination of bit time and number of pins is large, the
 * specified cycle time may not be achieved.
 *
 */

#ifndef IO_S88_H
#define IO_S88_H

#include "IODevice.h"

class S88bus : public IODevice {

private:
  uint8_t _loadPin;
  uint8_t _resetPin;
  uint8_t _clockPin;
  uint8_t _dataInPin;
  uint8_t *_values;  // Bit array, initialised in _begin method.
  int _currentByteIndex;
  unsigned long _lastAquireCycleStart;
  uint16_t _pulseDelayTime;  // Delay microsecs; can be tuned for the S88 hardware
  uint16_t _shortDelayTime;
  uint8_t _bitIndex;
  unsigned long _lastPulseTime;
  uint8_t _state;
  uint8_t _maxBitsPerEntry;


  // The acquire cycle time is a target maximum rate.  If there are a lot of signals or the
  // bit time is long, then the cycle time may be longer.
  const unsigned long _acquireCycleTime = 20000; // target 20 milliseconds between acquire cycles

public:
  S88bus(VPIN firstVpin, int nPins, uint8_t loadPin, uint8_t resetPin, uint8_t clockPin, uint8_t dataInPin, uint16_t bitTime) :
    IODevice(firstVpin, nPins),
    _loadPin(loadPin),
    _resetPin(resetPin),
    _clockPin(clockPin),
    _dataInPin(dataInPin),
    _currentByteIndex(0),
    _lastAquireCycleStart(0),
    _pulseDelayTime((bitTime+1)/2)
  { 
    // Allocate memory for input values.
    _values = (uint8_t *)calloc((nPins+7)/8, 1);
    _shortDelayTime = (_pulseDelayTime > 10) ? 10 : _pulseDelayTime;
    _state = 0;
    // The program typically manages 30 microseconds per clock cycle
    // with no waiting, so limit the number of bits so that loop doesn't 
    // take much more than 200us.
    _maxBitsPerEntry = 200 / (30+bitTime) + 1;
    addDevice(this);
  }

  static void create(VPIN firstVpin, int nPins, uint8_t loadPin, uint8_t resetPin, uint8_t clockPin, uint8_t dataInPin, uint16_t bitTime) {
    new S88bus(firstVpin, nPins, loadPin, resetPin, clockPin, dataInPin, bitTime);
  }

protected:
  void _begin() override {
    pinMode(_loadPin, OUTPUT);
    pinMode(_resetPin, OUTPUT);
    pinMode(_clockPin, OUTPUT);
    pinMode(_dataInPin, INPUT);

    #ifdef DIAG_IO 
    _display();
    #endif
  }

  // Read method returns the latest aquired value for the nominated VPIN number.
  int _read(VPIN vpin) override {
    uint16_t pin = vpin - _firstVpin;
    uint8_t mask = 1 << (pin % 8);
    uint16_t byteIndex = pin / 8;
    return (_values[byteIndex] & mask) ? 1 : 0;
  }

  // Loop method acquires the input states from the shift register.
  // At the beginning of each acquisition cycle, instruct the bus registers to acquire the
  // input states from the latches, then reset the latches.  On
  // subsequent loop entries, some of the input states are shifted from the
  // registers, until they have all been read.  Then the whole process
  // resumes for the next acquisition cycle.  The operations are spread over consecutive
  // loop entries to restrict the amount of time taken in each entry.
  void _loop(unsigned long currentMicros) override {
    // If just starting a new read, then latch the input values into the S88
    // registers.  The active edge is the rising edge in each case, so we
    // can use shorter delays for some transitions.
    switch (_state) {
      case 0: // Starting cycle.  Set up LOAD and CLOCK pins.
        _lastAquireCycleStart = currentMicros;
        // Set LOAD pin
        ArduinoPins::fastWriteDigital(_loadPin, HIGH);
        _lastPulseTime = micros();
        pulseDelay(_shortDelayTime);
        // Pulse CLOCK pin to read inputs into registers
        ArduinoPins::fastWriteDigital(_clockPin, HIGH);
        // Clear CLOCK, and set up RESET pin.
        pulseDelay(_pulseDelayTime);
        ArduinoPins::fastWriteDigital(_clockPin, LOW);
        pulseDelay(_shortDelayTime);
        // Pulse RESET pin to clear inputs ready for next acquisition period
        ArduinoPins::fastWriteDigital(_resetPin, HIGH);
        _state = 1;
        delayUntil(_lastPulseTime + _pulseDelayTime);
        return;
      case 1: // Clear RESET and LOAD
        ArduinoPins::fastWriteDigital(_resetPin, LOW);
        pulseDelay(_shortDelayTime);
        ArduinoPins::fastWriteDigital(_loadPin, LOW);
        // Initialise variables used in reading bits.
        _currentByteIndex = _bitIndex = 0;
        _state = 2;
        /* fallthrough */
      case 2:
        // Subsequent loop entries, read each bit in turn from the shiftregister.
        uint8_t bitCount = 0;
        while (true) {
          uint8_t mask = (1 << _bitIndex);
          bool newValue = ArduinoPins::fastReadDigital(_dataInPin);
#ifdef DIAG_IO
          bool oldValue = _values[_currentByteIndex] & mask;
          if (newValue != oldValue) DIAG(F("S88 VPIN:%d Value:%d"),
            _firstVpin+_currentByteIndex*8+_bitIndex, newValue);
#endif
          if (newValue)
            _values[_currentByteIndex] |= mask;
          else
            _values[_currentByteIndex] &= ~mask;
          if (++_bitIndex == 8) {
            // Byte completed, so move to next one.
            _currentByteIndex++;
            _bitIndex = 0;
          }

          // Check if this cycle is complete.
          if (_currentByteIndex*8 + _bitIndex >= _nPins) {
            // All bits in the shift register have been read now, so 
            // don't read again until next acquisition cycle time
            delayUntil(_lastAquireCycleStart + _acquireCycleTime);
            _state = 0;
            return;
          }

          // Clock next bit in.
          pulseDelay(_pulseDelayTime);
          ArduinoPins::fastWriteDigital(_clockPin, HIGH);
          pulseDelay(_pulseDelayTime);
          ArduinoPins::fastWriteDigital(_clockPin, LOW);

          // See if we've done all we're allowed on this entry
          if (++bitCount >= _maxBitsPerEntry) {
            delayUntil(_lastPulseTime + _pulseDelayTime);
            return;
          }
      }
    }
  }

  void _display() override {
    DIAG(F("S88bus Configured on Vpins %d-%d, LOAD=%d RESET=%d CLK=%d DATAIN=%d"), 
      _firstVpin, _firstVpin+_nPins-1, _loadPin, _resetPin, _clockPin, _dataInPin);
  }

  // Helper function to delay until a minimum number of microseconds have elapsed
  //  since _lastPulseTime.
  void pulseDelay(uint16_t duration) {
    delayMicroseconds(duration);
    _lastPulseTime = micros();
  }

};

#endif
