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

#ifndef io_analogueinputs_h
#define io_analogueinputs_h

// Uncomment following line to slow the scan cycle down to 1second ADC samples, with
// diagnostic output of scanned values.
//#define IO_ANALOGUE_SLOW

#include "IODevice.h"
#include "I2CManager.h"
#include "DIAG.h"
#include "FSH.h"

/**********************************************************************************************
 * ADS111x class for I2C-connected analogue input modules ADS1113, ADS1114 and ADS1115.
 * 
 * ADS1113 and ADS1114 are restricted to 1 input.  ADS1115 has a multiplexer which allows 
 * any of four input pins to be read by its ADC.
 * 
 * The driver polls the device in accordance with the constant 'scanInterval' below.  On first loop
 * entry, the multiplexer is set to pin A0 and the ADC is triggered.  On second and subsequent
 * entries, the analogue value is read from the conversion register and then the multiplexer and
 * ADC are set up to read the next pin.
 * 
 * The ADS111x is set up as follows:
 *    Single-shot scan
 *    Data rate 128 samples/sec (7.8ms/sample)
 *    Comparator off
 *    Gain FSR=6.144V
 * The gain means that the maximum input voltage of 5V (when Vss=5V) gives a reading 
 * of 32767*(5.0/6.144) = 26666.
 * 
 * Note: The device is simple and does not need initial configuration, so it should recover from
 * temporary loss of communications or power.
 **********************************************************************************************/
class ADS111x: public IODevice { 
public:
  ADS111x(VPIN firstVpin, int nPins, uint8_t i2cAddress) {
    _firstVpin = firstVpin;
    _nPins = min(nPins,4);
    _i2cAddress = i2cAddress;
    _currentPin = _nPins; // Suppress read on first loop entry.
    addDevice(this);
  }
  static void create(VPIN firstVpin, int nPins, uint8_t i2cAddress) {
    new ADS111x(firstVpin, nPins, i2cAddress);
  }
  void _begin() {
    // Initialise ADS device
    if (I2CManager.exists(_i2cAddress)) {
#ifdef DIAG_IO
      _display();
#endif
    } else {
      DIAG(F("ADS111x device not found, I2C:%x"), _i2cAddress);
    }
  }
  void _loop(unsigned long currentMicros) {

    if (currentMicros - _lastMicros >= scanInterval) {
      // Check that previous non-blocking write has completed, if not then wait
      _i2crb.wait();

      // If _currentPin is in the valid range, continue reading the pin values
      if (_currentPin < _nPins) {
        _outBuffer[0] = 0x00;  // Conversion register address
        uint8_t status = I2CManager.read(_i2cAddress, _inBuffer, 2, 1, _outBuffer); // Read register
        if (status == I2C_STATUS_OK) {
          _value[_currentPin] = ((uint16_t)_inBuffer[0] << 8) + (uint16_t)_inBuffer[1];
          #ifdef IO_ANALOGUE_SLOW
          DIAG(F("ADS111x pin:%d value:%d"), _currentPin, _value[_currentPin]);
          #endif
        }
      }
      // Move to next pin
      if (++_currentPin >= _nPins) _currentPin = 0;
      
      // Configure ADC and multiplexer for next scan.  See ADS111x datasheet for details
      // of configuration register settings.
      _outBuffer[0] = 0x01; // Config register address
      _outBuffer[1] = 0xC0 + (_currentPin << 4); // Trigger single-shot, channel n
      _outBuffer[2] = 0x83;           // 128 samples/sec, comparator off
      // Write command, without waiting for completion.
      I2CManager.write(_i2cAddress, _outBuffer, 3, &_i2crb);

      _lastMicros = currentMicros;
    }
  }
  int _readAnalogue(VPIN vpin) {
    int pin = vpin - _firstVpin;
    return _value[pin];
  }
  void _display() {
    DIAG(F("ADS111x I2C:x%x Configured on Vpins:%d-%d"), _i2cAddress, _firstVpin, _firstVpin+_nPins-1);
  }

protected:
  // With ADC set to 128 samples/sec, that's 7.8ms/sample.  So set the period between updates to 10ms
  #ifndef IO_ANALOGUE_SLOW
  const unsigned long scanInterval = 10000UL;  // Period between successive ADC scans in microseconds.
  #else
  const unsigned long scanInterval = 1000000UL;  // Period between successive ADC scans in microseconds.
  #endif
  uint16_t _value[4];
  uint8_t _i2cAddress;
  uint8_t _outBuffer[3];
  uint8_t _inBuffer[2];
  uint8_t _currentPin;  // ADC pin currently being scanned
  unsigned long _lastMicros = 0;
  I2CRB _i2crb;
};

#endif // io_analogueinputs_h