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
 *    Data rate 128 samples/sec (7.8ms/sample, but scanned every 10ms)
 *    Comparator off
 *    Gain FSR=6.144V
 * The gain means that the maximum input voltage of 5V (when Vss=5V) gives a reading 
 * of 32767*(5.0/6.144) = 26666.
 * 
 * A device is configured by the following:
 *   ADS111x::create(firstVpin, nPins, i2cAddress);
 * for example
 *   ADS111x::create(300, 1, 0x48);  // single-input ADS1113
 *   ADS111x::create(300, 4, 0x48);  // four-input ADS1115
 * 
 * Note: The device is simple and does not need initial configuration, so it should recover from
 * temporary loss of communications or power.
 **********************************************************************************************/
class ADS111x: public IODevice { 
public:
  static void create(VPIN firstVpin, int nPins, I2CAddress i2cAddress) {
    if (checkNoOverlap(firstVpin,nPins,i2cAddress)) new ADS111x(firstVpin, nPins, i2cAddress);
  }
private:
  ADS111x(VPIN firstVpin, int nPins, I2CAddress i2cAddress) {
    _firstVpin = firstVpin;
    _nPins = (nPins > 4) ? 4 : nPins;
    _I2CAddress = i2cAddress;
    _currentPin = 0;
    for (int8_t i=0; i<_nPins; i++)
      _value[i] = -1;
    addDevice(this);
  }
  void _begin() {
    // Initialise I2C
    I2CManager.begin();
    // ADS111x support high-speed I2C (4.3MHz) but that requires special
    // processing.  So stick to fast mode (400kHz maximum).
    I2CManager.setClock(400000);
    // Initialise ADS device
    if (I2CManager.exists(_I2CAddress)) {
      _nextState = STATE_STARTSCAN;
#ifdef DIAG_IO
      _display();
#endif
    } else {
      DIAG(F("ADS111x device not found, I2C:%s"), _I2CAddress.toString());
      _deviceState = DEVSTATE_FAILED;
    }
  }
  void _loop(unsigned long currentMicros) override {

    // Check that previous non-blocking write has completed, if not then wait
    uint8_t status = _i2crb.status;
    if (status == I2C_STATUS_PENDING) return;  // Busy, so don't do anything.
    if (status == I2C_STATUS_OK) {
      switch (_nextState) {
        case STATE_STARTSCAN:
          // Configure ADC and multiplexer for next scan.  See ADS111x datasheet for details
          // of configuration register settings.
          _outBuffer[0] = 0x01; // Config register address
          _outBuffer[1] = 0xC0 + (_currentPin << 4); // Trigger single-shot, channel n
          _outBuffer[2] = 0xA3;           // 250 samples/sec, comparator off
          // Write command, without waiting for completion.
          I2CManager.write(_I2CAddress, _outBuffer, 3, &_i2crb);

          delayUntil(currentMicros + scanInterval);
          _nextState = STATE_STARTREAD;
          break;

        case STATE_STARTREAD:
          // Reading the pin value
          _outBuffer[0] = 0x00;  // Conversion register address
          I2CManager.read(_I2CAddress, _inBuffer, 2, _outBuffer, 1, &_i2crb); // Read register
          _nextState = STATE_GETVALUE;
          break;

        case STATE_GETVALUE:
          _value[_currentPin] = ((uint16_t)_inBuffer[0] << 8) + (uint16_t)_inBuffer[1];
          #ifdef IO_ANALOGUE_SLOW
          DIAG(F("ADS111x VPIN:%u value:%d"), _currentPin, _value[_currentPin]);
          #endif

          // Move to next pin
          if (++_currentPin >= _nPins) _currentPin = 0;
          _nextState = STATE_STARTSCAN;
          break;
        
        default:
          break;
      }
    } else { // error status
      DIAG(F("ADS111x I2C:%s Error:%d %S"), _I2CAddress.toString(), status, I2CManager.getErrorMessage(status));
      _deviceState = DEVSTATE_FAILED;
    }
  }

  int _readAnalogue(VPIN vpin) override {
    int pin = vpin - _firstVpin;
    return _value[pin];
  }
  
  void _display() override {
    DIAG(F("ADS111x I2C:%s Configured on Vpins:%u-%u %S"), _I2CAddress.toString(), _firstVpin, _firstVpin+_nPins-1,
      _deviceState == DEVSTATE_FAILED ? F("OFFLINE") : F(""));
  }

  // ADC conversion rate is 250SPS, or 4ms per conversion.  Set the period between updates to 10ms. 
  // This is enough to allow the conversion to reliably complete in time.
  #ifndef IO_ANALOGUE_SLOW
  const unsigned long scanInterval = 10000UL;  // Period between successive ADC scans in microseconds.
  #else
  const unsigned long scanInterval = 1000000UL;  // Period between successive ADC scans in microseconds.
  #endif
  enum : uint8_t {
    STATE_STARTSCAN,
    STATE_STARTREAD, 
    STATE_GETVALUE,
  };
  uint16_t _value[4];
  uint8_t _outBuffer[3];
  uint8_t _inBuffer[2];
  uint8_t _currentPin;  // ADC pin currently being scanned
  I2CRB _i2crb;
  uint8_t _nextState;
};

#endif // io_analogueinputs_h