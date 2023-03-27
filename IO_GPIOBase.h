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

#ifndef IO_GPIOBASE_H
#define IO_GPIOBASE_H

#include "IODevice.h"
#include "I2CManager.h"
#include "DIAG.h"

// GPIOBase is defined as a class template.  This allows it to be instantiated by
// subclasses with different types, according to the number of pins on the GPIO module.
// For example, GPIOBase<uint8_t> for 8 pins, GPIOBase<uint16_t> for 16 pins etc.
// A module with up to 64 pins can be handled in this way (uint64_t).

template <class T>
class GPIOBase : public IODevice {

protected:
  // Constructor
  GPIOBase(FSH *deviceName, VPIN firstVpin, uint8_t nPins, I2CAddress i2cAddress, int interruptPin);
  // Device-specific initialisation
  void _begin() override;
  // Device-specific pin configuration function.  
  bool _configure(VPIN vpin, ConfigTypeEnum configType, int paramCount, int params[]) override;
  // Pin write function.
  void _write(VPIN vpin, int value) override;
  // Pin read function.
  int _read(VPIN vpin) override;
  void _display() override;
  void _loop(unsigned long currentMicros) override;

  // Data fields
 
  // Allocate enough space for all input pins
  T _portInputState; // 1=high (inactive), 0=low (activated)
  T _portOutputState; // 1 =high, 0=low
  T _portMode;  // 0=input, 1=output
  T _portPullup; // 0=nopullup, 1=pullup
  T _portInUse;  // 0=not in use, 1=in use
  // Target interval between refreshes of each input port
  static const int _portTickTime = 4000; // 4ms

  // Virtual functions for interfacing with I2C GPIO Device
  virtual void _writeGpioPort() = 0;
  virtual void _readGpioPort(bool immediate=true) = 0;
  virtual void _writePullups() {};
  virtual void _writePortModes() {};
  virtual void _setupDevice() {};
  virtual void _processCompletion(uint8_t status) {
    (void)status; // Suppress compiler warning
  };

  I2CRB requestBlock;
  FSH *_deviceName;
};

// Because class GPIOBase is a template, the implementation (below) must be contained within the same
// file as the class declaration (above).  Otherwise it won't compile!

// Constructor
template <class T>
GPIOBase<T>::GPIOBase(FSH *deviceName, VPIN firstVpin, uint8_t nPins, I2CAddress i2cAddress, int interruptPin) :
  IODevice(firstVpin, nPins)
{
  if (_nPins > (int)sizeof(T)*8) _nPins = sizeof(T)*8;  // Ensure nPins is consistent with the number of bits in T
  _deviceName = deviceName;
  _I2CAddress = i2cAddress;
  _gpioInterruptPin = interruptPin;
  _hasCallback = true;
  // Add device to list of devices.
  addDevice(this);

  _portMode = 0;  // default to input mode
  _portPullup = -1; // default to pullup enabled
  _portInputState = -1;  // default to all inputs high (inactive)
  _portInUse = 0;  // No ports in use initially.
}

template <class T>
void GPIOBase<T>::_begin() {
  // Configure pin used for GPIO extender notification of change (if allocated)
  if (_gpioInterruptPin >= 0) 
    pinMode(_gpioInterruptPin, INPUT_PULLUP);

  I2CManager.begin();
  I2CManager.setClock(400000);
  if (I2CManager.exists(_I2CAddress)) {
#if defined(DIAG_IO)
    _display();
#endif
    _setupDevice();
    _deviceState = DEVSTATE_NORMAL;
  } else {
    DIAG(F("%S I2C:%s Device not detected"), _deviceName, _I2CAddress.toString());
    _deviceState = DEVSTATE_FAILED;
  }
}

// Configuration parameters for inputs: 
//  params[0]: enable pullup
template <class T>
bool GPIOBase<T>::_configure(VPIN vpin, ConfigTypeEnum configType, int paramCount, int params[]) {
  if (configType != CONFIGURE_INPUT) return false;
  if (paramCount == 0 || paramCount > 1) return false;
  bool pullup = params[0];
  int pin = vpin - _firstVpin;
  #ifdef DIAG_IO
  DIAG(F("%S I2C:%s Config Pin:%d Val:%d"), _deviceName, _I2CAddress.toString(), pin, pullup);
  #endif
  uint16_t mask = 1 << pin;
  if (pullup) 
    _portPullup |= mask;
  else
    _portPullup &= ~mask;
  // Mark that port has been accessed
  _portInUse |= mask;
  // Set input mode
  _portMode &= ~mask;

  // Call subclass's virtual function to write to device
  _writePortModes();
  _writePullups();
  // Port change will be notified on next loop entry.

  return true;
}

// Periodically read the input port
template <class T>
void GPIOBase<T>::_loop(unsigned long currentMicros) {
  T lastPortStates = _portInputState;
  if (_deviceState == DEVSTATE_SCANNING && !requestBlock.isBusy()) {
    uint8_t status = requestBlock.status;
    if (status == I2C_STATUS_OK) {
      _deviceState = DEVSTATE_NORMAL;
    } else {
      _deviceState = DEVSTATE_FAILED;
      DIAG(F("%S I2C:%s Error:%d %S"), _deviceName, _I2CAddress.toString(), status, 
        I2CManager.getErrorMessage(status));
    }
    _processCompletion(status);
  // Set unused pin and write mode pin value to 1
    _portInputState |= ~_portInUse | _portMode;

    // Scan for changes in input states and invoke callback (if present)
    T differences = lastPortStates ^ _portInputState;
    if (differences && IONotifyCallback::hasCallback()) {
      // Scan for differences bit by bit
      T mask = 1;
      for (int pin=0; pin<_nPins; pin++) {
        if (differences & mask) {
          // Change detected.
          IONotifyCallback::invokeAll(_firstVpin+pin, (_portInputState & mask) == 0);
        }
        mask <<= 1;
      }
    }

    #ifdef DIAG_IO
    if (differences)
      DIAG(F("%S I2C:%s PortStates:%x"), _deviceName, _I2CAddress.toString(), _portInputState);
    #endif
  }

  // Check if interrupt configured.  If not, or if it is active (pulled down), then
  //  initiate a scan.
  if (_gpioInterruptPin < 0 || !digitalRead(_gpioInterruptPin)) {
    // TODO: Could suppress reads if there are no pins configured as inputs!

    // Read input
    if (_deviceState == DEVSTATE_NORMAL) {
      _readGpioPort(false);  // Initiate non-blocking read
      _deviceState= DEVSTATE_SCANNING;
    }
  }
  // Delay next entry until tick elapsed.
  delayUntil(currentMicros + _portTickTime);
}

template <class T>
void GPIOBase<T>::_display() {
  DIAG(F("%S I2C:%s Configured on Vpins:%u-%u %S"), _deviceName, _I2CAddress.toString(), 
    _firstVpin, _firstVpin+_nPins-1, (_deviceState==DEVSTATE_FAILED) ? F("OFFLINE") : F(""));
}

template <class T>
void GPIOBase<T>::_write(VPIN vpin, int value) {
  int pin = vpin - _firstVpin;
  T mask = 1 << pin;
  #ifdef DIAG_IO
  DIAG(F("%S I2C:%s Write Pin:%d Val:%d"), _deviceName, _I2CAddress.toString(), pin, value);
  #endif

  // Set port mode output if currently not output mode
  if (!(_portMode & mask)) {
    _portInUse |= mask;
    _portMode |= mask;
    _writePortModes();
  }

  // Update port output state
  if (value) 
    _portOutputState |= mask;
  else
    _portOutputState &= ~mask;

  // Call subclass's virtual function to write to device.
  return _writeGpioPort();
}

template <class T>
int GPIOBase<T>::_read(VPIN vpin) {
  int pin = vpin - _firstVpin;
  T mask = 1 << pin;

  // Set port mode to input if currently output or first use
  if ((_portMode | ~_portInUse) & mask) {
    _portMode &= ~mask;
    _portInUse |= mask;
    _writePullups();
    _writePortModes();
    // Port won't have been read yet, so read it now.
    _readGpioPort();
  // Set unused pin and write mode pin value to 1
    _portInputState |= ~_portInUse | _portMode;
    #ifdef DIAG_IO
    DIAG(F("%S I2C:%s PortStates:%x"), _deviceName, _I2CAddress.toString(), _portInputState);
    #endif
  }
  return (_portInputState & mask) ? 0 : 1;  // Invert state (5v=0, 0v=1)
}

#endif
