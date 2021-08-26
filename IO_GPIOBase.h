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
  GPIOBase(FSH *deviceName, VPIN firstVpin, uint8_t nPins, uint8_t I2CAddress, int interruptPin);
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
  bool _hasCallback(VPIN vpin) {
    (void)vpin;   // suppress compiler warning
    return true;  // Enable callback if caller wants to use it.
  }

  // Data fields
  uint8_t _I2CAddress; 
  // Allocate enough space for all input pins
  T _portInputState; 
  T _portOutputState;
  T _portMode;
  T _portPullup;
  // Interval between refreshes of each input port
  static const int _portTickTime = 4000;
  unsigned long _lastLoopEntry = 0;

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
GPIOBase<T>::GPIOBase(FSH *deviceName, VPIN firstVpin, uint8_t nPins, uint8_t I2CAddress, int interruptPin) {
  _deviceName = deviceName;
  _firstVpin = firstVpin;
  _nPins = nPins;
  _I2CAddress = I2CAddress;
  _gpioInterruptPin = interruptPin;
  // Add device to list of devices.
  addDevice(this);
}

template <class T>
void GPIOBase<T>::_begin() {
  // Configure pin used for GPIO extender notification of change (if allocated)
  if (_gpioInterruptPin >= 0) 
    pinMode(_gpioInterruptPin, INPUT_PULLUP);

  I2CManager.begin();
  I2CManager.setClock(400000);
  if (I2CManager.exists(_I2CAddress)) {
    _display();
    _portMode = 0;  // default to input mode
    _portPullup = -1; // default to pullup enabled
    _portInputState = -1; 
  }
  _setupDevice();
  _deviceState = DEVSTATE_NORMAL;
  _lastLoopEntry = micros();
}

// Configuration parameters for inputs: 
//  params[0]: enable pullup
//  params[1]: invert input (optional)
template <class T>
bool GPIOBase<T>::_configure(VPIN vpin, ConfigTypeEnum configType, int paramCount, int params[]) {
  if (configType != CONFIGURE_INPUT) return false;
  if (paramCount == 0 || paramCount > 1) return false;
  bool pullup = params[0];
  int pin = vpin - _firstVpin;
  #ifdef DIAG_IO
  DIAG(F("%S I2C:x%x Config Pin:%d Val:%d"), _deviceName, _I2CAddress, pin, pullup);
  #endif
  uint16_t mask = 1 << pin;
  if (pullup) 
    _portPullup |= mask;
  else
    _portPullup &= ~mask;

  // Call subclass's virtual function to write to device
  _writePullups();
  // Re-read port following change
  _readGpioPort();

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
      DIAG(F("%S I2C:x%x Error:%d %S"), _deviceName, _I2CAddress, status, 
        I2CManager.getErrorMessage(status));
    }
    _processCompletion(status);

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
      DIAG(F("%S I2C:x%x PortStates:%x"), _deviceName, _I2CAddress, _portInputState);
    #endif
  }

  // Check if interrupt configured.  If so, and pin is not pulled down, finish.
  if (_gpioInterruptPin >= 0) {
    if (digitalRead(_gpioInterruptPin)) return;
  } else
  // No interrupt pin.  Check if tick has elapsed.  If not, finish.
  if (currentMicros - _lastLoopEntry < (unsigned long)_portTickTime) return;

  // TODO: Could suppress reads if there are no pins configured as inputs!

  // Read input
  _lastLoopEntry = currentMicros;
  if (_deviceState == DEVSTATE_NORMAL) {
    _readGpioPort(false);  // Initiate non-blocking read
    _deviceState= DEVSTATE_SCANNING;
  }
}

template <class T>
void GPIOBase<T>::_display() {
  DIAG(F("%S I2C:x%x Configured on Vpins:%d-%d"), _deviceName, _I2CAddress, 
    _firstVpin, _firstVpin+_nPins-1);
}

template <class T>
void GPIOBase<T>::_write(VPIN vpin, int value) {
  int pin = vpin - _firstVpin;
  T mask = 1 << pin;
  #ifdef DIAG_IO
  DIAG(F("%S I2C:x%x Write Pin:%d Val:%d"), _deviceName, _I2CAddress, pin, value);
  #endif

  // Set port mode output
  if (!(_portMode & mask)) {
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

  // Set port mode to input
  if (_portMode & mask) {
    _portMode &= ~mask;
    _writePortModes();
    // Port won't have been read yet, so read it now.
    _readGpioPort();
    #ifdef DIAG_IO
    DIAG(F("%S I2C:x%x PortStates:%x"), _deviceName, _I2CAddress, _portInputState);
    #endif
  }
  return (_portInputState & mask) ? 0 : 1;  // Invert state (5v=0, 0v=1)
}

#endif