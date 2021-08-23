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
#include "IODevice.h"
#include "DIAG.h" 
#include "FSH.h"
#include "IO_MCP23017.h"

#if defined(ARDUINO_ARCH_AVR) || defined(ARDUINO_ARCH_MEGAAVR)
#define USE_FAST_IO
#endif

//==================================================================================================================
// Static methods
//------------------------------------------------------------------------------------------------------------------

// Static functions

// Static method to initialise the IODevice subsystem.  

#if !defined(IO_NO_HAL)

// Create any standard device instances that may be required, such as the Arduino pins 
// and PCA9685.
void IODevice::begin() {
  // Initialise the IO subsystem
  ArduinoPins::create(2, NUM_DIGITAL_PINS-2);  // Reserve pins for direct access
  // Predefine two PCA9685 modules 0x40-0x41
  // Allocates 32 pins 100-131
  PCA9685::create(100, 16, 0x40);
  PCA9685::create(116, 16, 0x41);
  // Predefine two MCP23017 module 0x20/0x21
  // Allocates 32 pins 164-195
  MCP23017::create(164, 16, 0x20);
  MCP23017::create(180, 16, 0x21);

  // Call the begin() methods of each configured device in turn
  for (IODevice *dev=_firstDevice; dev!=NULL; dev = dev->_nextDevice) {
    dev->_begin();
  }
  _initPhase = false;
}

// Overarching static loop() method for the IODevice subsystem.  Works through the
// list of installed devices and calls their individual _loop() method.
// Devices may or may not implement this, but if they do it is useful for things like animations 
// or flashing LEDs.
// The current value of micros() is passed as a parameter, so the called loop function
// doesn't need to invoke it.
void IODevice::loop() {
  unsigned long currentMicros = micros();
  // Call every device's loop function in turn, one per entry.
  if (!_nextLoopDevice) _nextLoopDevice = _firstDevice;
  _nextLoopDevice->_loop(currentMicros);
  _nextLoopDevice = _nextLoopDevice->_nextDevice;
  
  // Report loop time if diags enabled
#if defined(DIAG_LOOPTIMES)
  static unsigned long lastMicros = 0;
  static unsigned long maxElapsed = 0;
  static unsigned long lastOutputTime = 0;
  static unsigned long count = 0;
  const unsigned long interval = (unsigned long)5 * 1000 * 1000; // 5 seconds in microsec
  unsigned long elapsed = currentMicros - lastMicros;
  // Ignore long loop counts while message is still outputting
  if (currentMicros - lastOutputTime > 3000UL) {
    if (elapsed > maxElapsed) maxElapsed = elapsed;
  }
  count++;
  if (currentMicros - lastOutputTime > interval) {
    if (lastOutputTime > 0) 
      LCD(1,F("Loop=%lus,%lus max"), interval/count, maxElapsed);
    maxElapsed = 0;
    count = 0;
    lastOutputTime = currentMicros;
  }
  lastMicros = micros();
#endif
}

// Display a list of all the devices on the diagnostic stream.
void IODevice::DumpAll() {
  for (IODevice *dev = _firstDevice; dev != 0; dev = dev->_nextDevice) {
    dev->_display();
  }
}

// Determine if the specified vpin is allocated to a device.
bool IODevice::exists(VPIN vpin) {
  return findDevice(vpin) != NULL;
}

// check whether the pin supports notification.  If so, then regular _read calls are not required.
bool IODevice::hasCallback(VPIN vpin) {
  IODevice *dev = findDevice(vpin);
  if (!dev) return false;
  return dev->_hasCallback(vpin);
}

// Display (to diagnostics) details of the device.
void IODevice::_display() {
  DIAG(F("Unknown device Vpins:%d-%d"), (int)_firstVpin, (int)_firstVpin+_nPins-1);
}

// Find device associated with nominated Vpin and pass configuration values on to it.
//   Return false if not found.
bool IODevice::configure(VPIN vpin, ConfigTypeEnum configType, int paramCount, int params[]) {
  IODevice *dev = findDevice(vpin);
  if (dev) return dev->_configure(vpin, configType, paramCount, params);
  return false;
}

// Write value to virtual pin(s).  If multiple devices are allocated the same pin
//  then only the first one found will be used.
void IODevice::write(VPIN vpin, int value) {
  IODevice *dev = findDevice(vpin);
  if (dev) {
    dev->_write(vpin, value);
    return;
  }
#ifdef DIAG_IO
  //DIAG(F("IODevice::write(): Vpin ID %d not found!"), (int)vpin);
#endif
}

// Write analogue value to virtual pin(s).  If multiple devices are allocated the same pin
//  then only the first one found will be used.
void IODevice::writeAnalogue(VPIN vpin, int value, int profile) {
  IODevice *dev = findDevice(vpin);
  if (dev) {
    dev->_writeAnalogue(vpin, value, profile);
    return;
  }
#ifdef DIAG_IO
  //DIAG(F("IODevice::writeAnalogue(): Vpin ID %d not found!"), (int)vpin);
#endif
}

// isActive returns true if the device is currently in an animation of some sort, e.g. is changing
//  the output over a period of time.
bool IODevice::isActive(VPIN vpin) {
  IODevice *dev = findDevice(vpin);
  if (dev) 
    return dev->_isActive(vpin);
  else
    return false;
}

void IODevice::setGPIOInterruptPin(int16_t pinNumber) {
  if (pinNumber >= 0)
    pinMode(pinNumber, INPUT_PULLUP);
  _gpioInterruptPin = pinNumber;
}

// Private helper function to add a device to the chain of devices.
void IODevice::addDevice(IODevice *newDevice) {
  // Link new object to the end of the chain.  Thereby, the first devices to be declared/created
  // will be located faster by findDevice than those which are created later.
  // Ideally declare/create the digital IO pins first, then servos, then more esoteric devices.
  IODevice *lastDevice;
  if (_firstDevice == 0)
    _firstDevice = newDevice;
  else {
    for (IODevice *dev = _firstDevice; dev != 0; dev = dev->_nextDevice)
      lastDevice = dev;
    lastDevice->_nextDevice = newDevice;
  }
  newDevice->_nextDevice = 0;

  // If the IODevice::begin() method has already been called, initialise device here.  If not,
  // the device's _begin() method will be called by IODevice::begin().
  if (!_initPhase)
    newDevice->_begin();
}

// Private helper function to locate a device by VPIN.  Returns NULL if not found
IODevice *IODevice::findDevice(VPIN vpin) { 
  for (IODevice *dev = _firstDevice; dev != 0; dev = dev->_nextDevice) {
    if (dev->owns(vpin)) 
      return dev;
  }
  return NULL;
}
  
//==================================================================================================================
// Static data
//------------------------------------------------------------------------------------------------------------------

// Chain of callback blocks (identifying registered callback functions for state changes)
IONotifyCallback *IONotifyCallback::first = 0;

// Start of chain of devices.
IODevice *IODevice::_firstDevice = 0;

// Reference to next device to be called on _loop() method.
IODevice *IODevice::_nextLoopDevice = 0;

// Flag which is reset when IODevice::begin has been called.
bool IODevice::_initPhase = true;  


//==================================================================================================================
// Instance members
//------------------------------------------------------------------------------------------------------------------

// Method to check whether the id corresponds to this device
bool IODevice::owns(VPIN id) {
  return (id >= _firstVpin && id < _firstVpin + _nPins);
}

// Read value from virtual pin.
int IODevice::read(VPIN vpin) {
  for (IODevice *dev = _firstDevice; dev != 0; dev = dev->_nextDevice) {
    if (dev->owns(vpin)) 
      return dev->_read(vpin);
  }
#ifdef DIAG_IO
  //DIAG(F("IODevice::read(): Vpin %d not found!"), (int)vpin);
#endif
  return false;
}


#else // !defined(IO_NO_HAL)

// Minimal implementations of public HAL interface, to support Arduino pin I/O and nothing more.

void IODevice::begin() { DIAG(F("NO HAL CONFIGURED!")); }
bool IODevice::configure(VPIN vpin, ConfigTypeEnum configType, int paramCount, int params[]) {
  (void)vpin; (void)paramCount; (void)params; // Avoid compiler warnings
  if (configType == CONFIGURE_INPUT || configType == CONFIGURE_OUTPUT) 
    return true;
  else
    return false;
}
void IODevice::write(VPIN vpin, int value) {
  digitalWrite(vpin, value);
  pinMode(vpin, OUTPUT);
}
void IODevice::writeAnalogue(VPIN vpin, int value, int profile) {
  (void)vpin; (void)value; (void)profile; // Avoid compiler warnings
}
bool IODevice::hasCallback(VPIN vpin) { 
  (void)vpin;  // Avoid compiler warnings
  return false; 
}
int IODevice::read(VPIN vpin) { 
  pinMode(vpin, INPUT_PULLUP);
  return !digitalRead(vpin);  // Return inverted state (5v=0, 0v=1)
}
void IODevice::loop() {}
void IODevice::DumpAll() {
  DIAG(F("NO HAL CONFIGURED!"));
}
bool IODevice::exists(VPIN vpin) { return (vpin > 2 && vpin < 49); }
void IODevice::setGPIOInterruptPin(int16_t pinNumber) {
  (void) pinNumber; // Avoid compiler warning
}

// Chain of callback blocks (identifying registered callback functions for state changes)
// Not used in IO_NO_HAL but must be declared.
IONotifyCallback *IONotifyCallback::first = 0;

#endif // IO_NO_HAL


/////////////////////////////////////////////////////////////////////////////////////////////////////

// Constructor
ArduinoPins::ArduinoPins(VPIN firstVpin, int nPins) {
  _firstVpin = firstVpin;
  _nPins = nPins;
  uint8_t arrayLen = (_nPins+7)/8;
  _pinPullups = (uint8_t *)calloc(2, arrayLen);
  _pinModes = (&_pinPullups[0]) + arrayLen;
  for (int i=0; i<arrayLen; i++) {
    _pinPullups[i] = 0;
    _pinModes[i] = 0;
  }
}

// Device-specific pin configuration.  Configure should be called infrequently so simplify 
// code by using the standard pinMode function.
bool ArduinoPins::_configure(VPIN vpin, ConfigTypeEnum configType, int paramCount, int params[]) {
  if (configType != CONFIGURE_INPUT) return false;
  if (paramCount != 1) return false;
  bool pullup = params[0];

  int pin = vpin;
  #ifdef DIAG_IO
  DIAG(F("Arduino _configurePullup Pin:%d Val:%d"), pin, pullup);
  #endif
  uint8_t mask = 1 << ((pin-_firstVpin) % 8);
  uint8_t index = (pin-_firstVpin) / 8;
  _pinModes[index] &= ~mask;  // set to input mode
  if (pullup) {
    _pinPullups[index] |= mask;
    pinMode(pin, INPUT_PULLUP);
  } else {
    _pinPullups[index] &= ~mask;
    pinMode(pin, INPUT);
  }
  return true;
}

// Device-specific write function.
void ArduinoPins::_write(VPIN vpin, int value) {
  int pin = vpin;
  #ifdef DIAG_IO
  DIAG(F("Arduino Write Pin:%d Val:%d"), pin, value);
  #endif
  uint8_t mask = 1 << ((pin-_firstVpin) % 8);
  uint8_t index = (pin-_firstVpin) / 8;
  // First update the output state, then set into write mode if not already.
  fastWriteDigital(pin, value);
  if (!(_pinModes[index] & mask)) {
    // Currently in read mode, change to write mode
    _pinModes[index] |= mask;
    // Since mode changes should be infrequent, use standard pinMode function
    pinMode(pin, OUTPUT);
  }
}

// Device-specific read function.
int ArduinoPins::_read(VPIN vpin) {
  int pin = vpin;
  uint8_t mask = 1 << ((pin-_firstVpin) % 8);
  uint8_t index = (pin-_firstVpin) / 8;
  if (_pinModes[index] & mask) {
    // Currently in write mode, change to read mode
    _pinModes[index] &= ~mask;
    // Since mode changes should be infrequent, use standard pinMode function
    if (_pinPullups[index] & mask) 
      pinMode(pin, INPUT_PULLUP);
    else
      pinMode(pin, INPUT);
  }
  int value = !fastReadDigital(pin); // Invert (5v=0, 0v=1)

  #ifdef DIAG_IO
  //DIAG(F("Arduino Read Pin:%d Value:%d"), pin, value);
  #endif
  return value;
}

void ArduinoPins::_display() {
  DIAG(F("Arduino Vpins:%d-%d"), (int)_firstVpin, (int)_firstVpin+_nPins-1);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////


void ArduinoPins::fastWriteDigital(uint8_t pin, uint8_t value) {
#if defined(USE_FAST_IO)
  if (pin >= NUM_DIGITAL_PINS) return;
  uint8_t mask = digitalPinToBitMask(pin);
  uint8_t port = digitalPinToPort(pin);
  volatile uint8_t *outPortAdr = portOutputRegister(port);
  noInterrupts();
  if (value) 
    *outPortAdr |= mask;
  else
    *outPortAdr &= ~mask;
  interrupts();
#else
  digitalWrite(pin, value);
#endif
}

bool ArduinoPins::fastReadDigital(uint8_t pin) {
#if defined(USE_FAST_IO)
  if (pin >= NUM_DIGITAL_PINS) return false;
  uint8_t mask = digitalPinToBitMask(pin);
  uint8_t port = digitalPinToPort(pin);
  volatile uint8_t *inPortAdr = portInputRegister(port);
  // read input
  bool result = (*inPortAdr & mask) != 0;  
#else
  bool result = digitalRead(pin);
#endif
  return result;
}

