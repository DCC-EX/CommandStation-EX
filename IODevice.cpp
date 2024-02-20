/*
 *  © 2021 Neil McKechnie
 *  © 2021 Harald Barth
 *  All rights reserved.
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
#include "DCCTimer.h"

#if defined(ARDUINO_ARCH_AVR) || defined(ARDUINO_ARCH_MEGAAVR)
#define USE_FAST_IO
#endif

// Link to halSetup function.  If not defined, the function reference will be NULL.
extern __attribute__((weak)) void halSetup();
extern __attribute__((weak)) bool exrailHalSetup();

//==================================================================================================================
// Static methods
//------------------------------------------------------------------------------------------------------------------

// Static functions

// Static method to initialise the IODevice subsystem.  

#if !defined(IO_NO_HAL)

// Create any standard device instances that may be required, such as the Arduino pins 
// and PCA9685.
void IODevice::begin() {
  // Initialise the IO subsystem defaults
  ArduinoPins::create(2, NUM_DIGITAL_PINS-2);  // Reserve pins for direct access

  // Call user's halSetup() function (if defined in the build in myHal.cpp).
  //  The contents will depend on the user's system hardware configuration.
  //  The myHal.cpp file is a standard C++ module so has access to all of the DCC++EX APIs.
  // This is done early so that the subsequent defaults will detect an overlap and not
  // create something that conflicts with the user's vpin definitions. 
  if (halSetup)
    halSetup();

  // include any HAL devices defined in exrail. 
  bool ignoreDefaults=false;
  if (exrailHalSetup)
    ignoreDefaults=exrailHalSetup();
  if (ignoreDefaults) return;
  
  // Predefine two PCA9685 modules 0x40-0x41 if no conflicts
  // Allocates 32 pins 100-131
  const bool silent=true; // no message if these conflict
  if (checkNoOverlap(100, 16, 0x40, silent)) {
    PCA9685::create(100, 16, 0x40);
  } 

  if (checkNoOverlap(116, 16, 0x41, silent)) {
    PCA9685::create(116, 16, 0x41);
  } 
  
  // Predefine two MCP23017 module 0x20/0x21 if no conflicts
  // Allocates 32 pins 164-195
  if (checkNoOverlap(164, 16, 0x20, silent)) {
    MCP23017::create(164, 16, 0x20);
  } 

  if (checkNoOverlap(180, 16, 0x21, silent)) {
    MCP23017::create(180, 16, 0x21);
  } 
}

// reset() function to reinitialise all devices
void IODevice::reset() {
  unsigned long currentMicros = micros();
  for (IODevice *dev = _firstDevice; dev != NULL; dev = dev->_nextDevice) {
    dev->_deviceState = DEVSTATE_DORMANT;
    // First ensure that _loop isn't delaying 
    dev->delayUntil(currentMicros);
    // Then invoke _begin to restart driver
    dev->_begin();
  }
}

// Overarching static loop() method for the IODevice subsystem.  Works through the
// list of installed devices and calls their individual _loop() method.
// Devices may or may not implement this, but if they do it is useful for things like animations 
// or flashing LEDs.
// The current value of micros() is passed as a parameter, so the called loop function
// doesn't need to invoke it.
void IODevice::loop() {
  unsigned long currentMicros = micros();
  
  IODevice *lastLoopDevice = _nextLoopDevice;  // So we know when to stop...
  // Loop through devices until we find one ready to be serviced.
  do {
    if (!_nextLoopDevice) _nextLoopDevice = _firstDevice;
    if (_nextLoopDevice) {
      if (_nextLoopDevice->_deviceState != DEVSTATE_FAILED 
            && ((long)(currentMicros - _nextLoopDevice->_nextEntryTime)) >= 0) {
        // Found one ready to run, so invoke its _loop method.
        _nextLoopDevice->_nextEntryTime = currentMicros;
        _nextLoopDevice->_loop(currentMicros);
        _nextLoopDevice = _nextLoopDevice->_nextDevice;
        break;
      }
      // Not this one, move to next one
      _nextLoopDevice = _nextLoopDevice->_nextDevice;
    }
  } while (_nextLoopDevice != lastLoopDevice); // Stop looking when we've done all.
  
  // Report loop time if diags enabled
#if defined(DIAG_LOOPTIMES)
  unsigned long diagMicros = micros();
  static unsigned long lastMicros = 0;
  // Measure time since HAL's loop() method started.
  unsigned long halElapsed = diagMicros - currentMicros;
  // Measure time between loop() method entries (excluding this diagnostic).
  unsigned long elapsed = diagMicros - lastMicros;
  static unsigned long maxElapsed = 0, maxHalElapsed = 0;
  static unsigned long lastOutputTime = 0;
  static unsigned long halTotal = 0, total = 0;
  static unsigned long count = 0;
  const unsigned long interval = (unsigned long)5 * 1000 * 1000; // 5 seconds in microsec

  // Ignore long loop counts while message is still outputting (~3 milliseconds)
  if (currentMicros - lastOutputTime > 3000UL) {
    if (elapsed > maxElapsed) maxElapsed = elapsed;
    if (halElapsed > maxHalElapsed) maxHalElapsed = halElapsed;
    halTotal += halElapsed;
    total += elapsed;
    count++;
  }
  if (diagMicros - lastOutputTime > interval) {
    if (lastOutputTime > 0) 
      DIAG(F("Loop Total:%lus (%lus max) HAL:%lus (%lus max)"), 
        total/count, maxElapsed, halTotal/count, maxHalElapsed);
    maxElapsed = maxHalElapsed = total = halTotal = count = 0;
    lastOutputTime = diagMicros;
  }
  // Read microsecond count after calculations, so they aren't
  // included in the overall timings.
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

// Return the status of the device att vpin.
uint8_t IODevice::getStatus(VPIN vpin) {
  IODevice *dev = findDevice(vpin);
  if (!dev) return false;
  return dev->_deviceState;
}

// check whether the pin supports notification.  If so, then regular _read calls are not required.
bool IODevice::hasCallback(VPIN vpin) {
  IODevice *dev = findDevice(vpin);
  if (!dev) return false;
  return dev->_hasCallback;
}

// Display (to diagnostics) details of the device.
void IODevice::_display() {
  DIAG(F("Unknown device Vpins:%u-%u %S"), 
    (int)_firstVpin, (int)_firstVpin+_nPins-1, _deviceState==DEVSTATE_FAILED ? F("OFFLINE") : F(""));
}

// Find device associated with nominated Vpin and pass configuration values on to it.
//   Return false if not found.
bool IODevice::configure(VPIN vpin, ConfigTypeEnum configType, int paramCount, int params[]) {
  IODevice *dev = findDevice(vpin);
  if (dev) return dev->_configure(vpin, configType, paramCount, params);
#ifdef DIAG_IO
  DIAG(F("IODevice::configure(): VPIN %u not found!"), (int)vpin);
#endif
  return false;
}

// Read value from virtual pin.
int IODevice::read(VPIN vpin) {
  for (IODevice *dev = _firstDevice; dev != 0; dev = dev->_nextDevice) {
    if (dev->owns(vpin)) 
      return dev->_read(vpin);
  }
#ifdef DIAG_IO
  DIAG(F("IODevice::read(): VPIN %u not found!"), (int)vpin);
#endif
  return false;
}

// Read analogue value from virtual pin.
int IODevice::readAnalogue(VPIN vpin) {
  for (IODevice *dev = _firstDevice; dev != 0; dev = dev->_nextDevice) {
    if (dev->owns(vpin)) 
      return dev->_readAnalogue(vpin);
  }
#ifdef DIAG_IO
  DIAG(F("IODevice::readAnalogue(): VPIN %u not found!"), (int)vpin);
#endif
  return -1023;
}
int IODevice::configureAnalogIn(VPIN vpin) {
  for (IODevice *dev = _firstDevice; dev != 0; dev = dev->_nextDevice) {
    if (dev->owns(vpin)) 
      return dev->_configureAnalogIn(vpin);
  }
#ifdef DIAG_IO
  DIAG(F("IODevice::configureAnalogIn(): VPIN %u not found!"), (int)vpin);
#endif
  return -1023;
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
  DIAG(F("IODevice::write(): VPIN %u not found!"), (int)vpin);
#endif
}

// Write analogue value to virtual pin(s).  If multiple devices are allocated
// the same pin then only the first one found will be used.
//
// The significance of param1 and param2 may vary from device to device.
// For servo controllers, param1 is the profile of the transition and param2
// the duration, i.e. the time that the operation is to be animated over
// in deciseconds (0-3276 sec)
//
void IODevice::writeAnalogue(VPIN vpin, int value, uint8_t param1, uint16_t param2) {
  IODevice *dev = findDevice(vpin);
  if (dev) {
    dev->_writeAnalogue(vpin, value, param1, param2);
    return;
  }
#ifdef DIAG_IO
  DIAG(F("IODevice::writeAnalogue(): VPIN %u not found!"), (int)vpin);
#endif
}

// isBusy, when called for a device pin is always a digital output or analogue output,
//  returns input feedback state of the pin, i.e. whether the pin is busy performing
//  an animation or fade over a period of time.
bool IODevice::isBusy(VPIN vpin) {
  IODevice *dev = findDevice(vpin);
  if (dev) 
    return dev->_read(vpin);
  else
    return false;
}

void IODevice::setGPIOInterruptPin(int16_t pinNumber) {
  if (pinNumber >= 0)
    pinMode(pinNumber, INPUT_PULLUP);
  _gpioInterruptPin = pinNumber;
}

// Helper function to add a new device to the device chain.  If 
// slaveDevice is NULL then the device is added to the end of the chain.
// Otherwise, the chain is searched for slaveDevice and the new device linked
// in front of it (to support filter devices that share the same VPIN range
// as the devices they control).  If slaveDevice isn't found, then the
// device is linked to the end of the chain.
void IODevice::addDevice(IODevice *newDevice, IODevice *slaveDevice /* = NULL */) {
  if (slaveDevice == _firstDevice) {
    newDevice->_nextDevice = _firstDevice;
    _firstDevice = newDevice;
  } else {
    for (IODevice *dev = _firstDevice; dev != 0; dev = dev->_nextDevice) {
      if (dev->_nextDevice == slaveDevice || dev->_nextDevice == NULL) {
          // Link new device between dev and slaveDevice (or at end of chain)
        newDevice->_nextDevice = dev->_nextDevice;
        dev->_nextDevice = newDevice;
        break;
      }
    }
  }
  newDevice->_begin();
}

// Private helper function to locate a device by VPIN.  Returns NULL if not found.
//  This is performance-critical, so minimises the calculation and function calls necessary.
IODevice *IODevice::findDevice(VPIN vpin) { 
  for (IODevice *dev = _firstDevice; dev != 0; dev = dev->_nextDevice) {
    VPIN firstVpin = dev->_firstVpin;
    if (vpin >= firstVpin && vpin < firstVpin+dev->_nPins)
      return dev;
  }
  return NULL;
}

// Instance helper function for filter devices (layered over others).  Looks for 
//  a device that is further down the chain than the current device.
IODevice *IODevice::findDeviceFollowing(VPIN vpin) {
  for (IODevice *dev = _nextDevice; dev != 0; dev = dev->_nextDevice) {
    VPIN firstVpin = dev->_firstVpin;
    if (vpin >= firstVpin && vpin < firstVpin+dev->_nPins)
      return dev;
  }
  return NULL;
}

// Private helper function to check for vpin overlap. Run during setup only.
// returns true if pins DONT overlap with existing device
// TODO: Move the I2C address reservation and checks into the I2CManager code.
// That will enable non-HAL devices to reserve I2C addresses too.
// Silent is used by the default setup so that there is no message if the default 
// device has already been handled by the user setup.
bool IODevice::checkNoOverlap(VPIN firstPin, uint8_t nPins, 
      I2CAddress i2cAddress, bool silent) {
#ifdef DIAG_IO
  DIAG(F("Check no overlap %u %u %s"), firstPin,nPins,i2cAddress.toString());
#endif
  VPIN lastPin=firstPin+nPins-1;
  for (IODevice *dev = _firstDevice; dev != 0; dev = dev->_nextDevice) {
    
    if (nPins > 0 && dev->_nPins > 0) {
      // check for pin range overlaps (verbose but compiler will fix that)  
      VPIN firstDevPin=dev->_firstVpin;
      VPIN lastDevPin=firstDevPin+dev->_nPins-1;
      bool noOverlap= firstPin>lastDevPin || lastPin<firstDevPin;
      if (!noOverlap) {
          if (!silent) DIAG(F("WARNING HAL Overlap, redefinition of Vpins %u to %u ignored."),
              firstPin, lastPin);
          return false;
      } 
    }
    // Check for overlapping I2C address
    if (i2cAddress && dev->_I2CAddress==i2cAddress) {
      if (!silent) DIAG(F("WARNING HAL Overlap. i2c Addr %s ignored."),i2cAddress.toString());
      return false;
    } 
  }
  return true;  // no overlaps... OK to go on with constructor
}
  

//==================================================================================================================
// Static data
//------------------------------------------------------------------------------------------------------------------

// Chain of callback blocks (identifying registered callback functions for state changes)
IONotifyCallback *IONotifyCallback::first = 0;

// Start and end of chain of devices.
IODevice *IODevice::_firstDevice = 0;

// Reference to next device to be called on _loop() method.
IODevice *IODevice::_nextLoopDevice = 0;


//==================================================================================================================
// Instance members
//------------------------------------------------------------------------------------------------------------------

// Method to check whether the id corresponds to this device
bool IODevice::owns(VPIN id) {
  return (id >= _firstVpin && id < _firstVpin + _nPins);
}


#else // !defined(IO_NO_HAL)

// Minimal implementations of public HAL interface, to support Arduino pin I/O and nothing more.

void IODevice::begin() { DIAG(F("NO HAL CONFIGURED!")); }
bool IODevice::configure(VPIN pin, ConfigTypeEnum configType, int nParams, int p[]) {
  if (configType!=CONFIGURE_INPUT || nParams!=1 || pin >= NUM_DIGITAL_PINS) return false;
  #ifdef DIAG_IO
  DIAG(F("Arduino _configurePullup pin:%d Val:%d"), pin, p[0]);
  #endif
  pinMode(pin, p[0] ? INPUT_PULLUP : INPUT);
  return true;
}
void IODevice::write(VPIN vpin, int value) {
  if (vpin >= NUM_DIGITAL_PINS) return;
  digitalWrite(vpin, value);
  pinMode(vpin, OUTPUT);
}
void IODevice::writeAnalogue(VPIN, int, uint8_t, uint16_t) {}
bool IODevice::isBusy(VPIN) { return false; }
bool IODevice::hasCallback(VPIN) { return false; }
int IODevice::read(VPIN vpin) { 
  if (vpin >= NUM_DIGITAL_PINS) return 0;
  return !digitalRead(vpin);  // Return inverted state (5v=0, 0v=1)
}
int IODevice::readAnalogue(VPIN vpin) {
  return ADCee::read(vpin);
}
int IODevice::configureAnalogIn(VPIN vpin) {
  return ADCee::init(vpin);
}
void IODevice::loop() {}
void IODevice::DumpAll() {
  DIAG(F("NO HAL CONFIGURED!"));
}
bool IODevice::exists(VPIN vpin) { return (vpin > 2 && vpin < NUM_DIGITAL_PINS); }
void IODevice::setGPIOInterruptPin(int16_t) {}

// Chain of callback blocks (identifying registered callback functions for state changes)
// Not used in IO_NO_HAL but must be declared.
IONotifyCallback *IONotifyCallback::first = 0;

#endif // IO_NO_HAL


/////////////////////////////////////////////////////////////////////////////////////////////////////

// Constructor
ArduinoPins::ArduinoPins(VPIN firstVpin, int nPins) {
  _firstVpin = firstVpin;
  _nPins = nPins;
  int arrayLen = (_nPins+7)/8;
  _pinPullups = (uint8_t *)calloc(3, arrayLen);
  _pinModes = (&_pinPullups[0]) + arrayLen;
  _pinInUse = (&_pinPullups[0]) + 2*arrayLen;
  for (int i=0; i<arrayLen; i++) {
    _pinPullups[i] = 0xff;  // default to pullup on, for inputs
    _pinModes[i] = 0;
    _pinInUse[i] = 0;
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
  _pinInUse[index] |= mask;
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
    _pinInUse[index] |= mask;
  }
}

// Device-specific read function (digital input).
int ArduinoPins::_read(VPIN vpin) {
  int pin = vpin;
  uint8_t mask = 1 << ((pin-_firstVpin) % 8);
  uint8_t index = (pin-_firstVpin) / 8;
  if ((_pinModes[index] | ~_pinInUse[index]) & mask) {
    // Currently in write mode or not initialised, change to read mode
    _pinModes[index] &= ~mask;
    // Since mode changes should be infrequent, use standard pinMode function
    if (_pinPullups[index] & mask) 
      pinMode(pin, INPUT_PULLUP);
    else
      pinMode(pin, INPUT);
    _pinInUse[index] |= mask;
  }
  int value = !fastReadDigital(pin); // Invert (5v=0, 0v=1)

  #ifdef DIAG_IO
  //DIAG(F("Arduino Read Pin:%d Value:%d"), pin, value);
  #endif
  return value;
}

// Device-specific readAnalogue function (analogue input)
int ArduinoPins::_readAnalogue(VPIN vpin) {
  if (vpin > 255) return -1023;
  uint8_t pin = vpin;
  int value = ADCee::read(pin);

  #ifdef DIAG_IO
  DIAG(F("Arduino Read Pin:%d Value:%d"), pin, value);
  #endif
  return value;
}
int ArduinoPins::_configureAnalogIn(VPIN vpin) {
  if (vpin > 255) return -1023;
  uint8_t pin = vpin;
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
  int value = ADCee::init(pin);
  #ifdef DIAG_IO
  DIAG(F("configureAnalogIn Pin:%d Value:%d"), pin, value);
  #endif
  return value;
}

void ArduinoPins::_display() {
  DIAG(F("Arduino Vpins:%u-%u"), (int)_firstVpin, (int)_firstVpin+_nPins-1);
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

