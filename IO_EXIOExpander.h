/*
 *  © 2022, Peter Cole. All rights reserved.
 *  © 2024, Harald Barth. All rights reserved.
 *
 *  This file is part of EX-CommandStation
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
* The IO_EXIOExpander.h device driver integrates with one or more EX-IOExpander devices.
* This device driver will configure the device on startup, along with
* interacting with the device for all input/output duties.
*
* To create EX-IOExpander devices, these are defined in myAutomation.h:
* (Note the device driver is included by default)
*
* HAL(EXIOExpander,800,18,0x65)
* 
* All pins on an EX-IOExpander device are allocated according to the pin map for the specific
* device in use. There is no way for the device driver to sanity check pins are used for the
* correct purpose, however the EX-IOExpander device's pin map will prevent pins being used
* incorrectly (eg. A6/7 on Nano cannot be used for digital input/output).
*
* The total number of pins cannot exceed 256 because of the communications packet format.
* The number of analogue inputs cannot exceed 16 because of a limit on the maximum
* I2C packet size of 32 bytes (in the Wire library).
*/

#ifndef IO_EX_IOEXPANDER_H
#define IO_EX_IOEXPANDER_H

#include "IODevice.h"
#include "I2CManager.h"
#include "DIAG.h"
#include "FSH.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 * IODevice subclass for EX-IOExpander.
 */
class EXIOExpander : public IODevice {
public:

  enum ProfileType : uint8_t {
    Instant = 0,  // Moves immediately between positions (if duration not specified)
    UseDuration = 0, // Use specified duration
    Fast = 1,     // Takes around 500ms end-to-end
    Medium = 2,   // 1 second end-to-end
    Slow = 3,     // 2 seconds end-to-end
    Bounce = 4,   // For semaphores/turnouts with a bit of bounce!!
    NoPowerOff = 0x80, // Flag to be ORed in to suppress power off after move.
  };

  static void create(VPIN vpin, int nPins, I2CAddress i2cAddress) {
    if (checkNoOverlap(vpin, nPins, i2cAddress)) new EXIOExpander(vpin, nPins, i2cAddress);
  }

private:
  // Constructor
  EXIOExpander(VPIN firstVpin, int nPins, I2CAddress i2cAddress) {
    _firstVpin = firstVpin;
    // Number of pins cannot exceed 256 (1 byte) because of I2C message structure.
    if (nPins > 256) nPins = 256;
    _nPins = nPins;
    _I2CAddress = i2cAddress;
    addDevice(this);
  }

  void _begin() {
    uint8_t status;
    // Initialise EX-IOExander device
    I2CManager.begin();
    if (I2CManager.exists(_I2CAddress)) {
      // Send config, if EXIOPINS returned, we're good, setup pin buffers, otherwise go offline
      // NB The I2C calls here are done as blocking calls, as they're not time-critical
      // during initialisation and the reads require waiting for a response anyway.
      // Hence we can allocate I/O buffers from the stack.
      uint8_t receiveBuffer[3];
      uint8_t commandBuffer[4] = {EXIOINIT, (uint8_t)_nPins, (uint8_t)(_firstVpin & 0xFF), (uint8_t)(_firstVpin >> 8)};
      status = I2CManager.read(_I2CAddress, receiveBuffer, sizeof(receiveBuffer), commandBuffer, sizeof(commandBuffer));
      if (status == I2C_STATUS_OK) {
        if (receiveBuffer[0] == EXIOPINS) {
          _numDigitalPins = receiveBuffer[1];
          _numAnaloguePins = receiveBuffer[2];

          // See if we already have suitable buffers assigned
          if (_numDigitalPins>0) {
            size_t digitalBytesNeeded = (_numDigitalPins + 7) / 8;
            if (_digitalPinBytes < digitalBytesNeeded) {
              // Not enough space, free any existing buffer and allocate a new one
              if (_digitalPinBytes > 0) free(_digitalInputStates);
              if ((_digitalInputStates = (byte*) calloc(digitalBytesNeeded, 1)) != NULL) {
                _digitalPinBytes = digitalBytesNeeded;
              } else {
                DIAG(F("EX-IOExpander I2C:%s ERROR alloc %d bytes"), _I2CAddress.toString(), digitalBytesNeeded);
                _deviceState = DEVSTATE_FAILED;
                _digitalPinBytes = 0;
                return;
              }
            }
          }
          
          if (_numAnaloguePins>0) {
            size_t analogueBytesNeeded = _numAnaloguePins * 2;
            if (_analoguePinBytes < analogueBytesNeeded) {
              // Free any existing buffers and allocate new ones.
              if (_analoguePinBytes > 0) {
                free(_analogueInputBuffer);
                free(_analogueInputStates);
                free(_analoguePinMap);
              }
              _analogueInputStates = (uint8_t*) calloc(analogueBytesNeeded, 1);
              _analogueInputBuffer = (uint8_t*) calloc(analogueBytesNeeded, 1);
              _analoguePinMap = (uint8_t*) calloc(_numAnaloguePins, 1);
	      if (_analogueInputStates  != NULL &&
		  _analogueInputBuffer != NULL &&
		  _analoguePinMap != NULL) {
		_analoguePinBytes = analogueBytesNeeded;
	      } else {
		DIAG(F("EX-IOExpander I2C:%s ERROR alloc analog pin bytes"), _I2CAddress.toString());
		_deviceState = DEVSTATE_FAILED;
		_analoguePinBytes = 0;
		return;
	      }
            }
          }
        } else {
          DIAG(F("EX-IOExpander I2C:%s ERROR configuring device"), _I2CAddress.toString());
          _deviceState = DEVSTATE_FAILED;
          return;
        }
      } 
      // We now need to retrieve the analogue pin map if there are analogue pins
      if (status == I2C_STATUS_OK && _numAnaloguePins>0) {
        commandBuffer[0] = EXIOINITA;
        status = I2CManager.read(_I2CAddress, _analoguePinMap, _numAnaloguePins, commandBuffer, 1);
      }
      if (status == I2C_STATUS_OK) {
        // Attempt to get version, if we don't get it, we don't care, don't go offline
        uint8_t versionBuffer[3];
        commandBuffer[0] = EXIOVER;
        if (I2CManager.read(_I2CAddress, versionBuffer, sizeof(versionBuffer), commandBuffer, 1) == I2C_STATUS_OK) {
          _majorVer = versionBuffer[0];
          _minorVer = versionBuffer[1];
          _patchVer = versionBuffer[2];
        }
        DIAG(F("EX-IOExpander device found, I2C:%s, Version v%d.%d.%d"),
            _I2CAddress.toString(), _majorVer, _minorVer, _patchVer);

#ifdef DIAG_IO
        _display();
#endif
      }
      if (status != I2C_STATUS_OK)
        reportError(status);

    } else {
      DIAG(F("EX-IOExpander I2C:%s device not found"), _I2CAddress.toString());
      _deviceState = DEVSTATE_FAILED;
    }
  }

  // Digital input pin configuration, used to enable on EX-IOExpander device and set pullups if requested.
  // Configuration isn't done frequently so we can use blocking I2C calls here, and so buffers can
  // be allocated from the stack to reduce RAM allocation.
  bool _configure(VPIN vpin, ConfigTypeEnum configType, int paramCount, int params[]) override {
    if (paramCount != 1) return false;
    int pin = vpin - _firstVpin;
    if (configType == CONFIGURE_INPUT) {
      uint8_t pullup = params[0];
      uint8_t outBuffer[] = {EXIODPUP, (uint8_t)pin, pullup};
      uint8_t responseBuffer[1];
      uint8_t status = I2CManager.read(_I2CAddress, responseBuffer, sizeof(responseBuffer),
                                outBuffer, sizeof(outBuffer));
      if (status == I2C_STATUS_OK) {
        if (responseBuffer[0] == EXIORDY) {
          return true;
        } else {
          DIAG(F("EXIOVpin %u cannot be used as a digital input pin"), (int)vpin);
        }
      } else
        reportError(status);
    } else if (configType == CONFIGURE_ANALOGINPUT) {
      // TODO:  Consider moving code from _configureAnalogIn() to here and remove _configureAnalogIn
      // from IODevice class definition.  Not urgent, but each virtual function defined
      // means increasing the RAM requirement of every HAL device driver, whether it's relevant
      // to the driver or not.
      return false;
    }
    return false;
  }

  // Analogue input pin configuration, used to enable an EX-IOExpander device.
  // Use I2C blocking calls and allocate buffers from stack to save RAM.
  int _configureAnalogIn(VPIN vpin) override {
    int pin = vpin - _firstVpin;
    uint8_t commandBuffer[] = {EXIOENAN, (uint8_t)pin};
    uint8_t responseBuffer[1];
    uint8_t status = I2CManager.read(_I2CAddress, responseBuffer, sizeof(responseBuffer),
                                  commandBuffer, sizeof(commandBuffer));
    if (status == I2C_STATUS_OK) {
      if (responseBuffer[0] == EXIORDY) {
        return true;
      } else {
        DIAG(F("EX-IOExpander: Vpin %u cannot be used as an analogue input pin"), (int)vpin);
      }
    } else
      reportError(status);

    return false;
  }

  // Main loop, collect both digital and analogue pin states continuously (faster sensor/input reads)
  void _loop(unsigned long currentMicros) override {
    if (_deviceState == DEVSTATE_FAILED) return;    // If device failed, return

    // Request block is used for analogue and digital reads from the IOExpander, which are performed
    // on a cyclic basis.  Writes are performed synchronously as and when requested.

    if (_readState != RDS_IDLE) {
      if (_i2crb.isBusy()) return;                // If I2C operation still in progress, return

      uint8_t status = _i2crb.status;
      if (status == I2C_STATUS_OK) {             // If device request ok, read input data

        // First check if we need to process received data
        if (_readState == RDS_ANALOGUE) {
          // Read of analogue values was in progress, so process received values
          // Here we need to copy the values from input buffer to the analogue value array.  We need to 
          // do this to avoid tearing of the values (i.e. one byte of a two-byte value being changed
          // while the value is being read).
          memcpy(_analogueInputStates, _analogueInputBuffer, _analoguePinBytes); // Copy I2C input buffer to states

        } else if (_readState == RDS_DIGITAL) {
          // Read of digital states was in progress, so process received values 
          // The received digital states are placed directly into the digital buffer on receipt, 
          // so don't need any further processing at this point (unless we want to check for
          // changes and notify them to subscribers, to avoid the need for polling - see IO_GPIOBase.h).
        }
      } else
        reportError(status, false);   // report eror but don't go offline.

      _readState = RDS_IDLE;
    }

    // If we're not doing anything now, check to see if a new input transfer is due.
    if (_readState == RDS_IDLE) {
      if (_numDigitalPins>0 && currentMicros - _lastDigitalRead > _digitalRefresh) { // Delay for digital read refresh
        // Issue new read request for digital states.  As the request is non-blocking, the buffer has to
        // be allocated from heap (object state).
        _readCommandBuffer[0] = EXIORDD;
        I2CManager.read(_I2CAddress, _digitalInputStates, (_numDigitalPins+7)/8, _readCommandBuffer, 1, &_i2crb);
                                                                // non-blocking read
        _lastDigitalRead = currentMicros;
        _readState = RDS_DIGITAL;
      } else if (_numAnaloguePins>0 && currentMicros - _lastAnalogueRead > _analogueRefresh) { // Delay for analogue read refresh
        // Issue new read for analogue input states
        _readCommandBuffer[0] = EXIORDAN;
        I2CManager.read(_I2CAddress, _analogueInputBuffer,
            _numAnaloguePins * 2, _readCommandBuffer, 1, &_i2crb);
        _lastAnalogueRead = currentMicros;
        _readState = RDS_ANALOGUE;
      }
    }
  }

  // Obtain the correct analogue input value, with reference to the analogue
  // pin map.  
  // Obtain the correct analogue input value
  int _readAnalogue(VPIN vpin) override {
    if (_deviceState == DEVSTATE_FAILED) return 0;
    int pin = vpin - _firstVpin;
    for (uint8_t aPin = 0; aPin < _numAnaloguePins; aPin++) {
      if (_analoguePinMap[aPin] == pin) {
        uint8_t _pinLSBByte = aPin * 2;
        uint8_t _pinMSBByte = _pinLSBByte + 1;
        return (_analogueInputStates[_pinMSBByte] << 8) + _analogueInputStates[_pinLSBByte];
      }
    }
    return -1;  // pin not found in table
  }

  // Obtain the correct digital input value
  int _read(VPIN vpin) override {
    if (_deviceState == DEVSTATE_FAILED) return 0;
    int pin = vpin - _firstVpin;
    uint8_t pinByte = pin / 8;
    bool value = bitRead(_digitalInputStates[pinByte], pin - pinByte * 8);
    return value;
  }

  // Write digital value.  We could have an output buffer of states, that is periodically
  // written to the device if there are any changes; this would reduce the I2C overhead
  // if lots of output requests are being made.  We could also cache the last value 
  // sent so that we don't write the same value over and over to the output.  
  // However, for the time being, we just write the current value (blocking I2C) to the
  // IOExpander node.  As it is a blocking request, we can use buffers allocated from
  // the stack to save RAM allocation.
  void _write(VPIN vpin, int value) override {
    uint8_t digitalOutBuffer[3];
    uint8_t responseBuffer[1];
    if (_deviceState == DEVSTATE_FAILED) return;
    int pin = vpin - _firstVpin;
    digitalOutBuffer[0] = EXIOWRD;
    digitalOutBuffer[1] = pin;
    digitalOutBuffer[2] = value;
    uint8_t status = I2CManager.read(_I2CAddress, responseBuffer, 1, digitalOutBuffer, 3);
    if (status != I2C_STATUS_OK) {
      reportError(status);
    } else {
      if (responseBuffer[0] != EXIORDY) {
        DIAG(F("Vpin %u cannot be used as a digital output pin"), (int)vpin);
      }
    }
  }

  // Write analogue (integer) value.  Write the parameters (blocking I2C) to the
  // IOExpander node.  As it is a blocking request, we can use buffers allocated from
  // the stack to reduce RAM allocation.
  void _writeAnalogue(VPIN vpin, int value, uint8_t profile, uint16_t duration) override {
    uint8_t servoBuffer[7];
    uint8_t responseBuffer[1];

    if (_deviceState == DEVSTATE_FAILED) return;
    int pin = vpin - _firstVpin;
#ifdef DIAG_IO
    DIAG(F("Servo: WriteAnalogue Vpin:%u Value:%d Profile:%d Duration:%d %S"), 
      vpin, value, profile, duration, _deviceState == DEVSTATE_FAILED?F("DEVSTATE_FAILED"):F(""));
#endif
    servoBuffer[0] = EXIOWRAN;
    servoBuffer[1] = pin;
    servoBuffer[2] = value & 0xFF;
    servoBuffer[3] = value >> 8;
    servoBuffer[4] = profile;
    servoBuffer[5] = duration & 0xFF;
    servoBuffer[6] = duration >> 8;
    uint8_t status = I2CManager.read(_I2CAddress, responseBuffer, 1, servoBuffer, 7);
    if (status != I2C_STATUS_OK) {
      DIAG(F("EX-IOExpander I2C:%s Error:%d %S"), _I2CAddress.toString(), status, I2CManager.getErrorMessage(status));
      _deviceState = DEVSTATE_FAILED;
    } else {
      if (responseBuffer[0] != EXIORDY) {
        DIAG(F("Vpin %u cannot be used as a servo/PWM pin"), (int)vpin);
      }
    }
  }

  // Display device information and status.
  void _display() override {
    DIAG(F("EX-IOExpander I2C:%s v%d.%d.%d Vpins %u-%u %S"),
              _I2CAddress.toString(), _majorVer, _minorVer, _patchVer,
              (int)_firstVpin, (int)_firstVpin+_nPins-1,
              _deviceState == DEVSTATE_FAILED ? F("OFFLINE") : F(""));
  }

  // Helper function for error handling
  void reportError(uint8_t status, bool fail=true) {
    DIAG(F("EX-IOExpander I2C:%s Error:%d (%S)"), _I2CAddress.toString(), 
      status, I2CManager.getErrorMessage(status));
    if (fail)
    _deviceState = DEVSTATE_FAILED;
  }

  uint8_t _numDigitalPins = 0;
  uint8_t _numAnaloguePins = 0;

  uint8_t _majorVer = 0;
  uint8_t _minorVer = 0;
  uint8_t _patchVer = 0;

  uint8_t* _digitalInputStates  = NULL;
  uint8_t* _analogueInputStates = NULL;
  uint8_t* _analogueInputBuffer = NULL;  // buffer for I2C input transfers
  uint8_t _readCommandBuffer[1];

  uint8_t _digitalPinBytes = 0;   // Size of allocated memory buffer (may be longer than needed)
  uint8_t _analoguePinBytes = 0;  // Size of allocated memory buffer (may be longer than needed)
  uint8_t* _analoguePinMap = NULL;
  I2CRB _i2crb;

  enum {RDS_IDLE, RDS_DIGITAL, RDS_ANALOGUE};  // Read operation states
  uint8_t _readState = RDS_IDLE;
  
  unsigned long _lastDigitalRead = 0;
  unsigned long _lastAnalogueRead = 0;
  const unsigned long _digitalRefresh = 10000UL;    // Delay refreshing digital inputs for 10ms
  const unsigned long _analogueRefresh = 50000UL;   // Delay refreshing analogue inputs for 50ms

  // EX-IOExpander protocol flags
  enum {
    EXIOINIT = 0xE0,    // Flag to initialise setup procedure
    EXIORDY = 0xE1,     // Flag we have completed setup procedure, also for EX-IO to ACK setup
    EXIODPUP = 0xE2,    // Flag we're sending digital pin pullup configuration
    EXIOVER = 0xE3,     // Flag to get version
    EXIORDAN = 0xE4,    // Flag to read an analogue input
    EXIOWRD = 0xE5,     // Flag for digital write
    EXIORDD = 0xE6,     // Flag to read digital input
    EXIOENAN = 0xE7,    // Flag to enable an analogue pin
    EXIOINITA = 0xE8,   // Flag we're receiving analogue pin mappings
    EXIOPINS = 0xE9,    // Flag we're receiving pin counts for buffers
    EXIOWRAN = 0xEA,   // Flag we're sending an analogue write (PWM)
    EXIOERR = 0xEF,     // Flag we've received an error
  };
};

#endif
