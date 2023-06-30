/*
 *  © 2023, Peter Cole. All rights reserved.
 *  © 2022, Peter Cole. All rights reserved.
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
* The IO_RotaryEncoder device driver is used to receive positions from a rotary encoder connected to an Arduino via I2C.
*
* There is separate code required for the Arduino the rotary encoder is connected to, which is located here:
* https://github.com/peteGSX-Projects/dcc-ex-rotary-encoder
*
* This device driver receives the rotary encoder position when the rotary encoder button is pushed, and these positions
* can be tested in EX-RAIL with:
* ONCHANGE(vpin) - flag when the rotary encoder position has changed from the previous position
* IFRE(vpin, position) - test to see if specified rotary encoder position has been received
*
* Feedback can also be sent to the rotary encoder by using 2 Vpins, and sending a SET()/RESET() to the second Vpin.
* A SET(vpin) will flag that a turntable (or anything else) is in motion, and a RESET(vpin) that the motion has finished.
*
* In addition, defining a third Vpin will allow a position number to be sent so that when an EXRAIL automation or some other
* activity has moved a turntable, the position can be reflected in the rotary encoder software. This can be accomplished
* using the EXRAIL SERVO(vpin, position, profile) command, where:
* - vpin = the third defined Vpin (any other is ignored)
* - position = the defined position in the DCC-EX Rotary Encoder software, 0 (Home) to 255
* - profile = Must be defined as per the SERVO() command, but is ignored as it has no relevance
*
* Defining in myAutomation.h requires the device driver to be included in addition to the HAL() statement. Examples:
*
* #include "IO_RotaryEncoder.h"
* HAL(RotaryEncoder, 700, 1, 0x70)    // Define single Vpin, no feedback or position sent to rotary encoder software
* HAL(RotaryEncoder, 700, 2, 0x70)    // Define two Vpins, feedback only sent to rotary encoder software
* HAL(RotaryEncoder, 700, 3, 0x70)    // Define three Vpins, can send feedback and position update to rotary encoder software
*
* Refer to the documentation for further information including the valid activities and examples.
*/

#ifndef IO_ROTARYENCODER_H
#define IO_ROTARYENCODER_H

#include "EXRAIL2.h"
#include "IODevice.h"
#include "I2CManager.h"
#include "DIAG.h"

class RotaryEncoder : public IODevice {
public:
  
  static void create(VPIN firstVpin, int nPins, I2CAddress i2cAddress) {
    if (checkNoOverlap(firstVpin, nPins, i2cAddress)) new RotaryEncoder(firstVpin, nPins, i2cAddress);
  }

private:
  // Constructor
  RotaryEncoder(VPIN firstVpin, int nPins, I2CAddress i2cAddress){
    _firstVpin = firstVpin;
    _nPins = nPins;
    if (_nPins > 3) {
      _nPins = 3;
      DIAG(F("RotaryEncoder WARNING:%d vpins defined, only 3 supported"), _nPins);
    }
    _I2CAddress = i2cAddress;
    addDevice(this);
  }

  // Initiate the device
  void _begin() {
    uint8_t _status;
    // Attempt to initilalise device
    I2CManager.begin();
    if (I2CManager.exists(_I2CAddress)) {
      // Send RE_RDY, must receive RE_RDY to be online
      _sendBuffer[0] = RE_RDY;
      _status = I2CManager.read(_I2CAddress, _rcvBuffer, 1, _sendBuffer, 1);
      if (_status == I2C_STATUS_OK) {
        if (_rcvBuffer[0] == RE_RDY) {
          _sendBuffer[0] = RE_VER;
          if (I2CManager.read(_I2CAddress, _versionBuffer, 3, _sendBuffer, 1) == I2C_STATUS_OK) {
            _majorVer = _versionBuffer[0];
            _minorVer = _versionBuffer[1];
            _patchVer = _versionBuffer[2];
          }
        } else {
          DIAG(F("RotaryEncoder I2C:%s garbage received: %d"), _I2CAddress.toString(), _rcvBuffer[0]);
          _deviceState = DEVSTATE_FAILED;
          return;
        }
      } else {
        DIAG(F("RotaryEncoder I2C:%s ERROR connecting"), _I2CAddress.toString());
        _deviceState = DEVSTATE_FAILED;
        return;
      }
#ifdef DIAG_IO
      _display();
#endif
    } else {
      DIAG(F("RotaryEncoder I2C:%s device not found"), _I2CAddress.toString());
      _deviceState = DEVSTATE_FAILED;
    }
  }

  void _loop(unsigned long currentMicros) override {
    if (_deviceState == DEVSTATE_FAILED) return;  // Return if device has failed
    if (_i2crb.isBusy()) return;                  // Return if I2C operation still in progress

    if (currentMicros - _lastPositionRead > _positionRefresh) {
      _lastPositionRead = currentMicros;
      _sendBuffer[0] = RE_READ;
      I2CManager.read(_I2CAddress, _rcvBuffer, 1, _sendBuffer, 1, &_i2crb); // Read position from encoder
      _position = _rcvBuffer[0];
      // If EXRAIL is active, we need to trigger the ONCHANGE() event handler if it's in use
#if defined(EXRAIL_ACTIVE)
      if (_position != _previousPosition) {
        _previousPosition = _position;
        RMFT2::changeEvent(_firstVpin, 1);
      } else {
        RMFT2::changeEvent(_firstVpin, 0);
      }
#endif
    }
  }

  // Return the position sent by the rotary encoder software
  int _readAnalogue(VPIN vpin) override {
    if (_deviceState == DEVSTATE_FAILED) return 0;
    return _position;
  }

  // Send the feedback value to the rotary encoder software
  void _write(VPIN vpin, int value) override {
    if (vpin == _firstVpin + 1) {
      if (value != 0) value = 0x01;
      byte _feedbackBuffer[2] = {RE_OP, (byte)value};
      I2CManager.write(_I2CAddress, _feedbackBuffer, 2);
    }
  }

  // Send a position update to the rotary encoder software
  // To be valid, must be 0 to 255, and different to the current position
  // If the current position is the same, it was initiated by the rotary encoder
  void _writeAnalogue(VPIN vpin, int position, uint8_t profile, uint16_t duration) override {
    if (vpin == _firstVpin + 2) {
      if (position >= 0 && position <= 255 && position != _position) {
        byte newPosition = position & 0xFF;
        byte _positionBuffer[2] = {RE_MOVE, newPosition};
        I2CManager.write(_I2CAddress, _positionBuffer, 2);
      }
    }
  }
  
  void _display() override {
    DIAG(F("Rotary Encoder I2C:%s v%d.%d.%d Configured on VPIN:%u-%d %S"), _I2CAddress.toString(), _majorVer, _minorVer, _patchVer,
      (int)_firstVpin, _firstVpin+_nPins-1, (_deviceState==DEVSTATE_FAILED) ? F("OFFLINE") : F(""));
  }

  int8_t _position;
  int8_t _previousPosition = 0;
  uint8_t _versionBuffer[3];
  uint8_t _sendBuffer[1];
  uint8_t _rcvBuffer[1];
  uint8_t _majorVer = 0;
  uint8_t _minorVer = 0;
  uint8_t _patchVer = 0;
  I2CRB _i2crb;
  unsigned long _lastPositionRead = 0;
  const unsigned long _positionRefresh = 100000UL;    // Delay refreshing position for 100ms

  enum {
    RE_RDY = 0xA0,   // Flag to check if encoder is ready for operation
    RE_VER = 0xA1,   // Flag to retrieve rotary encoder software version
    RE_READ = 0xA2,  // Flag to read the current position of the encoder
    RE_OP = 0xA3,    // Flag for operation start/end, sent to when sending feedback on move start/end
    RE_MOVE = 0xA4,  // Flag for sending a position update from the device driver to the encoder
  };

};

#endif
