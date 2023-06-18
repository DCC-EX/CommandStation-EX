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
  // Constructor
  RotaryEncoder(VPIN firstVpin, int nPins, I2CAddress i2cAddress){
    _firstVpin = firstVpin;
    _nPins = nPins;
    _I2CAddress = i2cAddress;
    addDevice(this);
  }
  static void create(VPIN firstVpin, int nPins, I2CAddress i2cAddress) {
    if (checkNoOverlap(firstVpin, nPins, i2cAddress)) new RotaryEncoder(firstVpin, nPins, i2cAddress);
  }

private:
  // Initiate the device
  void _begin() {
    I2CManager.begin();
    if (I2CManager.exists(_I2CAddress)) {
      byte _getVersion[1] = {RE_VER};
      I2CManager.read(_I2CAddress, _versionBuffer, 3, _getVersion, 1);
      _majorVer = _versionBuffer[0];
      _minorVer = _versionBuffer[1];
      _patchVer = _versionBuffer[2];
      _buffer[0] = RE_OP;
      I2CManager.write(_I2CAddress, _buffer, 1);
#ifdef DIAG_IO
      _display();
#endif
    } else {
        _deviceState = DEVSTATE_FAILED;
    }
  }

  void _loop(unsigned long currentMicros) override {
    I2CManager.read(_I2CAddress, _buffer, 1);
    _position = _buffer[0];
    // This here needs to have a change check, ie. position is a different value.
  #if defined(EXRAIL_ACTIVE)
      if (_position != _previousPosition) {
        _previousPosition = _position;
        RMFT2::changeEvent(_firstVpin,1);
      } else {
        RMFT2::changeEvent(_firstVpin,0);
      }
  #endif
    delayUntil(currentMicros + 100000);
  }

  // Device specific read function
  int _readAnalogue(VPIN vpin) override {
    if (_deviceState == DEVSTATE_FAILED) return 0;
    return _position;
  }

  void _write(VPIN vpin, int value) override {
    if (vpin == _firstVpin + 1) {
      if (value != 0) value = 0x01;
      byte _feedbackBuffer[2] = {RE_OP, (byte)value};
      I2CManager.write(_I2CAddress, _feedbackBuffer, 2);
    }
  }

  void _writeAnalogue(VPIN vpin, int position, uint8_t profile, uint16_t duration) override {
    if (vpin == _firstVpin + 2) {
      if (position >= 0 && position <= 255) {
        byte _positionBuffer[2] = {RE_MOVE, position};
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
  uint8_t _buffer[1];
  uint8_t _majorVer = 0;
  uint8_t _minorVer = 0;
  uint8_t _patchVer = 0;

  enum {
    RE_VER = 0xA0,   // Flag to retrieve rotary encoder version from the device
    RE_OP = 0xA1,    // Flag for normal operation
    RE_MOVE = 0xA2,  // Flag for sending a position update
  };

};

#endif
