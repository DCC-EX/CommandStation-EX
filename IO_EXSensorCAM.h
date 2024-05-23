/*  12/MAY/24
 *  © 2022, Peter Cole. All rights reserved.
 *  © 2023, Barry Daniel ESP32 revision
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
 * The IO_EXSensorCAM.h device driver can integrate with the sensorCAM device.
 * This device driver will configure the device on startup, along with
 * interacting with the device for all input/output duties.
 *
 * To create EX-SensorCAM devices, define them in myAutomation.h:
 *
 *   // HAL(EXSensorCAM,vpin, num_vpins, i2c_address);
 *   HAL(EXSensorCAM,700, 80, 0x11)
 *
 * The total number of pins cannot exceed 80  because of the communications
 * packet format. The number of analogue inputs cannot exceed 16 because of a
 * limit on the maximum I2C packet size of 32 bytes (in the Wire library).
 */

#ifndef IO_EX_EXSENSORCAM_H
#define IO_EX_EXSENSORCAM_H

#include "DIAG.h"
#include "FSH.h"
#include "I2CManager.h"
#include "IODevice.h"
#include "CamParser.h"
/////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 * IODevice subclass for EX-SensorCAM.
 */
class EXSensorCAM : public IODevice {
 public:
  static void create(VPIN vpin, int nPins, I2CAddress i2cAddress) {
    if (checkNoOverlap(vpin, nPins, i2cAddress))
      new EXSensorCAM(vpin, nPins, i2cAddress);
  }
  static VPIN CAMBaseVpin;
 private:
  // Constructor
  EXSensorCAM(VPIN firstVpin, int nPins, I2CAddress i2cAddress) {
    _firstVpin = firstVpin;
    if (CAMBaseVpin==0) CAMBaseVpin=_firstVpin;
    // Number of pins cannot exceed 80.
    if (nPins > 80) nPins = 80;
    _nPins = nPins;
    _I2CAddress = i2cAddress;
    addDevice(this);
  }

  void _begin() {
    // Initialise EX-SensorCAM device
    I2CManager.begin();

    if (!I2CManager.exists(_I2CAddress)) {
      DIAG(F("EX-SensorCAM I2C:%s device not found"), _I2CAddress.toString());
      _deviceState = DEVSTATE_FAILED;
      return;
    }

    _i2crb.setRequestParams(_I2CAddress, _inputBuffer, sizeof(_inputBuffer),
                            _outputBuffer, sizeof(_outputBuffer));
    _outputBuffer[0]='V';
    _i2crb.writeLen=1;
    I2CManager.queueRequest(&_i2crb);
     _deviceState = DEVSTATE_SCANNING;
  }


  // Main loop, collect any input and reissue poll cmd
  void _loop(unsigned long currentMicros) override {
    if (_deviceState == DEVSTATE_FAILED) return;  // If device failed, return

    if (_i2crb.isBusy()) return;  // If I2C operation still in progress, return

    processInput();
    if (_deviceState == DEVSTATE_FAILED) return;  // If device failed, return


    // is it time to request another set of pins?
    if (currentMicros - _lastDigitalRead > _digitalRefresh) return;

    // Issue new read request for digital states.
    _lastDigitalRead = currentMicros;
    _outputBuffer[0] = 'Q';  // query sensor states
    _i2crb.writeLen = 1;
    I2CManager.queueRequest(&_i2crb);
    _deviceState = DEVSTATE_SCANNING;
  }

  // Obtain the a block of 8 sensors as an analog value.
  // can be used to track poisition in sequention pin blocks
  int _readAnalogue(VPIN vpin) override {
    if (_deviceState == DEVSTATE_FAILED) return 0;
    return _digitalInputStates[(vpin - _firstVpin) / 8];
  }

  // Obtain a sensor state
  int _read(VPIN vpin) override {
    if (_deviceState == DEVSTATE_FAILED) return 0;
    int pin = vpin - _firstVpin;
    return bitRead(_digitalInputStates[pin / 8], pin % 8);
  }
   
  // Used to write a command from the parseN function.
  // note parameter names changed to suit use in this scenario
  // but the function signature remians as per IO_DEVICE
  //   _writeAnalogue(VPIN vpin, int value, uint8_t profile, uint16_t duration) override {
  void _writeAnalogue(VPIN vpin, int p1,    uint8_t opcode,  uint16_t p2) override {
    if (_deviceState == DEVSTATE_FAILED) return;  // If device failed, return
    _i2crb.wait();
    // opcodes are lower case as-entered by user through CamParser.
    // except as follows: (to avoid adding more base functions to IO_DEVICE)
    // <N a bsno row col> is passed in here as opcode=128+bsNo, p1=row, p2=col
    //  so p0 is reverse engineered from the opcode 
    // <N vpin row col> is passed with opcode 'A' p1=row, p2=col
    // and is converted to 'a' and the P0 bsNo obtained from the VPIN.   
    uint16_t p0 =0;
    
    if (opcode>=128) { //  <N a bsNo row col>
       p0=opcode & 0x7f; // get bSno from opcode
       opcode='a';       // and revert to correct code  
    }
    else if (opcode=='A') { // <N vpin row col> 
       p0=vpin - _firstVpin;
       opcode='a';
    }

    _outputBuffer[0]=opcode;
    _outputBuffer[1]=p0 & 0xFF;
    _outputBuffer[2]=p0 >> 8;
    _outputBuffer[3]=p1 & 0xFF;
    _outputBuffer[4]=(p1 >> 8) & 0xFF;
    _outputBuffer[5]=p2 & 0xFF;
    _outputBuffer[6]=p2 >> 8;
    I2CManager.queueRequest(&_i2crb);
  } 

  // This function is invoked when an I/O operation on the requestBlock
  // completes.
  void processInput() {
    if (_deviceState != DEVSTATE_SCANNING) return;
    // some input was pending
    uint8_t status = _i2crb.status;
    if (status != I2C_STATUS_OK) {
      reportError(status);
      return;
    }
    
    _deviceState = DEVSTATE_NORMAL;
  
    if (_inputBuffer[0] == 'Q') {  // all sensors
      memcpy(_digitalInputStates, _inputBuffer + 1,
             sizeof(_digitalInputStates));
      return;
    }
    if (_inputBuffer[0] == 'V') {  // version response
      memcpy(_version, _inputBuffer + 1, sizeof(_version));
      _display();
      return;
    }
  }

  // Display device information and status.
  void _display() override {
    DIAG(F("EX-SensorCAM I2C:%s v%d.%d.%d Vpins %u-%u %S"),
         _I2CAddress.toString(), _version[0], _version[1], _version[2],
         (int)_firstVpin, (int)_firstVpin + _nPins - 1,
         _deviceState == DEVSTATE_FAILED ? F("OFFLINE") : F(""));
  }

  // Helper function for error handling
  void reportError(uint8_t status, bool fail = true) {
    DIAG(F("EX-SensorCAM I2C:%s Error:%d (%S)"), _I2CAddress.toString(), status,
         I2CManager.getErrorMessage(status));
    if (fail) _deviceState = DEVSTATE_FAILED;
  }

  uint8_t _numDigitalPins = 80;
  uint8_t _version[3];
  uint8_t _digitalInputStates[10];
  I2CRB _i2crb;
  byte _inputBuffer[12];
  byte _outputBuffer[8];

  unsigned long _lastDigitalRead = 0;
  const unsigned long _digitalRefresh =
      10000UL;  // Delay refreshing digital inputs for 10ms
};

#endif
