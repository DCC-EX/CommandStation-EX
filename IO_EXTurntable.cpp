/*
 *  © 2021, Peter Cole. All rights reserved.
 *
 *  This file is part of CommandStation-EX
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
* The IO_EXTurntable device driver is used to control a turntable via an Arduino with a stepper motor over I2C.
*
* The EX-Turntable code lives in a separate repo (https://github.com/DCC-EX/EX-Turntable) and contains the stepper motor logic.
*
* This device driver sends a step position to EX-Turntable to indicate the step position to move to using either of these commands:
* <D TT vpin steps activity> in the serial console
* MOVETT(vpin, steps, activity) in EX-RAIL
* Refer to the documentation for further information including the valid activities.
*/

#include "IODevice.h"
#include "I2CManager.h"
#include "DIAG.h"
#include "Turntables.h"
#include "CommandDistributor.h"

#ifndef IO_NO_HAL

void EXTurntable::create(VPIN firstVpin, int nPins, I2CAddress I2CAddress) {
  new EXTurntable(firstVpin, nPins, I2CAddress);
}

// Constructor
EXTurntable::EXTurntable(VPIN firstVpin, int nPins, I2CAddress I2CAddress) {
  _firstVpin = firstVpin;
  _nPins = nPins;
  _I2CAddress = I2CAddress;
  _stepperStatus = 0;
  _previousStatus = 0;
  addDevice(this);
}

// Initialisation of EXTurntable
void EXTurntable::_begin() {
  I2CManager.begin();
  if (I2CManager.exists(_I2CAddress)) {
    DIAG(F("EX-Turntable device found, I2C:%s"), _I2CAddress.toString());
#ifdef DIAG_IO
    _display();
#endif
  } else {
    DIAG(F("EX-Turntable I2C:%s device not found"), _I2CAddress.toString());
    _deviceState = DEVSTATE_FAILED;
  }
}

// Processing loop to obtain status of stepper
// 0 = finished moving and in correct position
// 1 = still moving
void EXTurntable::_loop(unsigned long currentMicros) {
  uint8_t readBuffer[1];
  I2CManager.read(_I2CAddress, readBuffer, 1);
  _stepperStatus = readBuffer[0];
  if (_stepperStatus != _previousStatus && _stepperStatus == 0) { // Broadcast when a rotation finishes
    if ( _currentActivity < 4) {
      _broadcastStatus(_firstVpin, _stepperStatus, _currentActivity);
    }
    _previousStatus = _stepperStatus;
  }
  delayUntil(currentMicros + 100000);  // Wait 100ms before checking again
}

// Read returns status as obtained in our loop.
// Return false if our status value is invalid.
int EXTurntable::_read(VPIN vpin) {
  if (_deviceState == DEVSTATE_FAILED) return 0;
  if (_stepperStatus > 1) {
    return false;
  } else {
    return _stepperStatus;
  }
}

// If a status change has occurred for a turntable object, broadcast it
void EXTurntable::_broadcastStatus (VPIN vpin, uint8_t status, uint8_t activity) {
  Turntable *tto = Turntable::getByVpin(vpin);
  if (tto) {
    if (activity < 4) {
      tto->setMoving(status);
      CommandDistributor::broadcastTurntable(tto->getId(), tto->getPosition(), status);
    }
  }
}

// writeAnalogue to send the steps and activity to Turntable-EX.
// Sends 3 bytes containing the MSB and LSB of the step count, and activity.
// value contains the steps, bit shifted to MSB + LSB.
// activity contains the activity flag as per this list:
// 
// Turn = 0,             // Rotate turntable, maintain phase
// Turn_PInvert = 1,     // Rotate turntable, invert phase
// Home = 2,             // Initiate homing
// Calibrate = 3,        // Initiate calibration sequence
// LED_On = 4,           // Turn LED on
// LED_Slow = 5,         // Set LED to a slow blink
// LED_Fast = 6,         // Set LED to a fast blink
// LED_Off = 7,          // Turn LED off
// Acc_On = 8,           // Turn accessory pin on
// Acc_Off = 9           // Turn accessory pin off
void EXTurntable::_writeAnalogue(VPIN vpin, int value, uint8_t activity, uint16_t duration) {
  if (_deviceState == DEVSTATE_FAILED) return;
  if (value < 0) return;
  uint8_t stepsMSB = value >> 8;
  uint8_t stepsLSB = value & 0xFF;
#ifdef DIAG_IO
  DIAG(F("EX-Turntable WriteAnalogue VPIN:%u Value:%d Activity:%d Duration:%d"),
    vpin, value, activity, duration);
  DIAG(F("I2CManager write I2C Address:%d stepsMSB:%d stepsLSB:%d activity:%d"),
    _I2CAddress.toString(), stepsMSB, stepsLSB, activity);
#endif
  if (activity < 4) _stepperStatus = 1;     // Tell the device driver Turntable-EX is busy
  _previousStatus = _stepperStatus;
  _currentActivity = activity;
  _broadcastStatus(vpin, _stepperStatus, activity); // Broadcast when the rotation starts
  I2CManager.write(_I2CAddress, 3, stepsMSB, stepsLSB, activity);
}

// Display Turnetable-EX device driver info.
void EXTurntable::_display() {
  DIAG(F("EX-Turntable I2C:%s Configured on Vpins:%u-%u %S"), _I2CAddress.toString(), (int)_firstVpin, 
    (int)_firstVpin+_nPins-1, (_deviceState==DEVSTATE_FAILED) ? F("OFFLINE") : F(""));
}

#endif
