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
* The IO_TurntableEX device driver is used to control a turntable via an Arduino Nano with a stepper motor over I2C.
*
* The Nano code lives in a separate repo (https://github.com/DCC-EX/Turntable-EX) and contains the stepper motor logic.
*
* This device driver sends an integer/byte to the Nano to indicate the position to move to using an EX-RAIL SERVO
* command, with the position in place of the PWM value. The profile value is used to flag phase/polarity switching
* or other activities as defined in Turntable-EX.
*
* For example, a ROUTE used for position one:
*
* ROUTE(600, "Layout connection")
*   SERVO(600, 1, Instant)
*   DONE
*/

#ifndef IO_TurntableEX_h
#define IO_TurntableEX_h

#include "IODevice.h"
#include "I2CManager.h"
#include "DIAG.h"

class TurntableEX : public IODevice {

public:
  static void create(VPIN firstVpin, int nPins, uint8_t I2CAddress) {
    new TurntableEX(firstVpin, nPins, I2CAddress);
  }

  // Constructor
  TurntableEX(VPIN firstVpin, int nPins, uint8_t I2CAddress) {
    _firstVpin = firstVpin;
    _nPins = nPins;
    _I2CAddress = I2CAddress;
    addDevice(this);
  }

private:
  uint8_t _I2CAddress;
  uint16_t numSteps;
  uint8_t stepperStatus;

// Initialisation of TurntableEX
  void _begin() {
    I2CManager.begin();
    I2CManager.setClock(1000000);
    if (I2CManager.exists(_I2CAddress)) {
#ifdef DIAG_IO
      _display();
#endif
    } else {
      _deviceState = DEVSTATE_FAILED;
    }
  }

// Processing loop to obtain status of stepper
// 0 = finished moving and in correct position
// 1 = still moving
// 2 = finished moving, in incorrect position
  void _loop(unsigned long currentMicros) {
    uint8_t readBuffer[1];
    I2CManager.read(_I2CAddress, readBuffer, 1);
    stepperStatus = readBuffer[0];
    delayUntil(currentMicros + 100000);  // Wait 100ms before checking again
  }

// Read returns status as obtained in our loop.
// Return false if our status value is invalid.
  int _read(VPIN vpin) {
    if (_deviceState == DEVSTATE_FAILED) return 0;
    if (stepperStatus > 2) {
      return false;
    } else {
      return stepperStatus;
    }
  }

// writeAnalogue to send the steps and activity to Turntable-EX.
// Sends 3 bytes containing the MSB and LSB of the step count, and activity.
// value contains the steps, bit shifted to MSB + LSB.
// profile contains the activity.
  void _writeAnalogue(VPIN vpin, int value, uint8_t profile, uint16_t duration) {
    if (_deviceState == DEVSTATE_FAILED) return;
    uint8_t stepsMSB = value >> 8;
    uint8_t stepsLSB = value & 0xFF;
#ifdef DIAG_IO
    DIAG(F("TurntableEX WriteAnalogue Vpin:%d Value:%d Profile:%d Duration:%d"),
      vpin, value, profile, duration);
    DIAG(F("I2CManager write I2C Address:%d stepsMSB:%d stepsLSB:%d profile:%d"),
      _I2CAddress, stepsMSB, stepsLSB, profile);
#endif
    I2CManager.write(_I2CAddress, 3, stepsMSB, stepsLSB, profile);
  }

// Display Turnetable-EX device driver info.
  void _display() {
    DIAG(F("TurntableEX I2C:x%x Configured on Vpins:%d-%d %S"), _I2CAddress, (int)_firstVpin, 
      (int)_firstVpin+_nPins-1, (_deviceState==DEVSTATE_FAILED) ? F("OFFLINE") : F(""));
  }

};

#endif
