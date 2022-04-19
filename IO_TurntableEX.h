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
    enum ActivityNumber : uint8_t {
      Turn = 0,             // Rotate turntable, maintain phase
      Turn_PInvert = 1,     // Rotate turntable, invert phase
      Home = 2,             // Initiate homing
      Calibrate = 3,        // Initiate calibration sequence - new feature not implemented yet
      LED_On = 4,           // Turn LED on - new feature not implemented yet
      LED_Slow = 5,         // Set LED to a slow blink - new feature not implemented yet
      LED_Fast = 6,         // Set LED to a fast blink - new feature not implemented yet
      LED_Off = 7,          // Turn LED off - new feature not implemented yet
      Acc_On = 8,           // Turn accessory pin on - new feature not implemented yet
      Acc_On = 9,           // Turn accessory pin off - new feature not implemented yet
    };
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
  uint8_t _stepperStatus;

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
    _stepperStatus = readBuffer[0];
    // DIAG(F("Turntable-EX returned status: %d"), _stepperStatus);
    delayUntil(currentMicros + 500000);  // Wait 500ms before checking again, turntables turn slowly
  }

// Read returns status as obtained in our loop.
// Return false if our status value is invalid.
  int _read(VPIN vpin) {
    if (_deviceState == DEVSTATE_FAILED) return 0;
    // DIAG(F("_read status: %d"), _stepperStatus);
    if (_stepperStatus > 1) {
      return false;
    } else {
      return _stepperStatus;
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
    _stepperStatus = 1;     // Tell the device driver Turntable-EX is busy
    I2CManager.write(_I2CAddress, 3, stepsMSB, stepsLSB, profile);
  }

// Display Turnetable-EX device driver info.
  void _display() {
    DIAG(F("TurntableEX I2C:x%x Configured on Vpins:%d-%d %S"), _I2CAddress, (int)_firstVpin, 
      (int)_firstVpin+_nPins-1, (_deviceState==DEVSTATE_FAILED) ? F("OFFLINE") : F(""));
  }

};

#endif
