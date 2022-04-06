/*
Standard license stuff goes here.
*/

/*
* The IO_TurntableEX device driver is used to control a turntable via an Arduino Nano with a stepper motor over I2C.
*
* The Nano code lives in a separate repo and contains the stepper motor logic, as well as the configuration for
* the various turntable positions.
*
* This device driver sends an integer/byte to the Nano to indicate the position to move to using an EX-RAIL SERVO
* command, with the position in place of the PWM value. The profile value is effectively ignored.
*
* For example, a ROUTE used for position one:
*
* ROUTE(600, "Layout connection")
*   SERVO(600, 1, Instant)
*   DONE
*
* All code below is "hacked up" from existing device drivers and based on SERVO using writeAnalogue and is the only
* way I could get it to work. The "value" that is sent turns out to be the number of bytes sent, so the Nano code
* simply counts the number of bytes as the position.
*
* I believe the correct mechanism here should be to send a byte containing the integer of the position, which the Nano
* receives and responds to with another byte once the stepper has stopped moving.
*
* In addition, I think it would be handy to have a Vpin defined per position as a sensor input, which the Nano activates
* once the move is complete. That would enable automation sequences via EX-RAIL with (AT) type commands.
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

  void _begin() {
    I2CManager.begin();
    I2CManager.setClock(1000000);
    if (I2CManager.exists(_I2CAddress)) {
      // What here?
#ifdef DIAG_IO
      _display();
#endif
    } else {
      _deviceState = DEVSTATE_FAILED;
    }
  }

  void _writeAnalogue(VPIN vpin, int value, uint8_t profile, uint16_t duration) {
    if (_deviceState == DEVSTATE_FAILED) return;
#ifdef DIAG_IO
    DIAG(F("TurntableEX WriteAnalogue Vpin:%d Value:%d Profile:%d Duration:%d"),
      vpin, value, profile, duration);
#endif
    I2CManager.write(_I2CAddress, 1, value);
  }

  void _display() {
    DIAG(F("TurntableEX I2C:x%x Configured on Vpins:%d-%d %S"), _I2CAddress, (int)_firstVpin, 
      (int)_firstVpin+_nPins-1, (_deviceState==DEVSTATE_FAILED) ? F("OFFLINE") : F(""));
  }

};

#endif
