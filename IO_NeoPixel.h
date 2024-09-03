/*
 *  © 2024, Chris Harlow. All rights reserved.
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
* The IO_NEOPIXEL.h device driver integrates with one or more Adafruit neopixel drivers.
* This device driver will configure the device on startup, along with
* interacting with the device for all input/output duties.
*
* To create NEOPIXEL devices, these are defined in myAutomation.h:
* (Note the device driver is included by default)
*
* HAL(NEOPIXEL,first vpin, number of pixels, i2c address) 
* e.g. HAL(NEOPIXEL,1000,64,0x60)
* This gives each pixel in the chain an individual vpin
* The number of pixels must match the physical pixels in the chain. 
* 
* This driver maintains a colour (rgb value in 5,5,5 bits only) plus an ON bit.
* This can be written/read with an analog write/read call. 
* The ON bit can be set on and off with a digital write. This allows for 
* a pixel to be preset a colour and then turned on and off like any other light. 
*/

#ifndef IO_EX_NeoPixel_H
#define IO_EX_NeoPixel_H

#include "IODevice.h"
#include "I2CManager.h"
#include "DIAG.h"
#include "FSH.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 * IODevice subclass for NeoPixel.
 */
class NeoPixel : public IODevice {
public:

  
  static void create(VPIN vpin, int nPins, I2CAddress i2cAddress) {
    if (checkNoOverlap(vpin, nPins, i2cAddress)) new NeoPixel(vpin, nPins, i2cAddress);
  }

private:
  static const uint16_t NEOPIXEL_ON_FLAG=0x0001; 
  
  // Constructor
  NeoPixel(VPIN firstVpin, int nPins, I2CAddress i2cAddress) {
    _firstVpin = firstVpin;
    _I2CAddress = i2cAddress;
    addDevice(this);
  }

  void _begin() {
    // Initialise Neopixel device
    I2CManager.begin();
    if (!I2CManager.exists(_I2CAddress)) {
      DIAG(F("NeoPixel I2C:%s device not found"), _I2CAddress.toString());
      _deviceState = DEVSTATE_FAILED;
      return;
    }

    // TODO - initialize the neopixel board

    pixelBuffer=(uint16_t *) calloc(_nPins,sizeof(uint16_t)); // all pixels off  
     _display();
  }
  

  // read back pixel colour (rarely needed I suspect)
  int _readAnalogue(VPIN vpin) override {
    if (_deviceState == DEVSTATE_FAILED) return 0;
    auto pin=vpin-_firstVpin;
    return pixelBuffer[pin];
  }

  // read back pixel on/off
  int _read(VPIN vpin) override {
    if (_deviceState == DEVSTATE_FAILED) return 0;
    auto pin=vpin-_firstVpin;
    return pixelBuffer[pin] & NEOPIXEL_ON_FLAG;
  }

  // Write digital value. Sets pixel on or off
  void _write(VPIN vpin, int value) override {
    if (_deviceState == DEVSTATE_FAILED) return;
    auto pin=vpin-_firstVpin;
    if (value) {
      if (pixelBuffer[pin] & NEOPIXEL_ON_FLAG) return;
      pixelBuffer[pin] |= NEOPIXEL_ON_FLAG;
    }
    else { // set off
      if (!(pixelBuffer[pin] & NEOPIXEL_ON_FLAG)) return;
      pixelBuffer[pin] &= (~NEOPIXEL_ON_FLAG);
     }
     transmit(pin);
  }

  // Write analogue (integer) value
  void _writeAnalogue(VPIN vpin, int colour, uint8_t ignore1, uint16_t ignore2) override {
    (void) ignore1;
    (void) ignore2;
    auto newColour=(uint16_t)colour;
    if (_deviceState == DEVSTATE_FAILED) return;
    auto pin=vpin-_firstVpin;
    if (pixelBuffer[pin]==newColour) return;
      pixelBuffer[pin]=newColour;
      transmit(pin);  
  }

  // Display device information and status.
  void _display() override {
    DIAG(F("NeoPixel I2C:%s Vpins %u-%u %S"),
              _I2CAddress.toString(), 
              (int)_firstVpin, (int)_firstVpin+_nPins-1,
              _deviceState == DEVSTATE_FAILED ? F("OFFLINE") : F(""));
  }

  // Helper function for error handling
  void reportError(uint8_t status, bool fail=true) {
    DIAG(F("NeoPixel I2C:%s Error:%d (%S)"), _I2CAddress.toString(), 
      status, I2CManager.getErrorMessage(status));
    if (fail)
    _deviceState = DEVSTATE_FAILED;
  }

  void transmit(uint16_t pin) {
    auto colour=pixelBuffer[pin];
    if (colour & NEOPIXEL_ON_FLAG) {
      // TODO convert 5,5,5 to RGB 
    } 
    else {
      // TODO set black
    }
    // TODO transmit pixel to driver
  }
  uint16_t*  pixelBuffer = nullptr;
  
};

#endif
