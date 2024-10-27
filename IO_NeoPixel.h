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
* HAL(NEOPIXEL,first vpin, number of pixels,mode, i2c address) 
* e.g. HAL(NEOPIXEL,1000,64,NEO_RGB,0x60)
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


// The following macros to define the Neopixel String type
// have been copied from the Adafruit Seesaw Library under the 
// terms of the GPL. 
// Credit to: https://github.com/adafruit/Adafruit_Seesaw

// The order of primary colors in the NeoPixel data stream can vary
// among device types, manufacturers and even different revisions of
// the same item.  The third parameter to the seesaw_NeoPixel
// constructor encodes the per-pixel byte offsets of the red, green
// and blue primaries (plus white, if present) in the data stream --
// the following #defines provide an easier-to-use named version for
// each permutation.  e.g. NEO_GRB indicates a NeoPixel-compatible
// device expecting three bytes per pixel, with the first byte
// containing the green value, second containing red and third
// containing blue.  The in-memory representation of a chain of
// NeoPixels is the same as the data-stream order; no re-ordering of
// bytes is required when issuing data to the chain.

// Bits 5,4 of this value are the offset (0-3) from the first byte of
// a pixel to the location of the red color byte.  Bits 3,2 are the
// green offset and 1,0 are the blue offset.  If it is an RGBW-type
// device (supporting a white primary in addition to R,G,B), bits 7,6
// are the offset to the white byte...otherwise, bits 7,6 are set to
// the same value as 5,4 (red) to indicate an RGB (not RGBW) device.
// i.e. binary representation:
// 0bWWRRGGBB for RGBW devices
// 0bRRRRGGBB for RGB

// RGB NeoPixel permutations; white and red offsets are always same
// Offset:         W          R          G          B
#define NEO_RGB ((0 << 6) | (0 << 4) | (1 << 2) | (2))
#define NEO_RBG ((0 << 6) | (0 << 4) | (2 << 2) | (1))
#define NEO_GRB ((1 << 6) | (1 << 4) | (0 << 2) | (2))
#define NEO_GBR ((2 << 6) | (2 << 4) | (0 << 2) | (1))
#define NEO_BRG ((1 << 6) | (1 << 4) | (2 << 2) | (0))
#define NEO_BGR ((2 << 6) | (2 << 4) | (1 << 2) | (0))

// RGBW NeoPixel permutations; all 4 offsets are distinct
// Offset:         W          R          G          B
#define NEO_WRGB ((0 << 6) | (1 << 4) | (2 << 2) | (3))
#define NEO_WRBG ((0 << 6) | (1 << 4) | (3 << 2) | (2))
#define NEO_WGRB ((0 << 6) | (2 << 4) | (1 << 2) | (3))
#define NEO_WGBR ((0 << 6) | (3 << 4) | (1 << 2) | (2))
#define NEO_WBRG ((0 << 6) | (2 << 4) | (3 << 2) | (1))
#define NEO_WBGR ((0 << 6) | (3 << 4) | (2 << 2) | (1))

#define NEO_RWGB ((1 << 6) | (0 << 4) | (2 << 2) | (3))
#define NEO_RWBG ((1 << 6) | (0 << 4) | (3 << 2) | (2))
#define NEO_RGWB ((2 << 6) | (0 << 4) | (1 << 2) | (3))
#define NEO_RGBW ((3 << 6) | (0 << 4) | (1 << 2) | (2))
#define NEO_RBWG ((2 << 6) | (0 << 4) | (3 << 2) | (1))
#define NEO_RBGW ((3 << 6) | (0 << 4) | (2 << 2) | (1))

#define NEO_GWRB ((1 << 6) | (2 << 4) | (0 << 2) | (3))
#define NEO_GWBR ((1 << 6) | (3 << 4) | (0 << 2) | (2))
#define NEO_GRWB ((2 << 6) | (1 << 4) | (0 << 2) | (3))
#define NEO_GRBW ((3 << 6) | (1 << 4) | (0 << 2) | (2))
#define NEO_GBWR ((2 << 6) | (3 << 4) | (0 << 2) | (1))
#define NEO_GBRW ((3 << 6) | (2 << 4) | (0 << 2) | (1))

#define NEO_BWRG ((1 << 6) | (2 << 4) | (3 << 2) | (0))
#define NEO_BWGR ((1 << 6) | (3 << 4) | (2 << 2) | (0))
#define NEO_BRWG ((2 << 6) | (1 << 4) | (3 << 2) | (0))
#define NEO_BRGW ((3 << 6) | (1 << 4) | (2 << 2) | (0))
#define NEO_BGWR ((2 << 6) | (3 << 4) | (1 << 2) | (0))
#define NEO_BGRW ((3 << 6) | (2 << 4) | (1 << 2) | (0))

// If 400 KHz support is enabled, the third parameter to the constructor
// requires a 16-bit value (in order to select 400 vs 800 KHz speed).
// If only 800 KHz is enabled (as is default on ATtiny), an 8-bit value
// is sufficient to encode pixel color order, saving some space.

#define NEO_KHZ800 0x0000 // 800 KHz datastream
#define NEO_KHZ400 0x0100 // 400 KHz datastream

/////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 * IODevice subclass for NeoPixel.
 */

class NeoPixel : public IODevice {
public:
  
  static void create(VPIN vpin, int nPins, uint16_t mode=(NEO_GRB | NEO_KHZ800), I2CAddress i2cAddress=0x60) {
    if (checkNoOverlap(vpin, nPins, mode, i2cAddress)) new NeoPixel(vpin, nPins, mode, i2cAddress);
  }

private:
  
  static const byte SEESAW_NEOPIXEL_BASE=0x0E;
  static const byte SEESAW_NEOPIXEL_STATUS = 0x00;
  static const byte SEESAW_NEOPIXEL_PIN = 0x01;
  static const byte SEESAW_NEOPIXEL_SPEED = 0x02;
  static const byte SEESAW_NEOPIXEL_BUF_LENGTH = 0x03;
  static const byte SEESAW_NEOPIXEL_BUF=0x04;
  static const byte SEESAW_NEOPIXEL_SHOW=0x05;

  // all adafruit examples say this pin. Presumably its hard wired 
  // in the adapter anyway. 
  static const byte SEESAW_PIN15 = 15;
  
  // Constructor
  NeoPixel(VPIN firstVpin, int nPins, uint16_t mode, I2CAddress i2cAddress) {
    _firstVpin = firstVpin;
    _nPins=nPins;
    _I2CAddress = i2cAddress;
    
    // calculate the offsets into the seesaw buffer for each colour depending
    // on the pixel strip type passed in mode.

    _redOffset=4+(mode >> 4 & 0x03);
    _greenOffset=4+(mode >> 2 & 0x03); 
    _blueOffset=4+(mode & 0x03); 
    if (4+(mode >>6 & 0x03) == _redOffset) _bytesPerPixel=3; 
    else _bytesPerPixel=4; // string has a white byte.
    
    _kHz800=(mode & NEO_KHZ400)==0;
    _showPendimg=false;
    
    // Each pixel requires 3 bytes RGB memory.
    // Although the driver device can remember this, it cant do off/on without
    // forgetting what the on colour was!
    pixelBuffer=(RGB *) malloc(_nPins*sizeof(RGB)); 
    stateBuffer=(byte *) calloc((_nPins+7)/8,sizeof(byte)); // all pixels off  
    if (pixelBuffer==nullptr || stateBuffer==nullptr) {
      DIAG(F("NeoPixel I2C:%s not enough RAM"), _I2CAddress.toString());
      return;
    }
    // preset all pins to white so a digital on/off will do something even if no colour set.
    memset(pixelBuffer,0xFF,_nPins*sizeof(RGB));
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
    
    byte speedBuffer[]={SEESAW_NEOPIXEL_BASE, SEESAW_NEOPIXEL_SPEED,_kHz800};
    I2CManager.write(_I2CAddress, speedBuffer, sizeof(speedBuffer));
    
    // In the driver there are 3 of 4 byts per pixel
    auto numBytes=_bytesPerPixel * _nPins; 
    byte setbuffer[] = {SEESAW_NEOPIXEL_BASE, SEESAW_NEOPIXEL_BUF_LENGTH,
                  (byte)(numBytes >> 8), (byte)(numBytes & 0xFF)};
    I2CManager.write(_I2CAddress, setbuffer, sizeof(setbuffer));
    
    const byte pinbuffer[] = {SEESAW_NEOPIXEL_BASE, SEESAW_NEOPIXEL_PIN,SEESAW_PIN15};
    I2CManager.write(_I2CAddress, pinbuffer, sizeof(pinbuffer));
    
    for (auto pin=0;pin<_nPins;pin++) transmit(pin);
     _display();
  }
  
 // loop called by HAL supervisor 
  void _loop(unsigned long currentMicros) override {
    (void)currentMicros;
    if (!_showPendimg) return;
    byte showBuffer[]={SEESAW_NEOPIXEL_BASE,SEESAW_NEOPIXEL_SHOW};
    I2CManager.write(_I2CAddress,showBuffer,sizeof(showBuffer));
    _showPendimg=false;
  }  
  
  
  // read back pixel on/off
  int _read(VPIN vpin) override {
    if (_deviceState == DEVSTATE_FAILED) return 0;
    return isPixelOn(vpin-_firstVpin);
  }

  // Write digital value. Sets pixel on or off
  void _write(VPIN vpin, int value) override {
    if (_deviceState == DEVSTATE_FAILED) return;
    auto pixel=vpin-_firstVpin;
    if (value) {
      if (isPixelOn(pixel)) return;
      setPixelOn(pixel);
    }
    else { // set off
      if (!isPixelOn(pixel)) return;
      setPixelOff(pixel);
     }
     transmit(pixel);
  }
   
  VPIN _writeRange(VPIN vpin,int value, int count) {
    // using write range cuts out the constant vpin to driver lookup so
    // we can update multiple pixels much faster.
    VPIN nextVpin=vpin +  (count>_nPins ? _nPins : count);
    if (_deviceState != DEVSTATE_FAILED) while(vpin<nextVpin) {
      _write(vpin,value);
      vpin++;
    }
    return nextVpin;  // next pin we cant 
  }  
  // Write analogue value.
  // The convoluted parameter mashing here is to allow passing the RGB and on/off
  // information through the generic HAL _writeAnalog interface which was originally
  // designed for servos and short integers  
  void _writeAnalogue(VPIN vpin, int colour_RG, uint8_t onoff, uint16_t colour_B) override {
    if (_deviceState == DEVSTATE_FAILED) return;
    RGB newColour={(byte)((colour_RG>>8) & 0xFF), (byte)(colour_RG & 0xFF), (byte)(colour_B & 0xFF)};
    auto pixel=vpin-_firstVpin;
    if (pixelBuffer[pixel]==newColour && isPixelOn(pixel)==(bool)onoff) return; // no change  
      
    if (onoff) setPixelOn(pixel); else setPixelOff(pixel);
    pixelBuffer[pixel]=newColour;
    transmit(pixel);
  }
 VPIN _writeAnalogueRange(VPIN vpin, int colour_RG, uint8_t onoff, uint16_t colour_B, int count) override {
    // using write range cuts out the constant vpin to driver lookup so
    VPIN nextVpin=vpin +  (count>_nPins ? _nPins : count); 
    if (_deviceState != DEVSTATE_FAILED) while(vpin<nextVpin) {
      _writeAnalogue(vpin,colour_RG, onoff,colour_B);
      vpin++;
    }
    return nextVpin;  // next pin we cant 
 }
 
  // Display device information and status.
  void _display() override {
    DIAG(F("NeoPixel I2C:%s Vpins %u-%u %S"),
              _I2CAddress.toString(), 
              (int)_firstVpin, (int)_firstVpin+_nPins-1,
              _deviceState == DEVSTATE_FAILED ? F("OFFLINE") : F(""));
  }


  
  bool isPixelOn(int16_t pixel) {return stateBuffer[pixel/8] & (0x80>>(pixel%8));}
  void setPixelOn(int16_t pixel) {stateBuffer[pixel/8] |= (0x80>>(pixel%8));}
  void setPixelOff(int16_t pixel) {stateBuffer[pixel/8] &= ~(0x80>>(pixel%8));}
  
  // Helper function for error handling
  void reportError(uint8_t status, bool fail=true) {
    DIAG(F("NeoPixel I2C:%s Error:%d (%S)"), _I2CAddress.toString(), 
      status, I2CManager.getErrorMessage(status));
    if (fail)
    _deviceState = DEVSTATE_FAILED;
  }

  
  void transmit(uint16_t pixel) { 
    byte buffer[]={SEESAW_NEOPIXEL_BASE,SEESAW_NEOPIXEL_BUF,0x00,0x00,0x00,0x00,0x00};
    uint16_t offset= pixel * _bytesPerPixel;
    buffer[2]=(byte)(offset>>8);
    buffer[3]=(byte)(offset & 0xFF);
    
    if (isPixelOn(pixel)) {
      auto colour=pixelBuffer[pixel];    
      buffer[_redOffset]=colour.red;
      buffer[_greenOffset]=colour.green;
      buffer[_blueOffset]=colour.blue;
    } // else leave buffer black (in buffer preset to zeros above)
    
    // Transmit pixel to driver
    I2CManager.write(_I2CAddress,buffer,4 +_bytesPerPixel);
    _showPendimg=true;
  
  }
  struct RGB { 
      byte red; 
      byte green; 
      byte blue; 
      bool operator==(const RGB& other) const {
        return red == other.red && green == other.green && blue == other.blue;
      }
    };

  RGB*   pixelBuffer = nullptr;
  byte*  stateBuffer = nullptr;  // 1 bit per pixel
  bool _showPendimg;
  
  // mapping of RGB onto pixel buffer for seesaw.
  byte _bytesPerPixel;
  byte _redOffset;
  byte _greenOffset;
  byte _blueOffset;
  bool _kHz800; 
};

#endif
