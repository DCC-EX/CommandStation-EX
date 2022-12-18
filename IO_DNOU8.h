/*
 *  © 2022, Chris Harlow. All rights reserved.
 *  
 *  This file is part of DCC++EX API
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
#ifndef IO_DNOU8_h
 #define IO_DNOU8_h
#include <Arduino.h>
#include "IODevice.h"
#define GET_BIT(bit) ((_pinValues[bit/8] >>(bit%8) == 1))
#define SET_BIT(bit) _pinValues[bit/8] |= 1<<(bit%8)
#define CLR_BIT(bit) _pinValues[bit/8] &= ~(1<<(bit%8))

class IO_DNOU8 : public IODevice {
public:
  static void create(VPIN firstVpin, int nPins, byte latchPin, byte clockPin, byte dataPin ) 
  {
      new IO_DNOU8( firstVpin,  nPins,  latchPin,  clockPin,  dataPin);
  }  

protected:
  IO_DNOU8(VPIN firstVpin, int nPins, byte latchPin, byte clockPin, byte dataPin) :
    IODevice(firstVpin, nPins) {
    _latchPin=latchPin;
    _clockPin=clockPin;
    _dataPin=dataPin;
    _nShiftBytes=(nPins+7)/8; // rounded up to multiples of 8 bits
    _pinValues=(byte*) calloc(_nShiftBytes,1);  
    // Connect to HAL so my _write, _read and _loop will be called as required.
    IODevice::addDevice(this);  
  }

// Called by HAL to start handling this device
  void _begin() override {
    _xmitPending=true; // will cause transmission of all zeros first time out
    _deviceState = DEVSTATE_NORMAL;
    pinMode(_latchPin,OUTPUT);
    pinMode(_clockPin,OUTPUT);
    pinMode(_dataPin,OUTPUT);
    _display();
  }

void _loop(unsigned long currentMicros) override {
   (void)currentMicros;  // Not needed

   if (!_xmitPending) return; // Nothing to do

    #ifdef DIAG_IO
        // We will create a DIAG on the fly
        USB_SERIAL.print("<* DNOU8 ");
    #endif

    // stream out the bitmap (highest pin first)
    _xmitPending=false; 
    digitalWrite(_latchPin, LOW);
    
    for (int xmitBit=_nShiftBytes*8 -1; xmitBit>=0; xmitBit--) {
        bool bit=GET_BIT(xmitBit);
    #ifdef DIAG_IO
        USB_SERIAL.print(bit?'1':'0');
    #endif
        digitalWrite(_dataPin,bit);
        digitalWrite(_clockPin,HIGH);
        digitalWrite(_clockPin,LOW);
    }
    
    digitalWrite(_latchPin, HIGH);
    #ifdef DIAG_IO
        USB_SERIAL.print(" *>\n");
    #endif           
  }

  void _write(VPIN vpin, int value) override {
    int pin = vpin - _firstVpin;
    bool oldval=GET_BIT(pin);
    bool newval=value!=0;
    if (newval==oldval) return; // no change  
    if (newval) SET_BIT(pin);
           else CLR_BIT(pin);
    _xmitPending=true;  // shift register will be sent on next _loop()
  }

  int _read(VPIN vpin) override {
    int pin = vpin - _firstVpin;
    return GET_BIT(pin);
  }

  void _display() override {
      DIAG(F("IO_DNOU8 Configured on VPins:%d-%d"), (int)_firstVpin, 
    (int)_firstVpin+_nPins-1);
  }

private:
  bool _xmitPending=false;
  int  _nShiftBytes=0; 
  VPIN _latchPin,_clockPin,_dataPin;
  byte* _pinValues;
};

#endif