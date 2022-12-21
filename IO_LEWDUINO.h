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
#ifndef IO_DNIN8_h
 #define IO_DNIN8_h
#include <Arduino.h>
#define PIN_MASK(bit) (0x80>>(bit%8))
#define GET_BIT(bit) (_pinValues[bit/8] & PIN_MASK(bit) )
#define SET_BIT(bit) _pinValues[bit/8] |= PIN_MASK(bit)
#define CLR_BIT(bit) _pinValues[bit/8] &= ~PIN_MASK(bit)
#define DIAG_IO



class IO_LEWDUINO : public IODevice {

public:
  IO_LEWDUINO(VPIN firstVpin, int nPins, 
                byte clockPin, byte latchPin, byte dataPin, 
                const byte* pinmap) :
    IODevice(firstVpin, nPins) {
 
   _latchPin=latchPin;
    _clockPin=clockPin;
    _dataPin=dataPin;
    _pinMap=pinmap;
    _nShiftBytes=(nPins+7)/8; // rounded up to multiples of 8 bits
    _pinValues=(byte*) calloc(_nShiftBytes,1);  
    // Connect to HAL so my _write, _read and _loop will be called as required.
    IODevice::addDevice(this);  
  }

// Called by HAL to start handling this device
  void _begin() override {
     _deviceState = DEVSTATE_NORMAL;
    pinMode(_latchPin,OUTPUT);
    pinMode(_clockPin,OUTPUT);
    pinMode(_dataPin,INPUT_PULLUP);
    _display();
  }

// loop called by HAL supervisor 
void _loop(unsigned long currentMicros) override {
    if (_pinMap) _loopInput(currentMicros);
    else if (_xmitPending) _loopOutput();
}

void _loopInput(unsigned long currentMicros)  {
   
   if (currentMicros-_prevMicros < POLL_MICROS) return; // Nothing to do
    _prevMicros=currentMicros;
   
    //set latch to HIGH to freeze & store parallel data
   ArduinoPins::fastWriteDigital(_latchPin, HIGH);
   delayMicroseconds(20);
   //set latch to LOW to enable the data to be transmitted serially
   ArduinoPins::fastWriteDigital(_latchPin, LOW);

  // stream in the bitmap useing mapping order provided at constructor   
  for (int xmitBit=0;xmitBit<_nShiftBytes*8; xmitBit++) {
      ArduinoPins::fastWriteDigital(_clockPin, LOW);
      delayMicroseconds(4);
      bool data = ArduinoPins::fastReadDigital(_dataPin);  
      byte map=_pinMap[xmitBit%8];
      //DIAG(F("DIN x=%d,d=%d m=%x"),xmitBit,data,map);
      if (data) _pinValues[xmitBit/8] |= map;
          else  _pinValues[xmitBit/8] &= ~map;
      ArduinoPins::fastWriteDigital(_clockPin, HIGH);    
  }
 // DIAG(F("DIN %x"),_pinValues[0]);
  }

void _loopOutput()  {
    // stream out the bitmap (highest pin first)
    _xmitPending=false; 
    ArduinoPins::fastWriteDigital(_latchPin, LOW);
    for (int xmitBit=_nShiftBytes*8 -1; xmitBit>=0; xmitBit--) {
        ArduinoPins::fastWriteDigital(_dataPin,GET_BIT(xmitBit));
        ArduinoPins::fastWriteDigital(_clockPin,HIGH);
        ArduinoPins::fastWriteDigital(_clockPin,LOW);
    }  
    digitalWrite(_latchPin, HIGH);
  }

  int _read(VPIN vpin) override {
    return GET_BIT(vpin - _firstVpin);
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

  void _display() override {
      DIAG(F("IO_LEWDUINO %SPUT Configured on VPins:%d-%d"), 
      _pinMap?F("IN"):F("OUT"),
      (int)_firstVpin, 
      (int)_firstVpin+_nPins-1);
  }

private:
  static const unsigned long POLL_MICROS=100000; // 10 / S
  unsigned long _prevMicros; 
  int  _nShiftBytes=0; 
  VPIN _latchPin,_clockPin,_dataPin;
  byte* _pinValues;
  bool _xmitPending; // Only relevant in output mode
  const byte* _pinMap;  // NULL in output mode 
};

class IO_DNIN8 {
public:
  static void create(VPIN firstVpin, int nPins, byte latchPin, byte clockPin, byte dataPin ) 
  {
      // input arrives as board pin 0,7,6,5,1,2,3,4 
      static const byte pinmap[8]={0x80,0x01,0x02,0x04,0x40,0x20,0x10,0x08};
      new IO_LEWDUINO( firstVpin,  nPins,  latchPin,  clockPin,  dataPin,pinmap);
  }

};

class IO_DNIN8K {
public:
  static void create(VPIN firstVpin, int nPins, byte clockPin, byte latchPin, byte dataPin ) 
  {
      static const byte pinmap[8]={0x80,0x01,0x02,0x04,0x40,0x20,0x10,0x08}; // TODO 
      new IO_LEWDUINO( firstVpin,  nPins, clockPin, latchPin, dataPin,pinmap);
  }
};

class IO_DNOUT8 {
public:
  static void create(VPIN firstVpin, int nPins, byte clockPin, byte latchPin, byte dataPin ) 
  {
       new IO_LEWDUINO( firstVpin,  nPins,  clockPin, latchPin,   dataPin,NULL);
  }

};
#endif