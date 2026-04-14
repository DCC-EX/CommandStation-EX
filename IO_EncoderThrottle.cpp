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
* The IO_EncoderThrottle device driver uses a rotary encoder connected to vpins
* to drive a loco.
*  Loco id is selected by writeAnalog.
*/

#include "IODevice.h"
#include "IO_EncoderThrottle.h"
#include "DIAG.h"
#include "DCC.h"

const byte _DIR_CW = 0x10;  // Clockwise step
const byte _DIR_CCW = 0x20;  // Counter-clockwise step

const byte transition_table[5][4]= {
    {0,1,3,0},            // 0: 00
    {1,1,1,2 | _DIR_CW},  // 1: 00->01
    {2,2,0,2},            // 2: 00->01->11
    {3,3,3,4 | _DIR_CCW}, // 3: 00->10
    {4,0,4,4}             // 4: 00->10->11
};

const byte _STATE_MASK = 0x07;
const byte _DIR_MASK = 0x30;



  void EncoderThrottle::create(VPIN firstVpin, int dtPin, int clkPin, int clickPin, byte notch) {
    if (checkNoOverlap(firstVpin)) new EncoderThrottle(firstVpin, dtPin,clkPin,clickPin,notch);
  }


  // Constructor
  EncoderThrottle::EncoderThrottle(VPIN firstVpin, int dtPin, int clkPin, int clickPin, byte notch){
    _firstVpin = firstVpin;
    _nPins = 1;
    _I2CAddress = 0;
    _dtPin=dtPin;
    _clkPin=clkPin;
    _clickPin=clickPin;
    _notch=notch;
    _locoid=0;
    _stopState=xrSTOP;
    _rocoState=0; 
    _prevpinstate=4; // not 01..11 
    IODevice::configureInput(dtPin,true);
    IODevice::configureInput(clkPin,true);
    IODevice::configureInput(clickPin,true);
    addDevice(this);
    _display();
  }

  

  void EncoderThrottle::_loop(unsigned long currentMicros)  {
    (void) currentMicros; // suppress warning

    if (_locoid==0) return;  // not in use
    
    // Clicking down on the roco, stops the loco and sets the direction as unknown.
  if (IODevice::read(_clickPin)) {
    if (_stopState==xrSTOP) return; // debounced multiple stops
    DCC::setThrottle(_locoid,1,DCC::getThrottleDirection(_locoid));
    _stopState=xrSTOP;
    DIAG(F("DRIVE %d STOP"),_locoid);
    return; 
  }

  // read roco pins and detect state change 
  byte pinstate = (IODevice::read(_dtPin) << 1) | IODevice::read(_clkPin);
  if (pinstate==_prevpinstate) return;
  _prevpinstate=pinstate;

  _rocoState = transition_table[_rocoState & _STATE_MASK][pinstate];
  if ((_rocoState & _DIR_MASK) == 0) return; // no value change 

  int change=(_rocoState & _DIR_CW)?+1:-1;
  // handle roco change -1 or +1 (clockwise)
  
  if (_stopState==xrSTOP) {
      // first move after button press sets the direction. (clockwise=fwd)
      _stopState=change>0?xrFWD:xrREV;
    }

    // when going fwd, clockwise increases speed. 
    // but when reversing, anticlockwise increases speed.
    // This is similar to a center-zero pot control but with
    // the added safety that you cant panic-spin into the other
    // direction.
    if (_stopState==xrREV) change=-change; 
    //  manage limits
    int oldspeed=DCC::getThrottleSpeed(_locoid);
    if (oldspeed==1)oldspeed=0; // break out of estop
    int  newspeed=change>0 ?  (min((oldspeed+_notch),126)) : (max(0,(oldspeed-_notch)));
    if (newspeed==1) newspeed=0; // normal decelereated stop. 
    if (oldspeed!=newspeed) {
        DIAG(F("DRIVE %d notch %S %d %S"),_locoid,
             change>0?F("UP"):F("DOWN"),_notch,
             _stopState==xrFWD?F("FWD"):F("REV"));
        DCC::setThrottle(_locoid,newspeed,_stopState==xrFWD);
        }
}

  // Set locoid as analog value to start drive
  // use <z vpin locoid [notch]>
  void EncoderThrottle::_writeAnalogue(VPIN vpin, int value, uint8_t param1, uint16_t param2)  {  
    (void)vpin; // not used, but needed to match IODevice interface
    (void) param2;
    _locoid=value;
    if (param1>0) _notch=param1;
    _rocoState=0;
    
    // If loco is moving, we inherit direction from it.
    _stopState=xrSTOP;
    if (_locoid>0) {
        auto speedbyte=DCC::getThrottleSpeedByte(_locoid);
        if ((speedbyte & 0x7f) >1) {
            // loco is moving
            _stopState= (speedbyte & 0x80)?xrFWD:xrREV;
        } 
    }
    _display();
  }

  
  void EncoderThrottle::_display() {
    DIAG(F("DRIVE vpin %d loco %d notch %d"),_firstVpin,_locoid,_notch);
  }
