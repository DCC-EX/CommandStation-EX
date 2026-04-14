/*
 *  © 2023-2025, Chris Harlow. All rights reserved.
 *  © 2021, Neil McKechnie. All rights reserved.
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

#ifndef io_trainbrains_h
#define io_trainbrains_h

#include "IO_GPIOBase.h"
#include "FSH.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 * IODevice subclass for Trainbrains devices.
 * For details see 
 * https://trainbrains.eu/wp-content/uploads/trainbrains-railroad-modules-communication-1.2-en.pdf
 */
 
  // TODO - use non-blocking I2C 
  // TODO - support for all-channels sensor read via new command code

  
 enum TrackUnoccupancy
{
    TRACK_UNOCCUPANCY_UNKNOWN = 0,
    TRACK_OCCUPIED = 1,
    TRACK_UNOCCUPIED = 2
};

//   generic class for self-identifying Trainbrains devices
class Trainbrains : public IODevice {
public:
  static void create(VPIN vpin, uint8_t nPins, I2CAddress i2cAddress,bool debug=false) {
    if (checkNoOverlap(vpin, nPins, i2cAddress)) 
      new Trainbrains(vpin, nPins, i2cAddress,debug);
  }

private:
  static const byte DT_Unknown=0;      
  static const byte DT_Signal=1; 
  static const byte DT_Turnout=2;
  static const byte DT_Power=3; 
  static const byte DT_Track=4;
  bool debugme; 
  byte deviceType=0; // 0=none, 1=signal, 2=turnout, 3=power, 4=track
  uint8_t outputBuffer[10];
  uint8_t inputBuffer[10];
  
  // Constructor
  Trainbrains(VPIN vpin, uint8_t nPins, I2CAddress i2cAddress, bool debug) 
  { 
    debugme=debug;
    _firstVpin = vpin; 
    _nPins = nPins; 
    _I2CAddress=i2cAddress;
    outputBuffer[2]=0; // will increment with each command
    addDevice(this);
    }
  
  void issueCommand(byte code, byte p0, byte dane0=0, byte dane1=0, byte dane2=0, byte dane3=0) {
     outputBuffer[0] = (uint8_t)_I2CAddress; // strips away the mux part.
     outputBuffer[1] = code;
     outputBuffer[2] ++; // increment the command counter
     outputBuffer[3] =p0;
     outputBuffer[4] =0;
     outputBuffer[5] =0;
     outputBuffer[6] =dane0;
     outputBuffer[7] =dane1;
     outputBuffer[8] =dane2;
     outputBuffer[9] =dane3;
     memset(inputBuffer, 0, sizeof(inputBuffer)); // clear input buffer
     auto status=I2CManager.read(_I2CAddress, 
                inputBuffer, sizeof(inputBuffer),
                outputBuffer, sizeof(outputBuffer) 
               );
     if (status!=I2C_STATUS_OK) {
       DIAG(F("Trainbrains I2C:%s Error:%S"), _I2CAddress.toString(), I2CManager.getErrorMessage(status));
       _deviceState = DEVSTATE_FAILED;
     } 
     else {
      if (debugme) _dumpBuffers();
     }    
  }
  
  void _dumpBuffers() {
    StringFormatter::send(&USB_SERIAL,F("<* TB %s\n out:"),_I2CAddress.toString());
    for (byte i=0;i<sizeof(outputBuffer);i++) {
      StringFormatter::send(&USB_SERIAL,F(" %-2x"),outputBuffer[i]);
    }
    StringFormatter::send(&USB_SERIAL,F("\n  in:"));
    for (byte i=0;i<sizeof(inputBuffer);i++) {
      StringFormatter::send(&USB_SERIAL,F(" %-2x"),inputBuffer[i]);
    }
    StringFormatter::send(&USB_SERIAL,F("\n*>\n"));    
  }

  void _begin() override {
    issueCommand(20,0); // read device type
    deviceType=inputBuffer[6]; // 0=notFound 1=signal, 2=turnout, 3=power, 4=track                   
    if (deviceType == DT_Unknown) { // device not found
      _deviceState = DEVSTATE_FAILED;
     }
    else {
      issueCommand(20,3); // read channels supported
      if (_nPins>inputBuffer[6]) _nPins=inputBuffer[6]; // max channels supported
    }
    _display();
  }
  
  // Display details of this device.
  void _display() {
    DIAG(F("Trainbrains type:%d I2C:%s Configured on Vpins:%u-%u %S"), deviceType, _I2CAddress.toString(), (int)_firstVpin, 
        (int)_firstVpin+_nPins-1, (_deviceState==DEVSTATE_FAILED) ? F("OFFLINE") : F(""));
  }


  // Digital Write will be from a power set or a turnout set
  void _write(VPIN vpin,int value) override { 
    if (deviceType!=DT_Turnout && deviceType!=DT_Power) return;
    int16_t pin=vpin-_firstVpin;
    issueCommand(12,pin+1,value?2:1); // write command    
  }

  // Read will be from a sensor poll
  int _read(VPIN vpin) override {
    if (deviceType!=DT_Track) return 0;
    byte pin=vpin-_firstVpin;
    issueCommand(14,pin+1); // read channel status 
    return (inputBuffer[6] == TRACK_OCCUPIED );
  }

  // analog write is for signal mask and state.
  // parameters are arranged to be compatible with using
  // neopixel commands where R,G,B values are mapped to the 
  // aspect, flashing and state values.

  void _writeAnalogue(VPIN vpin, int value, uint8_t param1=0, uint16_t param2=0) override {
    (void)param2;
    if (deviceType!=DT_Signal) return;
    int16_t pin=vpin-_firstVpin;
    issueCommand(13,pin+1,value>>8,value & 0xff,param1); // write command    
  }
  
};

// class retained for backward compatibility. Builds the generic class. 
class Trainbrains02 {
  public:
  static void create(VPIN vpin, uint8_t nPins, I2CAddress i2cAddress, bool debug=false) {
    Trainbrains::create(vpin, nPins, i2cAddress,debug);
  }
};

#endif
