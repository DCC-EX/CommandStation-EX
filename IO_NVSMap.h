/*
 *  © 2026, Chris Harlow. All rights reserved.
 *  
 *  This file is part of DCC-EX API
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
#ifndef IO_NVSMap_h
#define IO_NVSMap_h
#include <Arduino.h>
#include "defines.h"
#include "IODevice.h"
#include "NVSTable.h"

/* 
NVSMap provides a set of virtual pins which map over the NVS value set 
for the purposes of reading the values in EXRAIL IF-type conditions.

For most EXRAIL commands the NVS macro proivides access to the NVS values,
 for example DELAY(NVS(17))  will delay for the number of milliseconds stored in NVS value 17.

However, for IF conditions, or other EXRAIL run time commands that reference vpins, 
the NVS macro is not appropriate because IF(NVS(22)) will NOT test the value in NVS 22 but will
test the vpin number obtained from NVS(22).

Instead, the NVSMap device provides a set of virtual pins which map over the NVS values but with an 
offset of the user's choice so that the vpin numbers dont clash.

e.g  HAL(VPINMap,10000) will map vpins 10000-100255 to NVS values 0-255.  
     Thus IF(10017) will test the value in NVS 17.

NVS is NOT suitable for writing to NVS values, only for reading them.
  Writing to NVS values is done via the <C NVS ...> command.
  This is to prevent misuse as the underlying technology is not suitable for 
frequent writes.  Writing to NVS values is only suitable for configuration changes, not for run time data.

*/     

class NVSMap : public IODevice {

public:

  //CODE CAUTION: the nPins parameter must be <=255 because checkNoOverlap only accepts uint8_t
  // inconvenioent because there are 256 slots in the NVS table.
  static void create(VPIN firstVpin, uint16_t nPins=256) { 
    if (nPins>255) { // Pain due to checkNoOverlap so check each half
      uint8_t npins1=nPins/2; 
      if (!IODevice::checkNoOverlap(firstVpin,npins1)) return; 
      if (!IODevice::checkNoOverlap(firstVpin+npins1,nPins-npins1)) return; 
    }
    else if (IODevice::checkNoOverlap(firstVpin,nPins)) return; 
    new NVSMap( firstVpin, nPins);
  }

  NVSMap(VPIN firstVpin, int nPins) : IODevice(firstVpin, nPins) {
    // Connect to HAL so my _write, _read and _loop will be called as required.
    IODevice::addDevice(this);
  }

// Called by HAL to start handling this device
  void _begin() override {
    _display();
  }

  int _read(VPIN vpin) override {
    int pin=vpin - _firstVpin;
    return NVSTable::getNVS((uint8_t)pin) ? 1:0; // this is digital read
  }

  int _readAnalogue(VPIN vpin) override {
    int pin=vpin - _firstVpin;
    return NVSTable::getNVS((uint8_t)pin); // this is analog read  
  }

  void _display() override {
      DIAG(F("NVSMap Configured on Vpins:%u-%u"), 
      (int)_firstVpin, 
      (int)_firstVpin+_nPins-1);
  }
};
#endif
