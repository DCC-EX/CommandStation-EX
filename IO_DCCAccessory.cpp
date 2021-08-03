/*
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

#include "DCC.h"
#include "IODevice.h"
#include "DIAG.h"

// Note: For DCC Accessory Decoders, a particular output can be specified by 
// a linear address, or by an address/subaddress pair, where the subaddress is
// in the range 0 to 3 and specifies an output within a group of 4.
// NMRA and DCC++EX accepts addresses in the range 0-511.  Linear addresses
// are not specified by the NMRA and so different manufacturers may calculate them
// in different ways.  DCC+EX uses a range of 1-2044 which excludes decoder address 0.
// Therefore, I've avoided using linear addresses here because of the ambiguities 
// involved.  Instead I've used the term 'packedAddress'.

void DCCAccessoryDecoder::create(VPIN vpin, int nPins, int DCCAddress, int DCCSubaddress) {
  new DCCAccessoryDecoder(vpin, nPins, DCCAddress, DCCSubaddress);
}

// Constructor
DCCAccessoryDecoder::DCCAccessoryDecoder(VPIN vpin, int nPins, int DCCAddress, int DCCSubaddress) {
   _firstVpin = vpin;
  _nPins = nPins;
  _packedAddress = (DCCAddress << 2) + DCCSubaddress;
  int endAddress = _packedAddress + _nPins - 1;
  DIAG(F("DCC Accessory Decoder configured Vpins:%d-%d Linear Address:%d-%d (%d/%d-%d/%d)"), _firstVpin, _firstVpin+_nPins-1,
      _packedAddress, _packedAddress+_nPins-1,
     DCCAddress, DCCSubaddress, endAddress >> 2, endAddress % 4);
}

// Device-specific write function.
void DCCAccessoryDecoder::_write(VPIN id, int state) {
  int packedAddress = _packedAddress + id - _firstVpin;
  #ifdef DIAG_IO
  DIAG(F("DCC Write Linear Address:%d State:%d"), packedAddress, state);
  #endif
  DCC::setAccessory(packedAddress >> 2, packedAddress % 4, state);
}

void DCCAccessoryDecoder::_display() {
  int endAddress = _packedAddress + _nPins - 1;
  DIAG(F("DCC Accessory Vpins:%d-%d Linear Address:%d-%d (%d/%d-%d/%d)"), _firstVpin, _firstVpin+_nPins-1,
      _packedAddress, _packedAddress+_nPins-1,
      _packedAddress >> 2, _packedAddress % 4, endAddress >> 2, endAddress % 4);
}

