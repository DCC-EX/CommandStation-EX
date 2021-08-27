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
#include "defines.h"

// Note: For DCC Accessory Decoders, a particular output can be specified by 
// a linear address, or by an address/subaddress pair, where the subaddress is
// in the range 0 to 3 and specifies an output within a group of 4.
// NMRA and DCC++EX accepts addresses in the range 0-511.  Linear addresses
// are not specified by the NMRA and so different manufacturers may calculate them
// in different ways.  DCC++EX uses a range of 1-2044 which excludes decoder address 0.
// Linear address 1 corresponds to address 1 subaddress 0.

#define LINEARADDRESS(addr, subaddr) (((addr-1) << 2)  + subaddr + 1)
#define ADDRESS(linearaddr) (((linearaddr-1) >> 2) + 1)
#define SUBADDRESS(linearaddr) ((linearaddr-1) % 4)

void DCCAccessoryDecoder::create(VPIN vpin, int nPins, int DCCAddress, int DCCSubaddress) {
  new DCCAccessoryDecoder(vpin, nPins, DCCAddress, DCCSubaddress);
}

// Constructors
DCCAccessoryDecoder::DCCAccessoryDecoder(VPIN vpin, int nPins, int DCCAddress, int DCCSubaddress) {
   _firstVpin = vpin;
  _nPins = nPins;
  _packedAddress = LINEARADDRESS(DCCAddress, DCCSubaddress);
  addDevice(this);
}

void DCCAccessoryDecoder::_begin() {
#if defined(DIAG_IO)
  _display();
#endif
}

// Device-specific write function.  State 1=closed, 0=thrown.  Adjust for RCN-213 compliance
void DCCAccessoryDecoder::_write(VPIN id, int state) {
  int packedAddress = _packedAddress + id - _firstVpin;
  #ifdef DIAG_IO
  DIAG(F("DCC Write Linear Address:%d State:%d"), packedAddress, state);
  #endif
#if !defined(DCC_ACCESSORY_RCN_213)
  state = !state;
#endif
  DCC::setAccessory(ADDRESS(packedAddress), SUBADDRESS(packedAddress), state);
}

void DCCAccessoryDecoder::_display() {
  int endAddress = _packedAddress + _nPins - 1;
  DIAG(F("DCCAccessoryDecoder Configured on Vpins:%d-%d Linear Address:%d-%d (%d/%d-%d/%d)"), _firstVpin, _firstVpin+_nPins-1,
      _packedAddress, _packedAddress+_nPins-1,
      ADDRESS(_packedAddress), SUBADDRESS(_packedAddress), ADDRESS(endAddress), SUBADDRESS(endAddress));
}

