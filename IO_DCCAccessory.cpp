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

#define PACKEDADDRESS(addr, subaddr) (((addr) << 2)  + (subaddr))
#define ADDRESS(packedaddr) ((packedaddr) >> 2)
#define SUBADDRESS(packedaddr) ((packedaddr) % 4)

void DCCAccessoryDecoder::create(VPIN firstVpin, int nPins, int DCCAddress, int DCCSubaddress) {
  if (checkNoOverlap(firstVpin,nPins)) new DCCAccessoryDecoder(firstVpin, nPins, DCCAddress, DCCSubaddress);
}

// Constructors
DCCAccessoryDecoder::DCCAccessoryDecoder(VPIN vpin, int nPins, int DCCAddress, int DCCSubaddress) {
   _firstVpin = vpin;
  _nPins = nPins;
  _packedAddress = PACKEDADDRESS(DCCAddress, DCCSubaddress);
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
#if defined(HAL_ACCESSORY_COMMAND_REVERSE)
  state = !state;
#ifdef DIAG_IO
  DIAG(F("DCC Write Linear Address:%d State:%d (inverted)"), packedAddress, state);
#endif
#else
#ifdef DIAG_IO
  DIAG(F("DCC Write Linear Address:%d State:%d"), packedAddress, state);
#endif
#endif
  DCC::setAccessory(ADDRESS(packedAddress), SUBADDRESS(packedAddress), state);
}

void DCCAccessoryDecoder::_display() {
  int endAddress = _packedAddress + _nPins - 1;
  DIAG(F("DCCAccessoryDecoder Configured on Vpins:%u-%u Addresses %d/%d-%d/%d)"), _firstVpin, _firstVpin+_nPins-1,
      ADDRESS(_packedAddress), SUBADDRESS(_packedAddress), ADDRESS(endAddress), SUBADDRESS(endAddress));
}

