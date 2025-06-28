/*
 *  © 2025 Harald Barth
 *  
 *  This file is part of CommandStation-EX
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
#ifdef ARDUINO_ARCH_ESP32
#include "DCCDecoder.h"
#include "LocoTable.h"
#include "DCCEXParser.h"
#include "DIAG.h"
#include "DCC.h"

bool DCCDecoder::parse(DCCPacket &p) {
  if (!active)
    return false;
  const byte DECODER_MOBILE = 1;
  const byte DECODER_ACCESSORY = 2;
  byte decoderType = 0; // use 0 as none
  byte *d = p.data();
  byte *instr = 0;      // will be set to point to the instruction part of the DCC packet (instr[0] to instr[n])
  uint16_t addr;        // will be set to decoder addr (long/shor mobile or accessory) 
  bool locoInfoChanged = false;

  if (d[0] ==  0B11111111) {  // Idle packet
    return false;
  }
  // CRC verification here
  byte checksum = 0;
  for (byte n = 0; n < p.len(); n++)
    checksum ^= d[n];
  if (checksum) {  // Result should be zero, if not it's an error!
    DIAG(F("Checksum error:"));
    p.print();
    return false;
  }

/*
  Serial.print("< ");
  for(int n=0; n<8; n++) {
    Serial.print(d[0]&(1<<n)?"1":"0");
  }
  Serial.println(" >");
*/
  if (bitRead(d[0],7) == 0) { // bit7 == 0 => loco short addr
    decoderType = DECODER_MOBILE;
    instr = d+1;
    addr = d[0];
  } else {
    if (bitRead(d[0],6) == 1) { // bit7 == 1 and bit6 == 1 => loco long addr
      decoderType = DECODER_MOBILE;
      instr = d+2;
      addr = 256 * (d[0] & 0B00111111) + d[1];
    } else { // bit7 == 1 and bit 6 == 0
      decoderType = DECODER_ACCESSORY;
      instr = d+1;
      addr = d[0] &  0B00111111;
    }
  }
  if (decoderType == DECODER_MOBILE) {
    switch (instr[0] & 0xE0) {
    case 0x20: // 001x-xxxx Extended commands
      if (instr[0] == 0B00111111) { // 128 speed steps
	if ((locoInfoChanged = LocoTable::updateLoco(addr, instr[1])) == true) {
	  byte speed = instr[1] & 0B01111111;
	  byte direction = instr[1] & 0B10000000;
	  DCC::setThrottle(addr, speed, direction);
	  //DIAG(F("UPDATE"));
	  // send speed change to DCCEX here
	}
      }
      break;
    case 0x40: // 010x-xxxx 28 (or 14 step) speed we assume 28
    case 0x60: // 011x-xxxx
      if ((locoInfoChanged = LocoTable::updateLoco(addr, instr[0] & 0B00111111)) == true) {
	byte speed = instr[0] & 0B00001111; // first only look at 4 bits
	if (speed > 1) {               // neither stop nor emergency stop, recalculate speed
	  speed = ((instr[0] & 0B00001111) << 1) + bitRead(instr[0], 4); // reshuffle bits
	  speed = (speed - 3) * 9/2;
	}
	byte direction = instr[0] & 0B00100000;
	DCC::setThrottle(addr, speed, direction);
      }
      break;
    case 0x80: // 100x-xxxx Function group 1
      if ((locoInfoChanged = LocoTable::updateFunc(addr, instr[0], 1)) == true) {
	byte normalized = (instr[0] << 1 & 0x1e) | (instr[0] >> 4 & 0x01);
	DCCEXParser::funcmap(addr, normalized, 0, 4);
      }
      break;
    case 0xA0: // 101x-xxxx Function group 3 and 2
    {
      byte low, high;
      if (bitRead(instr[0], 4)) {
	low = 5;
	high = 8;
      } else {
	low = 9;
	high = 12;
      }
      if ((locoInfoChanged = LocoTable::updateFunc(addr, instr[0], low)) == true) {
	DCCEXParser::funcmap(addr, instr[0], low, high);
      }
    }
    break;
    case 0xC0: // 110x-xxxx Extended (here are functions F13 and up
      switch (instr[0] & 0B00011111) {
      case 0B00011110:  // F13-F20 Function Control
	if ((locoInfoChanged = LocoTable::updateFunc(addr, instr[0], 13)) == true) {
	  DCCEXParser::funcmap(addr, instr[1], 13, 20);
	}
	if ((locoInfoChanged = LocoTable::updateFunc(addr, instr[0], 17)) == true) {
	  DCCEXParser::funcmap(addr, instr[1], 13, 20);
	}
      break;
      case 0B00011111:  // F21-F28 Function Control
	if ((locoInfoChanged = LocoTable::updateFunc(addr, instr[1], 21)) == true) {
	  DCCEXParser::funcmap(addr, instr[1], 21, 28);
	}  // updateFunc handles only the 4 low bits as that is the most common case
	if ((locoInfoChanged = LocoTable::updateFunc(addr, instr[1]>>4, 25)) == true) {
	  DCCEXParser::funcmap(addr, instr[1], 21, 28);
	}
	break;
	/* do that later
      case 0B00011000:  // F29-F36 Function Control
	break;
      case 0B00011001:  // F37-F44 Function Control
	break;
      case 0B00011010:  // F45-F52 Function Control
	break;
      case 0B00011011:  // F53-F60 Function Control
	break;
      case 0B00011100:  // F61-F68 Function Control
	break;
	*/
      }
      break;
    case 0xE0: // 111x-xxxx Config vars
      break;
    }
    return locoInfoChanged;
  }
  if (decoderType == DECODER_ACCESSORY) {
      if (instr[0] & 0B10000000) {  // Basic Accessory
	addr = (((~instr[0]) & 0B01110000) << 2) + addr;
	byte port = (instr[0] & 0B00000110) >> 1;
	byte activate = (instr[0] & 0B00001000) >> 3;
	byte coil = (instr[0] & 0B00000001);
	locoInfoChanged = true;
	//(void)addr; (void)port; (void)coil; (void)activate;
	//DIAG(F("HL=%d LL=%d C=%d A=%d"), addr, port, coil, activate);
	DCC::setAccessory(addr, port, coil, activate);
      } else { // Accessory Extended NMRA spec, do we need to decode this?
	/*
	addr = (addr << 5) +
	  ((instr[0] & 0B01110000) >> 2) +
	  ((instr[0] & 0B00000110) >> 1);
	*/
      }
    return locoInfoChanged;
  }
  return false;
}
#endif // ARDUINO_ARCH_ESP32
