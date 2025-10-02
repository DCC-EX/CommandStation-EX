/* Copyright (c) 2023 Harald Barth
 * Copyright (c) 2025 Chris Harlow
 *
 * This source is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This source is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/* this represents a slot in the loco lookup table where
various loco-specific values are stored.
*/
#ifndef LocoSlot_h
#define LocoSlot_h  
#include <Arduino.h>
const byte MOMENTUM_USE_DEFAULT=255;

class LocoSlot {
private:
  LocoSlot() {};
  void prepare(uint16_t locoId);
  
  static LocoSlot* firstSlot;
  static LocoSlot* recycler;
  LocoSlot* next;
  
  // DCC data for this loco 
  uint16_t loco;
  byte speedCode;
  byte groupFlags;
  uint32_t functions;

  // Momentum management variables
  uint32_t momentum_base;     // millis() when speed modified under momentum
  byte momentumA, momentumD;
  byte targetSpeed;           // speed set by throttle
  
  // RAILCOM block occupancy flag
  uint16_t blockOccupied; // railcom detected block 
  
  // SNIFFER data for this loco.
  // This exists to allow sniffer to detect and ignore 
  // sniffed reminders for locos that have been taken over
  // by DCCEX.

  byte snifferSpeedCode;
  byte snifferGroupFlags;
  unsigned long snifferFunctions;

  // sniffer statistics 
  uint16_t snifferFunccounter;
  uint16_t snifferSpeedcounter;
  
public: 

  static LocoSlot * getFirst(){return firstSlot;}
  static void forgetAll();
 
  static LocoSlot * getSlot(uint16_t locoId, bool autoCreate);
  static void dumpTable(Print *output);

  LocoSlot * getNext() {return next;}
  uint16_t getLoco() { return loco; }
  byte getSnifferSpeedCode() { return snifferSpeedCode; }
  byte getSnifferGroupFlags() { return snifferGroupFlags; }
  unsigned long getSnifferFunctions() { return snifferFunctions; }
  uint16_t getSnifferFunccounter() { return snifferFunccounter; }
  uint16_t getSnifferSpeedcounter() { return snifferSpeedcounter; }
  void setSnifferSpeedCode(byte v) { snifferSpeedCode=v; }
  void setSnifferGroupFlags(byte v) { snifferGroupFlags=v; }
  void setSnifferFunctions(unsigned long v) { snifferFunctions=v; }
  void setSnifferFunccounter(unsigned int v) { snifferFunccounter=v; }
  void setSnifferSpeedcounter(unsigned int v) { snifferSpeedcounter=v; }  
  void incrementSnifferFunccounter() { snifferFunccounter++; }
  void incrementSnifferSpeedcounter() { snifferSpeedcounter++; }  
  
  
  byte getMomentumA() { return momentumA; }
  byte getMomentumD() { return momentumD; }
  void setMomentumA(byte v) { momentumA=v; }
  void setMomentumD(byte v) { momentumD=v; }
  uint32_t getMomentumBase() { return momentum_base; }
  void setMomentumBase(uint32_t v) { momentum_base=v; } 
  byte getTargetSpeed() { return targetSpeed; }
  void setTargetSpeed(byte v) { targetSpeed=v; }  

  byte getSpeedCode() { return speedCode; }
  void setSpeedCode(byte v) { speedCode=v; }
  byte getGroupFlags() { return groupFlags; }
  void setGroupFlags(byte v) { groupFlags=v; }
  uint32_t getFunctions() { return functions; }
  void setFunctions(uint32_t v) { functions=v; }
  uint16_t getBlockOccupied() { return blockOccupied; }
  void setBlockOccupied(uint16_t v) { blockOccupied=v; }
  void forget();

};
#endif