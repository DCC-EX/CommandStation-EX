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
  static uint16_t slotCount;


  // Member veriables here are arranged to reduce padding waste
  uint32_t functions;        // DCC function map
  uint32_t snifferFunctions; // sniffer function map
  uint32_t momentum_base;    // millis() when speed modified under momentum
  LocoSlot* next;
  
  // DCC data for this loco 
  uint16_t loco;             // DCC loco id
  uint16_t blockOccupied;    // railcom detected block 
  
  byte targetSpeed;       // speed set by throttle
  byte savedSpeed;
  byte speedCode;         // current DCC speed and direction
  byte snifferSpeedCode;  // sniffer speed and direction
  byte momentumA;         // momentum accelerating
  byte momentumD;         // momentum decelerating
  byte groupFlags;        // function groups acivated
  
  // SNIFFER data for each loco exists to allow sniffer to detect and ignore 
  // sniffed reminders for locos that have been taken over
  // by DCCEX. These bytes will be dropped on a Mega.
  
public: 
  // set chainModified true when the chain of locos is modified
  // so that reminders can restart from the beginning.
  static bool chainModified;
  
  static LocoSlot * getFirst(){return firstSlot;}
  static void forgetAll();
 
  static LocoSlot * getSlot(uint16_t locoId, bool autoCreate);
  static void dumpTable(Print *output);

  LocoSlot * getNext() {return next;}
  uint16_t getLoco() { return loco; }
  byte getSnifferSpeedCode() { return snifferSpeedCode; }
  unsigned long getSnifferFunctions() { return snifferFunctions; }
  void setSnifferSpeedCode(byte v) { snifferSpeedCode=v; }
  void setSnifferFunctions(unsigned long v) { snifferFunctions=v; }

  
  byte getMomentumA() { return momentumA; }
  byte getMomentumD() { return momentumD; }
  void setMomentumA(byte v) { momentumA=v; }
  void setMomentumD(byte v) { momentumD=v; }
  uint32_t getMomentumBase() { return momentum_base; }
  void setMomentumBase(uint32_t v) { momentum_base=v; } 
  byte getTargetSpeed() { return targetSpeed; }
  void setTargetSpeed(byte v) { targetSpeed=v; }  
  byte getSavedSpeed() { return savedSpeed; }
  void setSavedSpeed(byte v) { savedSpeed=v; }

  byte getSpeedCode() { return speedCode; }
  void setSpeedCode(byte v) { speedCode=v; }
  byte getGroupFlags() { return groupFlags; }
  void setGroupFlags(byte v) { groupFlags=v; }
  uint32_t getFunctions() { return functions; }
  void setFunctions(uint32_t v) { functions=v; }
  uint16_t getBlockOccupied() { return blockOccupied; }
  void setBlockOccupied(uint16_t v) { blockOccupied=v; }
  void forget();
  void saveSpeed();

};
#endif