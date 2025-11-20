/* Copyright (c) 2023 Harald Barth
 * Copyright (c) 2025 Chris Harlow
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
#include "LocoSlot.h"
#include "StringFormatter.h"
#include "DIAG.h"
LocoSlot * LocoSlot::firstSlot = nullptr;
LocoSlot * LocoSlot::recycler = nullptr;
bool LocoSlot::chainModified = false;
uint16_t LocoSlot::slotCount = 0;

void LocoSlot::prepare(uint16_t locoId) {
    loco = locoId;
    speedCode=128; // default direction forward
    groupFlags=0;
    functions=0;
    momentum_base=0;
    momentumA=MOMENTUM_USE_DEFAULT;
    momentumD=MOMENTUM_USE_DEFAULT;
    targetSpeed=128;
    savedSpeed=128;
    blockOccupied=0;

    snifferSpeedCode=128; // default direction forward
    snifferFunctions=0;

    // Add to start of list
    next = firstSlot;
    firstSlot = this;
    chainModified=true;
};

/* static */ LocoSlot *  LocoSlot::getSlot(uint16_t locoId, bool autoCreate) {
  auto slot=firstSlot;
  for(;slot;slot=slot->next){
    if (slot->loco==locoId) return slot;
  }
  if (!autoCreate) return nullptr;
  if (recycler) {
    slot=recycler;
    recycler=recycler->next;
    // slot will be rechained into list in prepare()
  } else {
    if (slotCount>=MAX_LOCOS) {
      DIAG(F("<* MAX_LOCOS %d EXCEEDED *>"),MAX_LOCOS);
      return nullptr; // Too many locos
    }
    slot=new LocoSlot();
    if (!slot) return nullptr; // allocation failure
    slotCount++;
  }
  slot->prepare(locoId);
  return slot;
}

void LocoSlot::forget() {
  // remove from list
  if (firstSlot==this) {
    firstSlot=next;
  } else {
    LocoSlot * prev;
    for(prev=firstSlot; prev && prev->next!=this; prev=prev->next);
    if (prev) prev->next=next;
  }
  // add to recycler
  next=recycler;
  recycler=this;
  chainModified=true;
}

void LocoSlot::saveSpeed() {
  // targetSpeed should eventually to be the speed we want to reach and save
  savedSpeed=targetSpeed;
}

/* static */ void LocoSlot::forgetAll() {
  // remove entire list
  LocoSlot * killnext=nullptr; 
  for (auto slot=firstSlot;slot;slot=killnext) {
    killnext=slot->next;
    delete(slot);
  }
  firstSlot=nullptr;
  // remove recycler 
  killnext=nullptr; 
  for (auto slot=recycler;slot;slot=killnext) {
    killnext=slot->next;
    delete(slot);
  }
  recycler=nullptr;
  chainModified=true;
  slotCount=0;
}

/* static */ void LocoSlot::dumpTable(Print * output) {
  StringFormatter::send(output, F("\n<* LocoSlots %d/%d size=%db"),
    slotCount,MAX_LOCOS,sizeof(LocoSlot));
  for (auto slot=firstSlot; slot; slot=slot->next) {
    StringFormatter::send(output, 
      F("\n Loco=%-5d s=%-3d f=%-11l t=%-3d mA=%-3d mD=%-3d b=%-5d"),
      slot->loco,slot->speedCode,slot->functions,
      slot->targetSpeed,slot->momentumA,slot->momentumD,
      slot->blockOccupied);
#ifdef ARDUINO_ARCH_ESP32
    StringFormatter::send(output, F(" Ss=%-3d Sf=%-11l"),
      slot->snifferSpeedCode,slot->snifferFunctions);
#endif      
    }
  output->print(F("\n*>\n"));
  }

