/* Copyright (c) 2023 Harald Barth
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
#include "LocoSlot.h"
#include "StringFormatter.h"
LocoSlot * LocoSlot::firstSlot = nullptr;
LocoSlot * LocoSlot::recycler = nullptr;
void LocoSlot::prepare(uint16_t locoId) {
    loco = locoId;
    speedCode=128; // default direction forward
    groupFlags=0;
    functions=0;
    momentum_base=0;
    momentumA=MOMENTUM_USE_DEFAULT;
    momentumD=MOMENTUM_USE_DEFAULT;
    targetSpeed=128;
    blockOccupied=0;

    snifferSpeedCode=128; // default direction forward
    snifferGroupFlags=0;
    snifferFunctions=0;
    snifferFunccounter=0;
    snifferSpeedcounter=0;

    // Add to start of list
    next = firstSlot;
    firstSlot = this;
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
    slot=new LocoSlot();
    if (!slot) return nullptr; // allocation failure
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
}



void LocoSlot::dumpTable(Print * output) {
  output->print(F("\n<*   Loco Speed Functions\n"));
  for (auto slot=firstSlot; slot; slot=slot->next) {
    StringFormatter::send(output, F("\nDCC %5d   %3d %11l" ),
      slot->loco,slot->speedCode,slot->functions);
    StringFormatter::send(output, F("\nSnf      %3d %11l" ),
      slot->snifferSpeedCode,slot->snifferFunctions);
    }
  output->print(F("\n*>\n"));
  }

