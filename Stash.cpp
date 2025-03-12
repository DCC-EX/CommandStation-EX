/* 
*  © 2024 Chris Harlow
 *  All rights reserved.
 *  
 *  This file is part of DCC-EX
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
#include "Stash.h"
#include "StringFormatter.h"

Stash::Stash(int16_t stash_id, int16_t loco_id) {
    this->stashId = stash_id;
    this->locoId = loco_id;
    this->next = first;
    first = this;
}

void Stash::clearAll() {
    for (auto s=first;s;s=s->next) {
        s->locoId = 0;
        s->stashId =0;
    }
}

void Stash::clearAny(int16_t loco_id) {
    auto lid=abs(loco_id);
    for (auto s=first;s;s=s->next)
        if (abs(s->locoId) == lid) {
            s->locoId = 0;
            s->stashId =0;
        }
}

void Stash::clear(int16_t stash_id) {
    set(stash_id,0);
}

int16_t Stash::get(int16_t stash_id) {
    for (auto s=first;s;s=s->next)
        if (s->stashId == stash_id) return s->locoId;
    return 0;
}

void Stash::set(int16_t stash_id, int16_t loco_id) {
    // replace any existing stash
    for (auto s=first;s;s=s->next)
        if (s->stashId == stash_id) {
            s->locoId=loco_id;
            if (loco_id==0) s->stashId=0; // recycle
            return;
        }
    if (loco_id==0) return; // no need to create a zero entry.

    // replace any empty stash 
    for (auto s=first;s;s=s->next)
        if (s->locoId == 0) {
            s->locoId=loco_id;
            s->stashId=stash_id;
            return;
        }
    // create a new stash
    new Stash(stash_id, loco_id);
}
 
void Stash::list(Print * stream, int16_t stash_id) {
    bool sent=false;
    for (auto s=first;s;s=s->next)
      if ((s->locoId) && (stash_id==0 || s->stashId==stash_id)) {
          StringFormatter::send(stream,F("<jM %d %d>\n"),
            s->stashId,s->locoId);
          sent=true;
      }
    if (!sent) StringFormatter::send(stream,F("<jM %d 0>\n"),
          stash_id);
}

Stash* Stash::first=nullptr;
