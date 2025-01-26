/*
 *  © 2025 Chris Harlow
 *  All rights reserved.
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

#include "DoLater.h"
#include "DCC.h"
#include "DCCWaveform.h"

// Initialize the static members
DoLater::PENDING * DoLater::pendingList=nullptr;
DoLater::PENDING * DoLater::recycleList=nullptr;

DoLater::PENDING* DoLater::getSlot(PendingType type, uint16_t delay) {
    PENDING* p;
    // pick a pending from the recycler, or create a new one.
    if (recycleList) {
        p=recycleList;
        recycleList=p->next; 
    }
    else p=new PENDING;

    // chain into pending list 
    p->next=pendingList;
    pendingList=p; 

    p->type=type;
    p->startTime = millis();
    p->delay = delay; 
    return p;   
}

void DoLater::sendDCCPacket(uint16_t delay, byte* packet,byte length,byte repeats) {
    auto p=getSlot(SEND_DCC,delay);
    memcpy(p->packet,packet,length);
    p->packetLength=length;
    p->packetRepeat=repeats;
}

void DoLater::resetVpin(uint16_t delay, VPIN pin) {
    auto p=getSlot(RESET_VPIN,delay);
    p->vpin=pin;
}

void DoLater::loop() {
    auto currentTime=millis();
    PENDING* previous=nullptr;
    for (auto p=pendingList; p; p=p->next) {
        if (currentTime - p->startTime >= p->delay) {
            // found a timed out candidate
            switch(p->type) {
                case SEND_DCC:
                    // dont allow this to block, continue will just leave it on the list
                    if (!DCCWaveform::mainTrack.isReminderWindowOpen()) continue;
                    DCCWaveform::mainTrack.schedulePacket(p->packet,p->packetLength,p->packetRepeat);  
                    break;
                  
                case RESET_VPIN:
                  IODevice::write(p->vpin,LOW);
                  break;
            }
    // unchain this and add to recycler
            if (previous) previous->next=p->next;
            else pendingList=p->next;
            p->next=recycleList;
            recycleList=p;
            return; // only ever one on each loop()
        }
    }
    // Nothing found 
}
