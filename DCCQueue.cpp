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


#include "Arduino.h"
#include "defines.h"
#include "DCCQueue.h"
#include "DCCWaveform.h"
#include "DIAG.h"

// create statics 
DCCQueue* DCCQueue::lowPriorityQueue=new DCCQueue();
DCCQueue* DCCQueue::highPriorityQueue=new DCCQueue();
PendingSlot* DCCQueue::recycleList=nullptr;

    DCCQueue::DCCQueue() {
        head=nullptr;
        tail=nullptr;
    }
    
    void DCCQueue::addQueue(PendingSlot* p) {
        if (tail) tail->next=p;
        else head=p;
        tail=p;
        p->next=nullptr;          
    }

    void DCCQueue::jumpQueue(PendingSlot* p) {
        p->next=head;
        head=p;
        if (!tail) tail=p; 
    }

    
    void DCCQueue::recycle(PendingSlot* p) {
        p->next=recycleList;
        recycleList=p;
    }
    
    // Packet joins end of low priority queue.
    void DCCQueue::scheduleDCCPacket(byte* packet, byte length, byte repeats) {
        lowPriorityQueue->addQueue(getSlot(NORMAL_PACKET,packet,length,repeats,0));
    }
    
    // Packet replaces existing loco speed packet or joins end of high priority queue. 
    
    void DCCQueue::scheduleDCCSpeedPacket(byte* packet, byte length, byte repeats, uint16_t loco) {
        for (auto p=highPriorityQueue->head;p;p=p->next) {
            if (p->locoId==loco) {
                // replace existing packet
                memcpy(p->packet,packet,length);
                p->packetLength=length;
                p->packetRepeat=repeats;
                return;
            }
        }
        highPriorityQueue->addQueue(getSlot(NORMAL_PACKET,packet,length,repeats,loco));
    }
    
    
    // ESTOP -  
    // any outstanding throttle packet for this loco (all if loco=0) discarded
    // Packet joins start of queue,
   
    
    void DCCQueue::scheduleEstopPacket(byte* packet, byte length, byte repeats,uint16_t loco) {
        
        // DIAG(F("DCC ESTOP loco=%d"),loco);
       
        // kill any existing throttle packets for this loco
        PendingSlot * previous=nullptr;
        auto p=highPriorityQueue->head;
        while(p) {
            if (loco==0 || p->locoId==loco) {
                // drop this packet from the highPriority   queue 
                if (previous) previous->next=p->next;
                else highPriorityQueue->head=p->next;
                
                recycle(p);  // recycle this slot

                // address next packet
                p=previous?previous->next : highPriorityQueue->head;
            }
            else {
                previous=p;
                p=p->next;
            }
        }
        // add the estop packet to the start of the queue
        highPriorityQueue->jumpQueue(getSlot(NORMAL_PACKET,packet,length,repeats,0));
    }

    // Accessory gate-On Packet joins end of queue as normal.
    // When dequeued, packet is retained at start of queue 
    // but modified to gate-off and given the delayed start.
    // getNext will ignore this packet until the requested start time. 
    void DCCQueue::scheduleAccOnOffPacket(byte* packet, byte length, byte repeats,int16_t delayms) {
        auto p=getSlot(ACC_ON_PACKET,packet,length,repeats,0);
        p->delayOff=delayms;
        lowPriorityQueue->addQueue(p);
    };

    
    // Obtain packet (fills packet, length and repeats) 
    // returns 0 length if nothing in queue.

    bool DCCQueue::scheduleNext() {
        // check high priority queue first
        if (!DCCWaveform::mainTrack.isReminderWindowOpen()) return false;
        PendingSlot* previous=nullptr;
        for (auto p=highPriorityQueue->head;p;p=p->next) {
            // skip over pending ACC_OFF packets which are still delayed
            if (p->type == ACC_OFF_PACKET && millis()<p->startTime) continue;
            // use this slot
            DCCWaveform::mainTrack.schedulePacket(p->packet,p->packetLength,p->packetRepeat);
            // remove this slot from the queue 
            if (previous) previous->next=p->next;
            else highPriorityQueue->head=p->next;
            if (!highPriorityQueue->head) highPriorityQueue->tail=nullptr;
            
            // and recycle it.
            recycle(p);     
            return true;
        }
        
        // No high priopity packets found, check low priority queue
        auto p=lowPriorityQueue->head;
        if (!p) return false;  // nothing in queues
        
        // schedule first packet in queue 
        DCCWaveform::mainTrack.schedulePacket(p->packet,p->packetLength,p->packetRepeat);

        // remove from queue
        lowPriorityQueue->head=p->next;
        if (!lowPriorityQueue->head) lowPriorityQueue->tail=nullptr;
        
        if (p->type == ACC_ON_PACKET) {
                // convert to a delayed off packet and jump the high priority queue
                p->type= ACC_OFF_PACKET;
                p->packet[1]  &= ~0x08; // set C to 0 (gate off) 
                p->startTime=millis()+p->delayOff;
                highPriorityQueue->jumpQueue(p);
                }
        else recycle(p);  // recycle this slot
        return true;   
    }
    
    // obtain and initialise slot for a PendingSlot.
    PendingSlot*  DCCQueue::getSlot(PendingType type, byte* packet, byte length, byte repeats,uint16_t loco) {
        PendingSlot * p; 
        if (recycleList) {
            p=recycleList;
            recycleList=p->next;
        }
        else { 
            DIAG(F("New DCC queue slot"));
            p=new PendingSlot; // need a queue entry 
        }
        p->next=nullptr;
        p->type=type;
        p->packetLength=length;
        p->packetRepeat=repeats; 
        memcpy((void*)p->packet,packet,length);
        p->locoId=loco;
        return p; 
    }

    
