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

/* What does this queue manager do:
  1. It provides a high priority queue and a low priority queue.
  2. It manages situations where multiple loco speed commands are in the queue.
  3. It allows an ESTOP to jump the queue and eliminate any outstanding speed commands that would later undo the stop.
  4. It allows for coil on/off accessory commands to be synchronized to a given time delay.
  5. It prevents transmission of sequential packets to the same loco id 
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
uint16_t DCCQueue::lastSentPacketLocoId=0; // used to prevent two packets to the same loco in a row


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
    
    void DCCQueue::remove(PendingSlot* premove) {
        PendingSlot* previous=nullptr;
        for (auto p=head;p;previous=p,p=p->next) {
            if (p==premove) {
                // remove this slot from the queue 
                if (previous) previous->next=p->next;
                else head=p->next;
                if (p==tail) tail=previous; // if last packet, update tail
                return;
            }
        }
        DIAG(F("DCCQueue::remove slot not found"));
      
    }

    // Packet joins end of low priority queue.
    void DCCQueue::scheduleDCCPacket(byte* packet, byte length, byte repeats, uint16_t loco) {
        lowPriorityQueue->addQueue(getSlot(NORMAL_PACKET,packet,length,repeats,loco));
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
       
        // kill any existing throttle packets for this loco (or all locos if broadcast)
        // this will also remove any estop packets for this loco (or all locos if broadcast) but they will be replaced
        PendingSlot * pNext=nullptr;
        for (auto p=highPriorityQueue->head;p;p=pNext) {
            pNext=p->next; // save next packet in case we recycle this one                
            if (p->type!=ACC_OFF_PACKET && (loco==0 || p->locoId==loco)) {
                // remove this slot from the queue or it will interfere with our ESTOP
                highPriorityQueue->remove(p);
                recycle(p);  // recycle this slot
            }
        }
        // add the estop packet to the start of the queue
        highPriorityQueue->jumpQueue(getSlot(NORMAL_PACKET,packet,length,repeats,0));
    }

    // Accessory coil-On Packet joins end of queue as normal.
    // When dequeued, packet is retained at start of queue 
    // but modified to coil-off and given the delayed start.
    // getNext will ignore this packet until the requested start time. 
    void DCCQueue::scheduleAccOnOffPacket(byte* packet, byte length, byte repeats,int16_t delayms) {
        auto p=getSlot(ACC_ON_PACKET,packet,length,repeats,0);
        p->delayOff=delayms;
        lowPriorityQueue->addQueue(p);
    };

    
    // Schedule the next dcc packet from the queues or an idle packet if none pending.
    const byte idlePacket[] = {0xFF, 0x00};

    bool DCCQueue::scheduleNext(bool force) {
        if (highPriorityQueue->scheduleNextInternal()) return true;
        if (lowPriorityQueue->scheduleNextInternal()) return true;
        if (force) {
            // This will arise when there is nothing available to be sent that will not compromise the rules
            // typically this will only happen when there is only one loco in the reminders as the closely queued 
            // speed and function reminders must be separated by at least one packet not sent to that loco.
            DCCWaveform::mainTrack.schedulePacket(idlePacket,sizeof(idlePacket),0);
            lastSentPacketLocoId=0;
            return true;
        }
        return false;
    }

    bool DCCQueue::scheduleNextInternal() {

        for (auto p=head;p;p=p->next) {
            // skip over pending ACC_OFF packets which are still delayed
            if (p->type == ACC_OFF_PACKET && millis()<p->startTime) continue;
            if (p->locoId) {
                // Prevent two consecutive packets to the same loco. 
                // this also means repeats cant be done by waveform 
                if (p->locoId==lastSentPacketLocoId) continue;  // try again later 
                DCCWaveform::mainTrack.schedulePacket(p->packet,p->packetLength,0);
                lastSentPacketLocoId=p->locoId;
                if (p->packetRepeat) {
                    p->packetRepeat--;
                    return true; // leave this packet in the queue
                }
            }
            else {
                // Non loco packets can repeat automatically
                DCCWaveform::mainTrack.schedulePacket(p->packet,p->packetLength,p->packetRepeat);
                lastSentPacketLocoId=0;
            }

            // remove this slot from the queue 
            remove(p);
            
            // special cases handling 
            if (p->type == ACC_ON_PACKET) {
                // convert to a delayed off packet and jump the high priority queue
                p->type= ACC_OFF_PACKET;
                p->packet[1]  &= ~0x08; // set C to 0 (gate off) 
                p->startTime=millis()+p->delayOff;
                highPriorityQueue->jumpQueue(p);
            }
            else recycle(p); 
            return true;
        }
        
        // No packets found
        return false;
    }
    
    // obtain and initialise slot for a PendingSlot.
    PendingSlot*  DCCQueue::getSlot(PendingType type, byte* packet, byte length, byte repeats,uint16_t loco) {
        PendingSlot * p; 
        if (recycleList) {
            p=recycleList;
            recycleList=p->next;
        }
        else { 
            static int16_t created=0;
            int16_t q1=0;
            int16_t q2=0;
            for (auto p=highPriorityQueue->head;p;p=p->next) q1++;
            for (auto p=lowPriorityQueue->head;p;p=p->next) q2++;
            bool leak=(q1+q2)!=created;
            DIAG(F("New DCC queue slot type=%d length=%d loco=%d q1=%d q2=%d created=%d"),
                   (int16_t)type,length,loco,q1,q2, created);
            if (leak) {
                for (auto p=highPriorityQueue->head;p;p=p->next) DIAG(F("q1 %d %d"),p->type,p->locoId);
                for (auto p=lowPriorityQueue->head;p;p=p->next) DIAG(F("q2 %d %d"),p->type,p->locoId);
            }       
            p=new PendingSlot; // need a queue entry
            created++; 
        }
        p->next=nullptr;
        p->type=type;
        p->packetLength=length;
        p->packetRepeat=repeats; 
        if (length>sizeof(p->packet)) {
            DIAG(F("DCC bad packet length=%d"),length);
            length=sizeof(p->packet); // limit to size of packet
        }
        p->startTime=0; // not used for loco packets
        memcpy((void*)p->packet,packet,length);
        p->locoId=loco;
        return p; 
    }

    
