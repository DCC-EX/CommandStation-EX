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

#ifndef DCCQueue_h
#define DCCQueue_h
#include "Arduino.h"
#include "DCCWaveform.h"

enum PendingType:byte {NORMAL_PACKET,SPEED_PACKET,FUNCTION_PACKET,ACC_ON_PACKET,ACC_OFF_PACKET,DEAD_PACKET};
  struct PendingSlot {
      PendingSlot* next; 
      PendingType type;
      byte packetLength;
      byte packetRepeat;
      byte packet[MAX_PACKET_SIZE];
      
      union { // use depends on packet type
        uint16_t locoId;    // SPEED & FUNCTION packets  
        uint16_t delayOff;  // ACC_ON_PACKET delay to apply between on/off
        uint32_t startTime; // ACC_OFF_PACKET time (mS) to transmit 
      };
  };
  
class DCCQueue {
  public:
    
    
    // Non-speed packets are queued in the main queue
    static void scheduleDCCPacket(byte* packet, byte length, byte repeats, uint16_t loco=0);

    // Speed packets are queued in the high priority queue
    static void scheduleDCCSpeedPacket(byte* packet, byte length, byte repeats, uint16_t loco);
    
    // ESTOP packets jump the high priority queue and discard any outstanding throttle packets for this loco  
    static void scheduleEstopPacket(byte* packet, byte length, byte repeats,uint16_t loco);

    // Accessory gate-On Packet joins end of main queue as normal. 
    // When dequeued, packet is modified to gate-off and given the delayed start in the high priority queue. 
    // getNext will ignore this packet until the requested start time.
    static void scheduleAccOnOffPacket(byte* packet, byte length, byte repeats,int16_t delayms);

  
    // Schedules a main track packet from the queues.
    static bool scheduleNext(bool force); 

  private:
    bool scheduleNextInternal(); 
  // statics to manage high and low priority queues and recycleing of PENDINGs
    static PendingSlot* recycleList;
    static DCCQueue* highPriorityQueue;
    static DCCQueue* lowPriorityQueue;
    static uint16_t lastSentPacketLocoId; // used to prevent two packets to the same loco in a row

    DCCQueue();
    
    PendingSlot*  head;
    PendingSlot * tail;
    
    // obtain and initialise slot for a PendingSlot. 
    static PendingSlot*  getSlot(PendingType type, byte* packet, byte length, byte repeats, uint16_t loco);
    static void recycle(PendingSlot* p);
    void addQueue(PendingSlot * p);
    void jumpQueue(PendingSlot * p);
    void remove(PendingSlot * p);
};
#endif // DCCQueue_h
