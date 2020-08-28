/*
 *  DCCTimers.cpp
 * 
 *  This file is part of CommandStation-EX.
 *
 *  CommandStation-EX is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  CommandStation-EX is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with CommandStation-EX.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "DCC.h"

bool DCC::interrupt1() {
  switch (interruptState) {
  case 0:   // start of bit transmission
    board->signal(HIGH);    
    interruptState = 1; 
    return true; // must call interrupt2 to set currentBit
  case 1:   // 29us after case 0
    if(rcomCutout)
      board->rcomCutout(true);             // Start the cutout       
    interruptState = 2;
    break;
  case 2:   // 58us after case 0
    if(currentBit && !rcomCutout)
      board->signal(LOW);  
    if(rcomCutout) 
      board->rcomEnable(true);  // Prepare the serial port to receive
    interruptState = 3;
    break; 
  case 3:   // 87us after case 0
    if(currentBit && !rcomCutout)
      // Return to case zero for start of another bit at 116us
      interruptState = 0;
    else interruptState = 4;
    break;
  case 4:   // 116us after case 0
    if(!rcomCutout)
      board->signal(LOW);
    interruptState = 5;
    break;
  // Case 5 and 6 fall to default case
  case 7:   // 203us after case 0
    if(!rcomCutout)
      // Return to case zero for start of another bit at 232us
      interruptState = 0; 
    else interruptState = 8;
    break;
  // Cases 8-15 are for railcom timing
  case 16:
    board->rcomEnable(false);   // Turn off serial so we don't get garbage
    board->rcomCutout(false);   // Stop the cutout
    board->signal(LOW);         // Send out 29us of signal before case 0 flips it
    
    // Read the data out and tag it with identifying info
    rcomAddr = transmitAddress;
    rcomID = transmitID;
    rcomTxType = transmitType;
    board->rcomRead(); 

    rcomCutout = false;         // Don't generate another railcom cutout
    interruptState = 0;         // Go back to start of new bit
    break;
  // Default case increments to the next case, 29us later
  default:
    interruptState++;
    break;
  }

  return false;   // Don't call interrupt2
}

void DCC::interrupt2() {
  if (remainingPreambles > 0 ) {    // If there's more preambles to be sent
    currentBit=true;                // Send a one bit (preambles are one)

    // If we're on the first preamble bit, we're not in programming mode, and 
    // RailCom is enabled, send out a RailCom cutout. 

    if((board->getPreambles() - remainingPreambles == 0) && !board->getProgMode()) {
      rcomCutout = true; 
      remainingPreambles -= 4;    // We're skipping 4 bits in the cutout
      return;
    }

    remainingPreambles--;   // decrement the number of preambles to send
    return;
  }
  
  // beware OF 9-BIT MASK  generating a zero to start each byte   
  currentBit=transmitPacket[bytes_sent] & kBitMask[bits_sent];
  bits_sent++;

  // If this is the last bit of a byte, prepare for the next byte 
  if (bits_sent==9) { // zero followed by 8 bits of a byte
    //end of Byte
    bits_sent=0;
    bytes_sent++;
    // if this is the last byte, prepare for next packet
    if (bytes_sent >= transmitLength) { 
      // end of transmission buffer... repeat or switch to next message
      bytes_sent = 0;
      remainingPreambles = board->getPreambles() + 1;  // Add one for the stop bit

      int pendingCount = packetQueue.count();

      // Note that the number of repeats does not include the final repeat, so
      // the number of times transmitted is nRepeats+1
      if (transmitRepeats > 0) {
        transmitRepeats--;
      }
      else if (pendingCount > 0) {
        // Copy pending packet to transmit packet
        Packet pendingPacket = packetQueue.pop();

        // Load info about the packet into the transmit variables.
        for (int b=0;b<pendingPacket.length;b++) 
          transmitPacket[b] = pendingPacket.payload[b];
        transmitLength=pendingPacket.length;
        transmitRepeats=pendingPacket.repeats;
        transmitID=pendingPacket.transmitID;
        transmitAddress=pendingPacket.address;
        transmitType=pendingPacket.type;
        transmitResetCount = 0;
      }
      else {
        // Load an idle or reset packet
        memcpy(transmitPacket, (board->getProgMode() ? kResetPacket : kIdlePacket) , sizeof(kIdlePacket));
        transmitLength=sizeof(kIdlePacket);
        transmitRepeats=0;
        if(transmitResetCount < 250) transmitResetCount++;
      }
    }
  }
}
