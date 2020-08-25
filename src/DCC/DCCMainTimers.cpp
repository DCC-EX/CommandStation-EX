/*
 *  DCCMainTimers.cpp
 * 
 *  This file is part of CommandStation.
 *
 *  CommandStation is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  CommandStation is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with CommandStation.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "DCCMain.h"

bool DCCMain::interrupt1() {
  switch (interruptState) {
  case 0:   // start of bit transmission
    board->signal(HIGH);    
    interruptState = 1; 
    return true; // must call interrupt2 to set currentBit
  case 1:   // 29us after case 0
    if(generateRailcomCutout) {
      board->cutout(true);             // Start the cutout
      inRailcomCutout = true;         
      railcom->enableRecieve(true);  // Turn on the serial port so we can RX
    }
    interruptState = 2;
    break;
  case 2:   // 58us after case 0
    if(currentBit && !generateRailcomCutout) {
      board->signal(LOW);  
    }
    interruptState = 3;
    break; 
  case 3:   // 87us after case 0
    if(currentBit && !generateRailcomCutout) {
      // Return to case zero for start of another bit at 116us
      interruptState = 0;
    }
    else interruptState = 4;
    break;
  case 4:   // 116us after case 0
    if(!generateRailcomCutout) {
      board->signal(LOW);
    }
    interruptState = 5;
    break;
  // Case 5 and 6 fall to default case
  case 7:   // 203us after case 0
    if(!generateRailcomCutout) {
      // Return to case zero for start of another bit at 232us
      interruptState = 0; 
    }
    else interruptState = 8;
    break;
  // Cases 8-15 are for railcom timing
  case 16:
    board->cutout(false);      // Stop the cutout
    board->signal(LOW);     // Send out 29us of signal before case 0 flips it
    railcom->enableRecieve(false); // Turn off serial so we don't get garbage
    // Read the data out and tag it with identifying info
    railcom->readData(transmitID, transmitType, transmitAddress); 
    generateRailcomCutout = false;    // Don't generate another railcom cutout
    inRailcomCutout = false;        // We aren't in a railcom pulse
    interruptState = 0;         // Go back to start of new bit
    break;
  default:
    interruptState++;
    break;
  }

  return false;   // Don't call interrupt2
}

void DCCMain::interrupt2() {
  if (remainingPreambles > 0 ) {    // If there's more preambles to be sent
    currentBit=true;                // Send a one bit (preambles are one)

    // If we're on the first preamble bit and railcom is enabled, send out a 
    // railcom cutout. 
    if((board->getPreambles() - remainingPreambles == 0) && railcom->config.enable) {
      generateRailcomCutout = true; 
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
        // TODO(davidcutting42@gmail.com): check if this can be done with a 
        // peek() into packetQueue intead.
        for (int b=0;b<pendingPacket.length;b++) 
          transmitPacket[b] = pendingPacket.payload[b];
        transmitLength=pendingPacket.length;
        transmitRepeats=pendingPacket.repeats;
        transmitID=pendingPacket.transmitID;
        transmitAddress=pendingPacket.address;
        transmitType=pendingPacket.type;
      }
      else {
        // Load an idle packet
        memcpy(transmitPacket, kIdlePacket, sizeof(kIdlePacket));
        transmitLength=sizeof(kIdlePacket);
        transmitRepeats=0;
      }
    }
  }
}
