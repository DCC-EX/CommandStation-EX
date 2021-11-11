/*
 *  © 2021, Neil McKechnie. All rights reserved.
 *  
 *  This file is part of DCC++EX API
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

/*
 * ENC28J60 default mode of operation:
 *
 * The node number is used as the last byte of the MAC address
 * field, purely so that the MAC address is unique for each distinct 
 * node number.  The ENC28J60 driver doesn't implement ARP cache, 
 * since it would take up a minimum of 11 bytes per node and, with
 * up to 254 nodes, this would take a significant part of the RAM.
 * Instead, all transmissions are sent with MAC address allOnes, i.e.
 * as a broadcast.  Regular sensor state updates are broadcast anyway, 
 * and other write/writeAnalogue command packets are relatively 
 * infrequent.
 *
 * Usage:
 *  Net_ENC28J60 *encDriver = new Net_ENC28J60(10);
 *  Network<Net_ENC28J60>::create(4000, NUMREMOTEPINS(rpins), 1, rpins, encDriver);
 * 
 * The ENC28J60 device has to be connected to the hardware MISO, MOSI, SCK and CS pins of the 
 * microcontroller.  The CS pin is specified in the command above (e.g. 10).
 * For details of the Network class configuration, see IO_Network.h.
 * 
 */

#ifndef NET_ENC28J60_H
#define NET_ENC28J60_H

#include "IO_Network.h"
#include "DIAG.h"
#include "EtherCard.h"

// The ethernet buffer is global to different protocol layers, to avoid almost
// all copying of data.
byte ENC28J60::buffer[74]; // Need space for 32 byte payload and 42 byte header.

class Net_ENC28J60 : public EtherCard {

private:
  // pins must be arduino GPIO pins, not extender pins or HAL pins.
  int _csPin;
  // Number of the current node (1-254)
  uint8_t _thisNode;
  // 6-byte MAC address.  Last byte will contain the node number (1-254).
  byte _address[6] = {0xEE,0xCC,0xEE,0xEE,0xCC,0x00};
  // 4-byte IP address.  Last byte will contain the node number (1-254) or 255 for broadcast.
  byte _ipAddress[4] = {192,168,1,0};
  byte _gwAddress[4] = {192,168,1,254};
  byte _netMask[4] = {255,255,255,0};
  const uint16_t _port = 8900;
  uint8_t _packetBytesAvailable;

public:
  // Constructor performs static initialisation of the device object
  Net_ENC28J60 (int csPin) {
    _csPin = csPin;
    _packetBytesAvailable = 0;
  }

  // Perform dynamic initialisation of the device
  bool begin(uint8_t thisNode) {
    _thisNode = thisNode;
    _address[5] = _thisNode;
    _ipAddress[3] = _thisNode;
    if (ether.begin(sizeof(ENC28J60::buffer), _address, _csPin)) {
      ether.staticSetup(_ipAddress, _gwAddress, 0, _netMask);
      return true;
    } else {
      // Error in initialising
      DIAG(F("ENC28J60 Failed to initialise"));
      return false;
    }
  }

  // The following function should be called regularly to handle incoming packets.
  // Check if there is a received packet ready for reading.
  bool available() {
    uint16_t packetLen = ether.packetReceive();
      
    if (packetLen > 0) {
      // Packet received.  First handle ICMP, ARP etc.
      if (ether.packetLoop(packetLen)) {
        // UDP packet to be handled.  Check if it's our port number
        byte *gbp = ENC28J60::buffer;
        uint16_t destPort = (gbp[UDP_DST_PORT_H_P] << 8) + gbp[UDP_DST_PORT_L_P];
        if (destPort == _port) {
          // Yes, so mark that data is to be processed.
          _packetBytesAvailable = packetLen;
          return true;
        }
      }
    }
    _packetBytesAvailable = 0;
    return false;
  }

  // Read packet from the ethernet buffer, and return the number of bytes
  // that were read.
  uint8_t read(uint8_t buffer[], uint8_t bufferSize) {
    if (_packetBytesAvailable > 0) {
      //DIAG(F("ReadPacket(%d)"), _packetBytesAvailable);
      // Adjust length and pointer for UDP header
      byte *gbp = ENC28J60::buffer;
      int udpDataSize = (gbp[UDP_LEN_H_P] << 8) + gbp[UDP_LEN_L_P] - UDP_HEADER_LEN;
      byte *udpFrame = &ENC28J60::buffer[UDP_DATA_P];

      // Clear packet byte marker
      _packetBytesAvailable = 0;

      // Check if there's room for the data
      if (bufferSize >= udpDataSize) {
        memcpy(buffer, udpFrame, udpDataSize);
        return udpDataSize;
      }
    }
    return 0;
  }

  // Wrapper functions for ENC28J60 sendUdp function.
  // The node parameter is either 1-254 (for specific nodes) or 255 (for broadcast).
  // This aligns with the subnet broadcast IP address of "x.x.x.255".
  bool sendCommand(uint8_t node, const uint8_t buffer[], uint8_t len) {
    _ipAddress[3] = node;
    ether.sendUdp((const char *)buffer, len, _port, _ipAddress, _port);
    return true;
  }

  void loop() { }

};

#endif //NET_ENC28J60_H