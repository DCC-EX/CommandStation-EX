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
 * Ethernet (W5100,W5200,W5500) default mode of operation:
 *
 * All transmissions are sent with MAC address allOnes, i.e.
 * as a broadcast.  Regular sensor state updates are broadcast anyway, 
 * and other write/writeAnalogue command packets are relatively 
 * infrequent.
 *
 * Usage:
 *  Net_Ethernet *etherDriver = new Net_Ethernet(10);
 *  Network<Net_Ethernet>::create(4000, NUMREMOTEPINS(rpins), 1, rpins, etherDriver);
 * 
 * The W5x00 device has to be connected to the hardware MISO, MOSI, SCK and CS pins of the 
 * microcontroller.  The CS pin is specified in the command above (e.g. 10).
 * For details of the Network class configuration, see IO_Network.h.
 * 
 */

#ifndef NET_ETHERNET_H
#define NET_ETHERNET_H

#include "IO_Network.h"
#include "DIAG.h"
#include "Ethernet.h"
#include "EthernetUdp.h"

class Net_Ethernet {

private:
  // pins must be arduino GPIO pins, not extender pins or HAL pins.
  int _csPin;
  // Number of the current node (1-254)
  uint8_t _thisNode;
  // 4-byte IP address.  Last byte will contain the node number (1-254) or 255 for broadcast.
  byte _ipAddress[4] = {192,168,1,0};
  const uint16_t _port = 8900;
  uint8_t _packetBytesAvailable;
  EthernetUDP udp;

public:
  // Constructor performs static initialisation of the device object
  Net_Ethernet () {
    _csPin = 10;   // The Ethernet driver doesn't allow CS pin to be selected.
    _packetBytesAvailable = 0;
  }

  // Perform dynamic initialisation of the device
  bool begin(uint8_t thisNode) {
    _thisNode = thisNode;
    if (Ethernet.hardwareStatus() != EthernetNoHardware) {
      // // Set IP address.
      // _ipAddress[3] = thisNode;
      // Ethernet.setLocalIP(_ipAddress);
      // _ipAddress[3] = 254;
      // Ethernet.setGatewayIP(_ipAddress);
      // IPAddress mask = IPAddress(255,255,255,0);
      // Ethernet.setSubnetMask(mask);
      // Begin listening on receive port
      udp.begin(_port);
      return true;
    } else {
      DIAG(F("Ethernet (W5x00) no hardware found"));
      return false;
    }
  }

  // The following function should be called regularly to handle incoming packets.
  // Check if there is a received packet ready for reading.
  bool available() {
    uint16_t packetLen = udp.parsePacket();
      
    if (packetLen > 0) {
      // Packet received. 
      _packetBytesAvailable = packetLen;
        return true;
    }
    _packetBytesAvailable = 0;
    return false;
  }

  // Read packet from the ethernet buffer, and return the number of bytes
  // that were read.
  uint8_t read(uint8_t buffer[], uint8_t bufferSize) {
    uint8_t bytesReceived = _packetBytesAvailable;
    // Clear packet byte marker
    _packetBytesAvailable = 0;
    if (bytesReceived > 0) {
      //DIAG(F("ReadPacket(%d)"), bytesReceived);
      // Check if there's room for the data
      if (bufferSize >= bytesReceived) {
        return udp.read(buffer, bytesReceived);
      }
    }
    return 0;
  }

  // Wrapper functions for Ethernet UDP write function.
  // Since we don't know the IP address of the node, just broadcast
  // over the subnet.
  bool sendCommand(uint8_t node, const uint8_t buffer[], uint8_t len) {
    _ipAddress[3] = 255;
    udp.beginPacket(_ipAddress, _port);
    udp.write(buffer, len);
    udp.endPacket();
    return true;
  }

  void loop() { }

};

#endif //NET_ETHERNET_H