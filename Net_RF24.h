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
 * nRF24 default mode of operation:
 *   Channel: 108
 *   Bit rate: 2MHz
 *   CRC: 16-bit
 *   Power Level: High
 *
 * The node number is used as the first byte of the nRF24's 5-byte address
 * field. Number 255 is treated as a multicast address.  All stations listen on
 * their own address and on the multicast address.
 *
 * The nRF24 device receives and acknowledges data packets autonomously.
 * Therefore, this driver just needs to detect when a packet is received and
 * read and process its contents.  The time to read the packet is under 200us
 * typically.
 *
 * The nRF24 is also capable of autonomously sending packets, processing
 * acknowledgements, and generating retries.  The driver writes the packet to
 * the device and then waits for notification of completion (success, or retries
 * exceeded) through the device's registers.  Similarly, the time to write a
 * packet is under 200us and, if we don't wait for the completion, we can allow
 * the processor to do other things while the transmission is in progress.
 * A write with ack can complete in under 600us, plus the time of turning the
 * receiver off and on.
 *
 * Usage:
 *  Net_RF24 *rf24Driver = new Net_RF24(48, 49);
 *  Network<Net_RF24>::create(4000, NUMREMOTEPINS(rpins), 1, rpins, rf24Driver);
 * 
 * The nRF24 device has to be connected to the hardware MISO, MOSI, SCK and CS pins of the 
 * microcontroller; in addition, the CE and CSN pins on the nRF24 are connected to 
 * two pins (e.g. 48 and 49 above).
 * 
 */

#ifndef NET_RF24_H
#define NET_RF24_H

#include "IO_Network.h"
#include "DIAG.h"

class Net_RF24 : public RF24 {

private:
  // pins must be arduino GPIO pins, not extender pins or HAL pins.
  int _cePin;
  int _csnPin;
  // Number of the current node (1-254)
  uint8_t _thisNode;
  // 5-byte nRF24L01 address.  First byte will contain the node number (0-254) or 255 for broadcast
  byte _address[5];
  bool _sendInProgress;


public:
  // Constructor performs static initialisation of the device object
  Net_RF24 (int cePin, int csnPin) {
    _cePin = cePin;
    _csnPin = csnPin;
    // Initialise with an arbitrary address.  The first byte will contain
    // the node number.
    _address[0] = 0x00;
    _address[1] = 0xCC;
    _address[2] = 0xEE;
    _address[3] = 0xEE;
    _address[4] = 0xCC;
  }

  // Perform dynamic initialisation of the device
  bool begin(uint8_t thisNode) {
    if (RF24::begin(_cePin, _csnPin)) {
      // Device initialisation OK, set up parameters
      RF24::setDataRate(RF24_2MBPS);
      RF24::setPALevel(RF24_PA_HIGH);
      RF24::setChannel(108);
      RF24::enableDynamicPayloads();  // variable length packets
      RF24::setAutoAck(true);    // required for acks to work
      RF24::enableDynamicAck();  // required for multicast to work
      RF24::setRetries(1, 5);    // Retry time=1*250+250us=500us, count=5.

      _thisNode = thisNode;
      // Set to listen on the address 255
      _address[0] = 255;
      RF24::openReadingPipe(1, _address);
      // Also allow receives on own node address
      _address[0] = _thisNode;
      RF24::openReadingPipe(2, _address);
      RF24::startListening();
      _sendInProgress = false;
      return true;
    } else {
      // Error in initialising
      DIAG(F("nRF24L01 Failed to initialise"));
      return false;
    }
  }

  // Check if there is a received packet ready for reading.
  bool available()  {
    return RF24::available();
  }

  // Read next packet from the device, and return the number of bytes
  // that were read.
  uint8_t read(uint8_t buffer[], uint8_t size)  {
    uint8_t len = RF24::getDynamicPayloadSize();
    RF24::read(buffer, size);
    return len;
  }

  // Wrapper functions for RF24 send functions.  If node=255, then
  //  the packet is to be sent as a multicast without acknowledgements.
  //  The multicast message takes ~400us. A further 260us is required to turn
  //  the receiver off and on for the transmission, totalling 660us.
  //  If the node is not 255, then the packet will be sent directly to the
  //  addressed node, with acknowledgement requested.  If no acknowledgement is
  //  received, then the device will retry up to the defined maximum number of
  //  retries.  This will take longer than a multicast.  For example, with
  //  setRetries(1,3) the timeout is 500us and a maximum of 3 retries are
  //  carried out, so the operation will take as much as 2.26 milliseconds if
  //  the node in question is not responding, and as little as 890us if the 
  //  ack is received immediately (including turning receiver on/off).
  //
  bool sendCommand(uint8_t node, const uint8_t buffer[], uint8_t len)  {
    _address[0] = node;
    openWritingPipe(_address);
    // We have to stop the receiver before we can transmit.
    RF24::stopListening();
    // Copy the message into the radio and start the transmitter.
    // Multicast (no ack expected) if destination node is 255.
    bool ok = RF24::writeFast(buffer, len, (node==255));
    // We will poll the radio later on to see when the transmit queue
    // has emptied.  When that happens, we will go back to receive mode.
    // This prevents txStandBy() from blocking while the transmission 
    // is in progress.
    _sendInProgress = true;;
    return ok;
  }

  // The following function should be called regularly to ensure that the
  // device goes back into listening mode when a transmission is not
  // in progress. (The nRF24 is a half-duplex device and cannot be in 
  // transmit mode and receive mode at the same time.)
  void loop() {
    bool completed = RF24::isWriteFinished();
    if (_sendInProgress && completed) {
      _sendInProgress = false;
      RF24::txStandBy();
      RF24::startListening();
    }
  }

};

#endif //NET_RF24_H