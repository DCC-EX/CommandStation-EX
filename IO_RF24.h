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
 * Each node on the network is configured with a node number in the range 0-254.
 * The remoting configuration defines, for each pin to be available remotely,
 * the node number and the VPIN number on that node. The configuration must
 * match in all nodes, since it is used by the sending node to identify the node
 * and VPIN to which a write command is to be sent, and the VPIN number for a
 * sensor/input, and on the receiving node to identify the node from which a
 * sensor/input value is being sourced.
 *
 * The node number is also used as the first byte of the nRF24's 5-byte address
 * field. Number 255 is treated as a multicast address.  All stations listen on
 * their own address and on the multicast address.
 *
 * All nodes send regular multicast packets containing the latest values of the
 * sensors as they know them.  On receipt of such a packet, each node extracts
 * the states of the sensors which are sourced by the originating node, and
 * updates the values in its own local data.  Thus, each node has a copy of the
 * states of all digital input pin values that are defined in the remoting
 * configuration.  Multicasts are sent frequently, so if one is missed
 * then, like a London bus, another will be along shortly.
 *
 * Commands (originating from write() or writeAnalogue() calls) are sent
 * immediately, directly from the originating node to the target node.  This
 * is done with acknowlegements enabled to maximise the probability of
 * successful delivery.
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
 *  First declare, for each remote pin in the common area, the mapping onto
 *  a node and VPIN number.  The array below assumes that the first remote
 *  VPIN is 4000.  The nRF24L01 device is connected to the standard SPI pins
 *  plus two others referred to as CE and CSN.  The Arduino pin numbers used
 *  for these are specified in the create() call.  The REMOTEPINS definition
 *  should be the same on all nodes in the network.  For outputs, it is the 
 *  definition in the sending node that dictates which node and VPIN the 
 *  action is performed on.  For inputs, the value is placed into the
 *  VPIN location defined in the sending node (that scans the input value), 
 *  but the value is only accepted in the receiving node if its definition
 *  shows that the signal originates in the sending node.
 * 
 * Example to go into mySetup() function in mySetup.cpp:
 *  REMOTEPINS rpins[] = {
 *    {0,30,RPIN_OUT},     //4000  Node 0 GPIO pin 30 (output)
 *    {1,30,RPIN_IN},      //4001  Node 1 GPIO pin 30 (input)
 *    {1,100,RPIN_INOUT},  //4002  Node 1 Servo (PCA9685) pin (output to servo, input busy flag)
 *    {1,164,RPIN_IN},     //4003  Node 1 GPIO extender (MCP23017) pin (input)
 *    {2,164,RPIN_IN}      //4004  Node 2 GPIO extender (MCP23017) pin (input)
 *  }
 *  // FirstVPIN, nPins, thisNode, pinDefs, CEPin, CSNPin
 *  RF24Net::create(4000, NUMREMOTEPINS(rpins), 0, rpins, 48, 49);
 * 
 * This example defines VPINs 4000-4004 which map onto pins on nodes 0, 1 and 2.
 * The nRF24 device has to be connected to the hardware MISO, MOSI, SCK and CS pins of the 
 * microcontroller; in addition, the CE and CSN pins on the nRF24 are connected to 
 * two pins (48 and 49 above).
 * 
 * If any of pins 4000-4004 are referenced by turnouts, outputs or sensors, or by EX-RAIL,
 * then the corresponding remote pin state will be retrieved or updated.  
 * For example, in EX-RAIL,
 *    SET(4000) on node 1 or 2 will set pin 30 on Node 0 to +5V (pin is put into output mode on first write).
 *    AT(4001) on node 0 or 2 will wait until the sensor attached to pin 30 on Node 1 activates.
 *    SERVO(4002,300,2) on node 0 or 2 will reposition the servo on Node 1 PCA9685 module to position 300, and
 *              AT(-4002) will wait until the servo has finished moving.
 * 
 * The following sensor definition on node 0 will map onto VPIN 4004, i.e. Node 2 VPIN 164, 
 * which is the first pin on the first MCP23017:
 *    <S 1 4004 0>
 * and when a sensor attached to the pin on node 2 is activated (pin pulled down to 0V) the following
 * message will be generated on node 0:
 *    <Q 1>
 * When the sensor deactivates, the following message will be generated on node 0:
 *    <q 1>
 */

#ifndef IO_RF24_H
#define IO_RF24_H

#include "IODevice.h"
#include "RF24.h"

// Macros and type for creating the remote pin definitions.
// The definitions are stored in PROGMEM to reduce RAM requirements.
// The flags byte contains, in the low 2 bits, RPIN_IN, RPIN_OUT or RPIN_INOUT.
typedef struct { uint8_t node; VPIN vpin; uint8_t flags; } RPIN;
#define REMOTEPINS  static const RPIN PROGMEM
#define NUMREMOTEPINS(x) (sizeof(x)/sizeof(RPIN))
enum {
  RPIN_IN=1,
  RPIN_OUT=2,
  RPIN_INOUT=RPIN_IN|RPIN_OUT,
};

class RF24Net : public IODevice {

private:
  // pins must be arduino GPIO pins, not extender pins or HAL pins.
  int _cePin = -1;
  int _csnPin = -1;
  const RPIN *_pinDefs;  // May need to become a far pointer!
  // Time of last loop execution
  unsigned long _lastExecutionTime;
  // Current digital values for remoted pins, stored as a bit field
  uint8_t *_pinValues;
  // Number of the current node (0-254)
  uint8_t _thisNode;
  // 5-byte nRF24L01 address.  First byte will contain the node number (0-254) or 255 for broadcast
  byte _address[5] = {0x00, 0xCC, 0xEE, 0xEE, 0xCC};
  // Maximum size of payload (must be 32 or less)
  static const uint8_t maxPayloadSize = 32;
  // Current node being sent sensor data and polled
  uint8_t _currentSendNode = 0;
  bool _sendInProgress = false;
  bool _changesPending;
  int _nextSendPin = 0;
  unsigned long _lastMulticastTime;
  int _firstPinToSend;  // must be a multiple of 8
  int _numPinsToSend;   // need not be a multiple of 8
  
  RF24 _radio;

  // List of network commands
  enum : uint8_t {
    NET_CMD_WRITE,
    NET_CMD_WRITEANALOGUE,
    NET_CMD_VALUEUPDATE,
  };

public:
  // Constructor performs static initialisation of the device object
  RF24Net (VPIN firstVpin, int nPins, uint8_t thisNode, const RPIN pinDefs[], int cePin, int csnPin) {
    _firstVpin = firstVpin;
    _nPins = nPins;
    _cePin = cePin;
    _csnPin = csnPin;
    _thisNode = thisNode;
    _pinDefs = pinDefs; 
    _address[0] = 0x00;
    _address[1] = 0xCC;
    _address[2] = 0xEE;
    _address[3] = 0xEE;
    _address[4] = 0xCC;
    _pinValues = (uint8_t *)calloc((nPins+7)/8, 1);  // Allocate space for input values.
    addDevice(this);

    // Identify which pins are allocated to this node.
    _firstPinToSend = -1;
    _numPinsToSend = 0;
    for (int pin=0; pin<_nPins; pin++) {
      uint8_t node = GETFLASH(&_pinDefs[pin].node);
      uint8_t flags = GETFLASH(&_pinDefs[pin].flags);
      // Check if the pin is an input on this node?
      if (node == _thisNode && (flags & RPIN_IN)) {
        if (_firstPinToSend==-1) _firstPinToSend = pin;
        _numPinsToSend = pin - _firstPinToSend + 1;
      }
      //DIAG(F("Node=%d FirstPin=%d, NumPins=%d"), node, _firstPinToSend, _numPinsToSend);
    }
    // Round down to multiple of 8 (byte boundary).
    _firstPinToSend /= 8;
    _firstPinToSend *= 8;
    _nextSendPin = _firstPinToSend;
    //DIAG(F("FirstPin=%d, NumPins=%d"), _firstPinToSend, _numPinsToSend);
  }

  // Static create function provides alternative way to create object
  static void create(VPIN firstVpin, int nPins, uint8_t thisNode, const RPIN pinDefs[], int cePin, int csnPin) {
    new RF24Net(firstVpin, nPins, thisNode, pinDefs, cePin, csnPin);
  }

protected:
  // _begin function called to perform dynamic initialisation of the device
  void _begin() override {
#if defined(DIAG_IO)
    _display();
#endif
    if (_radio.begin(_cePin, _csnPin)) {
      // Device initialisation OK, set up parameters
      _radio.setDataRate(RF24_2MBPS);
      _radio.setPALevel(RF24_PA_HIGH);
      _radio.setChannel(108);
      _radio.enableDynamicPayloads();  // variable length packets
      _radio.setAutoAck(true);
      _radio.enableDynamicAck(); // required for multicast to work
      _radio.setRetries(1, 5);  // Retry time=1*250+250us=500us, count=5.

      // Set to listen on the address 255
      _address[0] = 255;
      _radio.openReadingPipe(1, _address);
      // Also allow receives on own node address
      _address[0] = _thisNode;
      _radio.openReadingPipe(2, _address);
      _radio.startListening();

      _display();
      _deviceState = DEVSTATE_NORMAL;
    } else {
      // Error in initialising
      DIAG(F("nRF24L01 Failed to initialise"));
      _deviceState = DEVSTATE_FAILED;
    }
    _lastMulticastTime = _lastExecutionTime = micros();
  }

  // _read function - just return _value (updated in _loop when message received from remote node)
  int _read(VPIN vpin) override {
    int pin = vpin - _firstVpin;
    uint8_t mask = 1 << (pin & 7);
    int byteIndex = pin / 8;
    return (_pinValues[byteIndex] & mask) ? 1 : 0;
  }

  // _write (digital) - send command directly to the appropriate remote node.
  void _write(VPIN vpin, int value) override {
    // Send message
    int pin = vpin - _firstVpin;
    uint8_t node = GETFLASH(&_pinDefs[pin].node);
    uint8_t flags = GETFLASH(&_pinDefs[pin].flags);
    VPIN remoteVpin = GETFLASHW(&_pinDefs[pin].vpin);
    if (node != _thisNode && remoteVpin != VPIN_NONE && (flags & RPIN_OUT)) {
      #ifdef DIAG_IO
      DIAG(F("RF24: write(%d,%d)=>send(%d,\"write(%d,%d)\")"), vpin, value, node, remoteVpin, value);
      #endif

      outBuffer[0] = node;
      outBuffer[1] = NET_CMD_WRITE;
      outBuffer[2] = getMsb(remoteVpin);
      outBuffer[3] = getLsb(remoteVpin);
      outBuffer[4] = (uint8_t)value;
      // Set up to send to the specified node address
      sendCommand(node, outBuffer, 5);
    }
  }

  // _writeAnalogue - send command directly to the appropriate remote node.
  void _writeAnalogue(VPIN vpin, int value, uint8_t param1, uint16_t param2) override {
    // Send message
    int pin = vpin - _firstVpin;
    uint8_t node = GETFLASH(&_pinDefs[pin].node);
    uint8_t flags = GETFLASH(&_pinDefs[pin].flags);
    VPIN remoteVpin = GETFLASHW(&_pinDefs[pin].vpin);
    if (node != _thisNode && remoteVpin != VPIN_NONE && (flags & RPIN_OUT)) {
      #ifdef DIAG_IO
      DIAG(F("RF24: writeAnalogue(%d,%d,%d,%d)=>send(%d,\"writeAnalogue(%d,%d,...)\")"), 
        vpin, value, param1, param2, node, remoteVpin, value);
      #endif

      outBuffer[0] = node;
      outBuffer[1] = NET_CMD_WRITEANALOGUE;
      outBuffer[2] = getMsb(remoteVpin);
      outBuffer[3] = getLsb(remoteVpin);
      outBuffer[4] = getMsb(value);
      outBuffer[5] = getLsb(value);
      outBuffer[6] = param1;
      outBuffer[7] = getMsb(param2);
      outBuffer[8] = getLsb(param2);
      // Set up to send to the specified node address
      sendCommand(node, outBuffer, 9);
    }
  }

  // _loop function - check for, and process, received data from RF24, and send any
  // updates that are due.
  void _loop(unsigned long currentMicros) override {

    // Check for incoming data
    if (_radio.available(NULL))
      processReceivedData();

    // Force a data update broadcast every 500ms irrespective of whether there are
    // data changes or not.
    if (currentMicros - _lastMulticastTime > (500 * 1000UL)) 
      _changesPending = true;

    // Send out data update broadcasts once every 100ms if there are changes
    if (currentMicros - _lastExecutionTime > (100 * 1000UL)) {
      // Broadcast updates to all other nodes.  The preparation is done in a number of 
      // successive calls, and when sendSensorUpdates() returns true it has completed.
      if (sendSensorUpdates()) {
        _lastExecutionTime = currentMicros; // Send complete, wait another 100ms
      }
    }

    // Check if outstanding writes have completed.  If so, move to Standby-I mode 
    // and enable the receiver.
    if (_sendInProgress && _radio.isWriteFinished()) {
      _sendInProgress = false;
      _radio.txStandBy();
      _radio.startListening();
    }
  }

  void _display() override {
    DIAG(F("nRF24L01 Configured on Vpin:%d-%d CEPin:%d CSNPin:%d"),
      _firstVpin, _firstVpin+_nPins-1, _cePin, _csnPin);
  }

private:
  // Send sensor updates only if one or more locally sourced inputs that
  // are mapped to remote VPINs have changed state.
  //
  bool sendSensorUpdates() { 
    // This loop is split into multiple loop() entries, so as not to hog
    // the cpu for too long.  Otherwise it could take over 2700us with 108 remote
    // pins configured, for example.  So we do just 5 pins per call.  
    // We could make digital state change notification mandatory, which would 
    // allow us to remove the loop altogether!

    if (_numPinsToSend == 0) return true; // No pins to send from this node.

    // Update the _pinValues bitfield to reflect the current values of local pins.
    uint8_t count = 5;
    bool state;
    for (int pin=_nextSendPin; pin<_firstPinToSend+_numPinsToSend; pin++) {
      uint8_t flags = GETFLASH(&_pinDefs[pin].flags);
      if ((flags & RPIN_IN) && GETFLASH(&_pinDefs[pin].node) == _thisNode) {
        // Local input pin, read and update current state of input
        VPIN localVpin = GETFLASHW(&_pinDefs[pin].vpin);
        if (localVpin != VPIN_NONE) {
          state = IODevice::read(localVpin);
          uint16_t byteIndex = pin / 8;
          uint8_t bitMask = 1 << (pin & 7);
          uint8_t byteValue = _pinValues[byteIndex];
          bool oldState = byteValue & bitMask;
          if (state != oldState) {
            // Store state in remote values array
            if (state) 
              byteValue |= bitMask;
            else
              byteValue &= ~bitMask;
            _pinValues[byteIndex] = byteValue;
            _changesPending = true;
            //DIAG(F("RF24 VPIN:%d Val:%d"), _firstVpin+pin, state);
          }
          if (--count == 0) {
            // Done enough checks for this entry, resume on next one.
            _nextSendPin = pin+1;
            return false;
          }
        }
     }
    }

    if (_changesPending) { 
      // On master and on slave, send pin states to other nodes
      outBuffer[0] = _thisNode;  // Originating node
      outBuffer[1] = NET_CMD_VALUEUPDATE;
      // The packet size is 32 bytes, header is 4 bytes, so 28 bytes of data.
      // We can therefore send up to 224 binary states per packet.
      int byteCount = _numPinsToSend/8+1;
      VPIN remoteVpin = _firstVpin+_firstPinToSend;
      outBuffer[2] = getMsb(remoteVpin);
      outBuffer[3] = getLsb(remoteVpin);

      // Copy from pinValues array into buffer.  This is why _firstPinToSend must be a multiple of 8.
      memcpy(&outBuffer[4], &_pinValues[_firstPinToSend/8], byteCount);

      // Broadcast update
      sendCommand(255, outBuffer, byteCount + 4);
    
      //DIAG(F("Sent %d bytes: %x %x ..."), byteCount, outBuffer[4], outBuffer[5]);
      _lastMulticastTime = micros();
      _changesPending = false;
    }
    // Set next pin ready for next entry.
    _nextSendPin = _firstPinToSend;

    return true;  // Done all we need to for this cycle.
  }

  // Read next packet from the device's input buffers.  Decode the message, 
  // and take the appropriate action.
  // The packet may be a command to do an output write (digital or analogue), or
  // it may be an update for digital input signals.
  // For digital input signals, the values are propagated from the node that is
  // the pin source, via the master, to all the other nodes.  
  void processReceivedData() {
    // Read received data from input pipe
    byte size = _radio.getDynamicPayloadSize();
    // if (size > maxPayloadSize) return; // Packet too long to read!!
    // Read packet
    _radio.read(inBuffer, size);
    // Extract command type from packet.
    uint8_t command = inBuffer[1];
    // Process received data 
    switch (command) {
      case NET_CMD_WRITE: // Digital write command
        {
          uint8_t targetNode = inBuffer[0];
          if (targetNode == _thisNode) {
            VPIN vpin = makeWord(inBuffer[2], inBuffer[3]);
            uint8_t state = inBuffer[4];
            IODevice::write(vpin, state); 
          } else {
            sendCommand(targetNode, inBuffer, size);
          }
        }
        break;
      case NET_CMD_WRITEANALOGUE:  // Analogue write command
        {
          uint8_t targetNode = inBuffer[0];
          if (targetNode == _thisNode) {
            VPIN vpin = makeWord(inBuffer[2], inBuffer[3]);
            int value = makeWord(inBuffer[4], inBuffer[5]);
            uint8_t param1 = inBuffer[6];
            uint16_t param2 = makeWord(inBuffer[7], inBuffer[8]);
            IODevice::writeAnalogue(vpin, value, param1, param2);
            // Set the local value for the pin, used by isBusy(),
            // and subsequently updated by the remote node.
            _pinValues[vpin-_firstVpin] = true;
          } else {
            sendCommand(targetNode, inBuffer, size);
          }
        }
        break;
      case NET_CMD_VALUEUPDATE: // Updates of input states (sensors etc).
        {
          uint8_t sendingNode = inBuffer[0];
          //DIAG(F("Node %d Rx %x"), sendingNode, inBuffer[4]);
          VPIN vpin = makeWord(inBuffer[2], inBuffer[3]);

          // Read through the buffer one byte at a time.
          uint8_t *buffPtr = &inBuffer[4];
          uint8_t *bitFieldPtr = &_pinValues[(vpin-_firstVpin)/8];

          int currentPin = vpin - _firstVpin;
          for (int byteNo=0; byteNo<size-4 && currentPin<_nPins; byteNo++) {
            // Now work through the byte examining each bit.
            uint8_t byteValue = *buffPtr++;
            uint8_t bitMask = 1;
            for (int bitNo=0; bitNo<8 && currentPin<_nPins; bitNo++) {
              // Process incoming value if it's come from the pin source node
              uint8_t pinSource = GETFLASH(&_pinDefs[currentPin].node);
              if (sendingNode == pinSource) {
                if (byteValue & bitMask)
                  byteValue |= bitMask;
                else
                  byteValue &= ~bitMask;
                // if (pinNode == _thisNode) { // Local pin }
              }
              bitMask <<= 1;
              currentPin++;
            }
            // Store the modified byte back
            *bitFieldPtr++ = byteValue;
          }
        }
        break;
      default:
        break;
    }
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
  bool sendCommand(uint8_t node, uint8_t *buffer, uint8_t len) {
    _address[0] = node;
    _radio.openWritingPipe(_address);
    // We have to stop the receiver before we can transmit.
    _radio.stopListening();
    // Copy the message into the radio and start the transmitter.
    // Multicast (no ack expected) if destination node is 255.
    bool ok = _radio.writeFast(buffer, len, (node==255));
    // We will poll the radio later on to see when the transmit queue
    // has emptied.  When that happens, we will go back to receive mode.
    // This prevents txStandBy() from blocking while the transmission 
    // is in progress.
    _sendInProgress = true;;
    return ok;
  }

  // Helper functions for packing/unpacking buffers.
  inline uint16_t makeWord(uint8_t msb, uint8_t lsb) {
    return ((uint16_t)msb << 8) | lsb;
  }
  inline uint8_t getMsb(uint16_t w) {
    return w >> 8;
  }
  inline uint8_t getLsb(uint16_t w) {
    return w & 0xff;
  }
  // Data space for actual input and output buffers.  
  uint8_t inBuffer[maxPayloadSize];
  uint8_t outBuffer[maxPayloadSize];

};

#endif //IO_RF24Net4_H