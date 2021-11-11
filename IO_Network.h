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
 * Each node on the network is configured with a node number in the range 0-254.
 * The remoting configuration defines, for each pin to be available remotely,
 * the node number and the VPIN number on that node. The configuration must
 * match in all nodes, since it is used by the sending node to identify the node
 * and VPIN to which a write command is to be sent, and the VPIN number for a
 * sensor/input, and on the receiving node to identify the node from which a
 * sensor/input value is being sourced.
 *
 * The node number is also used in the network driver's address
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
 * Usage:
 *  First declare, for each remote pin in the common area, the mapping onto
 *  a node and VPIN number.  The array below assumes that the first remote
 *  VPIN is 4000. The REMOTEPINS definition
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
 *  Network::create(4000, NUMREMOTEPINS(rpins), 0, rpins, new RF24Driver(48, 49));
 * 
 * This example defines VPINs 4000-4004 which map onto pins on nodes 0, 1 and 2.
 * The network device in this case is an nRF24L01, which has to be connected to the hardware 
 * MISO, MOSI, SCK and CS pins of the microcontroller; in addition, the CE and 
 * CSN pins on the nRF24 are connected to two pins (48 and 49 above).
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

#ifndef IO_NETWORK_H
#define IO_NETWORK_H

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

// Define interface for network driver.  This should be implemented for each supported
// network type.
// class NetInterface {
// public:
//   bool begin();
//   bool sendCommand(uint8_t node, const uint8_t buffer[], uint8_t size);
//   bool available();
//   uint8_t read(uint8_t buffer[], uint8_t size);
//   void loop();
// };

// Class implementing the Application-layer network functionality.
// This is implemented as an IODevice instance so it can be easily 
// plugged in to the HAL framwork.
template <class NetInterface>
class Network : public IODevice {

private:
  const RPIN *_pinDefs;  // May need to become a far pointer!
  // Time of last loop execution
  unsigned long _lastExecutionTime;
  // Current digital values for remoted pins, stored as a bit field
  uint8_t *_pinValues;
  // Number of the current node (1-254)
  uint8_t _thisNode;
  // Maximum size of payload (must be 32 or less for RF24)
  static const uint8_t maxPayloadSize = 32;
  bool _updatePending;
  int _nextSendPin;
  unsigned long _lastMulticastTime;
  int _firstPinToSend;  // must be a multiple of 8
  int _numPinsToSend;   // need not be a multiple of 8
  NetInterface *_netDriver;
  
  // List of network commands
  enum : uint8_t {
    NET_CMD_WRITE = 0,
    NET_CMD_WRITEANALOGUE = 1,
    NET_CMD_VALUEUPDATE = 2,
  };

  // Field Positions in Network Header
  enum NetHeader {
    IONET_SENDNODE = 0,   // for VALUEUPDATE
    IONET_DESTNODE = 0,   // for WRITE/WRITEANALOGUE
    IONET_CMDTYPE = 1,
    IONET_VPIN = 2,
    IONET_VPIN_H = 2,
    IONET_VPIN_L = 3,
    IONET_DATA = 4,
  };

public:
  // Constructor performs static initialisation of the device object
  Network (VPIN firstVpin, int nPins, uint8_t thisNode, const RPIN pinDefs[], NetInterface *netDriver) {
    _firstVpin = firstVpin;
    _nPins = nPins;
    _thisNode = thisNode;
    _pinDefs = pinDefs; 
    _pinValues = (uint8_t *)calloc((nPins+7)/8, 1);  // Allocate space for input values.
    _netDriver = netDriver;
    addDevice(this);

    // Identify which pins are allocated to this node.
    _firstPinToSend = -1;
    int lastPinToSend = 0;
    for (int pin=0; pin<_nPins; pin++) {
      uint8_t node = GETFLASH(&_pinDefs[pin].node);
      uint8_t flags = GETFLASH(&_pinDefs[pin].flags);
      // Check if the pin is an input on this node?
      if (node == _thisNode && (flags & RPIN_IN)) {
        if (_firstPinToSend==-1) _firstPinToSend = pin;
        lastPinToSend = pin;
      }
      //DIAG(F("Node=%d FirstPin=%d, NumPins=%d"), node, _firstPinToSend, _numPinsToSend);
    }
    // Round down to multiple of 8 (byte boundary).
    _firstPinToSend /= 8;
    _firstPinToSend *= 8;
    _numPinsToSend = lastPinToSend - _firstPinToSend + 1;
    // Restrict to the max that fit in a packet
    _numPinsToSend = min(8*(MAX_MSG_SIZE-IONET_DATA),_numPinsToSend);
    //DIAG(F("FirstPin=%d, NumPins=%d"), _firstPinToSend, _numPinsToSend);

    // Prepare for first transmission
    _nextSendPin = _firstPinToSend;
  }

  // Static create function provides alternative way to create object
  static void create(VPIN firstVpin, int nPins, uint8_t thisNode, const RPIN pinDefs[], NetInterface *netDriver) {
    new Network(firstVpin, nPins, thisNode, pinDefs, netDriver);
  }

protected:
  // _begin function called to perform dynamic initialisation of the device
  void _begin() override {
    if (_netDriver->begin(_thisNode)) {
      _display();
      _deviceState = DEVSTATE_NORMAL;
      _lastMulticastTime = _lastExecutionTime = micros();
      _updatePending = true;
    } else {
      // Error in initialising
      DIAG(F("Network Failed to initialise"));
      _deviceState = DEVSTATE_FAILED;
    }
  }

  // _read function - just return pin value (updated in _loop when message received from remote node)
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
      DIAG(F("Network: write(%d,%d)=>send(%d,\"write(%d,%d)\")"), vpin, value, node, remoteVpin, value);
      #endif

      netBuffer[IONET_DESTNODE] = node;
      netBuffer[IONET_CMDTYPE] = NET_CMD_WRITE;
      netBuffer[IONET_VPIN_H] = getMsb(remoteVpin);
      netBuffer[IONET_VPIN_L] = getLsb(remoteVpin);
      netBuffer[IONET_DATA] = (uint8_t)value;
      // Set up to send to the specified node address
      _netDriver->sendCommand(node, netBuffer, IONET_DATA+1);
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
      DIAG(F("Network: writeAnalogue(%d,%d,%d,%d)=>send(%d,\"writeAnalogue(%d,%d,...)\")"), 
        vpin, value, param1, param2, node, remoteVpin, value);
      #endif

      netBuffer[IONET_DESTNODE] = node;
      netBuffer[IONET_CMDTYPE] = NET_CMD_WRITEANALOGUE;
      netBuffer[IONET_VPIN_H] = getMsb(remoteVpin);
      netBuffer[IONET_VPIN_L] = getLsb(remoteVpin);
      netBuffer[IONET_DATA+0] = getMsb(value);
      netBuffer[IONET_DATA+1] = getLsb(value);
      netBuffer[IONET_DATA+2] = param1;
      netBuffer[IONET_DATA+3] = getMsb(param2);
      netBuffer[IONET_DATA+4] = getLsb(param2);
      // Set up to send to the specified node address
      _netDriver->sendCommand(node, netBuffer, IONET_DATA+5);
    }
  }

  // _loop function - check for, and process, received data from RF24, and send any
  // updates that are due.
  void _loop(unsigned long currentMicros) override {

    // Perform cyclic netdriver functions, including switching back to receive mode 
    //  (for half-duplex network drivers) and receiving input packets.
    _netDriver->loop();

    // Check for incoming data
    if (_netDriver->available())
      processReceivedData();

    // Force a data update broadcast every 1000ms irrespective of whether there are
    // data changes or not.
    if (currentMicros - _lastMulticastTime > (1000 * 1000UL)) 
      _updatePending = true;

    // Send out data update broadcasts once every 20ms if there are changes
    if (currentMicros - _lastExecutionTime > (20 * 1000UL)) {
      // Broadcast updates to all other nodes.  The preparation is done in a number of 
      // successive calls, and when sendSensorUpdates() returns true it has completed.
      if (sendSensorUpdates()) {
        _lastExecutionTime = currentMicros; // Send complete, wait for next time
      }
    }
  }

  void _display() override {
    DIAG(F("Network Configured on Vpins:%d-%d Node:%d%S"),
      _firstVpin, _firstVpin+_nPins-1, _thisNode, (_deviceState==DEVSTATE_FAILED) ? F(" OFFLINE") : F(""));
  }

private:
  // Send sensor updates only if one or more locally sourced inputs that
  // are mapped to remote VPINs have changed state.
  //
  bool sendSensorUpdates() { 
    // This loop is split into multiple loop() entries, so as not to hog
    // the cpu for too long.  

    if (_numPinsToSend == 0) return true; // No pins to send from this node.

    // Update the _pinValues bitfield to reflect the current values of local pins.
    // Process maximum of 5 pins per entry.
    uint8_t count = 5;
    bool state;
    // First time through, _nextSendPin is equal to _firstPinToSend.
    for (int pin=_nextSendPin; pin<_firstPinToSend+_numPinsToSend; pin++) {
      uint8_t flags = GETFLASH(&_pinDefs[pin].flags);
      // Is the pin an input on this node?
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
            _updatePending = true;
          }
          if (--count == 0) {
            // Done enough checks for this entry, resume on next one.
            _nextSendPin = pin+1;
            return false;
          }
        }
     }
    }

    // When we get here, we've updated the _pinValues array.  See if an
    //  update is due.
    if (_updatePending) { 
      // On master and on slave, send pin states to other nodes
      netBuffer[IONET_SENDNODE] = _thisNode;  // Originating node
      netBuffer[IONET_CMDTYPE] = NET_CMD_VALUEUPDATE;
      // The packet size is 32 bytes, header is 4 bytes, so 28 bytes of data.
      // We can therefore send up to 224 binary states per packet.
      int byteCount = (_numPinsToSend+7)/8;
      VPIN remoteVpin = _firstVpin+_firstPinToSend;
      netBuffer[IONET_VPIN_H] = getMsb(remoteVpin);
      netBuffer[IONET_VPIN_L] = getLsb(remoteVpin);

      // Copy from pinValues array into buffer.  This is why _firstPinToSend must be a multiple of 8.
      memcpy(&netBuffer[IONET_DATA], &_pinValues[_firstPinToSend/8], byteCount);

      // Broadcast update
      _netDriver->sendCommand(255, netBuffer, IONET_DATA + byteCount);
    
      //DIAG(F("Sent %d bytes: %x %x ..."), byteCount, netBuffer[4], netBuffer[5]);
      _lastMulticastTime = micros();
      _updatePending = false;
    }
    // Set next pin ready for next entry.
    _nextSendPin = _firstPinToSend;

    return true;  // Done all we need to for this cycle.
  }

  // Read next packet from the device's input buffers.  Decode the message, 
  // and take the appropriate action.
  // The packet may be a command to do an output write (digital or analogue), or
  // it may be an update for digital input signals.
  // For digital input signals, the values are broadcast from the node that is
  // the pin source to all the other nodes.  
  void processReceivedData() {
    // Read packet
    uint8_t size = _netDriver->read(netBuffer, sizeof(netBuffer));
    if (size < IONET_DATA) return; // packet too short.
    // Extract command type from packet.
    uint8_t command = netBuffer[IONET_CMDTYPE];
    //DIAG(F("Received %d bytes, type=%d"), size, command);
    // Process received data 
    switch (command) {
      case NET_CMD_WRITE: // Digital write command
        {
          uint8_t targetNode = netBuffer[IONET_DESTNODE];
          if (targetNode == _thisNode && size == IONET_DATA+1) {
            VPIN vpin = makeWord(netBuffer[IONET_VPIN_H], netBuffer[IONET_VPIN_L]);
            uint8_t state = netBuffer[IONET_DATA];
            IODevice::write(vpin, state); 
          }
        }
        break;
      case NET_CMD_WRITEANALOGUE:  // Analogue write command
        {
          uint8_t targetNode = netBuffer[IONET_DESTNODE];
          if (targetNode == _thisNode && size == IONET_DATA+5) {
            VPIN vpin = makeWord(netBuffer[IONET_VPIN_H], netBuffer[IONET_VPIN_L]);
            int value = makeWord(netBuffer[IONET_DATA], netBuffer[IONET_DATA+1]);
            uint8_t param1 = netBuffer[IONET_DATA+2];
            uint16_t param2 = makeWord(netBuffer[IONET_DATA+3], netBuffer[IONET_DATA+4]);
            IODevice::writeAnalogue(vpin, value, param1, param2);
            // Set the local value for the pin, used by isBusy(),
            // and subsequently updated by the remote node.
            _pinValues[vpin-_firstVpin] = true;
          } 
        }
        break;
      case NET_CMD_VALUEUPDATE: // Updates of input states (sensors etc).
        {
          uint8_t sendingNode = netBuffer[IONET_SENDNODE];
          VPIN vpin = makeWord(netBuffer[IONET_VPIN_H], netBuffer[IONET_VPIN_L]);
          //DIAG(F("Node %d Size %d VPIN %d Rx States %x"), sendingNode, size, vpin, netBuffer[IONET_DATA]);

          // Read through the buffer one byte at a time.
          uint8_t *buffPtr = &netBuffer[IONET_DATA];
          uint8_t *bitFieldPtr = &_pinValues[(vpin-_firstVpin)/8];

          int currentPin = vpin - _firstVpin;
          for (int byteNo=0; byteNo<size-4 && currentPin<_nPins; byteNo++) {
            // Now work through the received byte examining each bit.
            uint8_t byteValue = *buffPtr++;
            uint8_t bitFieldValue = *bitFieldPtr;
            uint8_t bitMask = 1;
            for (int bitNo=0; bitNo<8 && currentPin<_nPins; bitNo++) {
              // Process incoming value if it's come from the pin source node
              uint8_t pinSource = GETFLASH(&_pinDefs[currentPin].node);
              if (sendingNode == pinSource) {
                if (byteValue & bitMask)
                  bitFieldValue |= bitMask;
                else
                  bitFieldValue &= ~bitMask;
              }
              bitMask <<= 1;
              currentPin++;
            }
            // Store the modified byte back
            *bitFieldPtr++ = bitFieldValue;
          }
        }
        break;
      default:
        break;
    }
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
  // Data space for actual input and output buffer.  
  uint8_t netBuffer[maxPayloadSize];

};

#endif //IO_NETWORK_H