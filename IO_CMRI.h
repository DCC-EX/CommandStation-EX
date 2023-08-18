/*
 *  © 2023, Neil McKechnie. All rights reserved.
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
 * CMRIbus
 * =======
 * To define a CMRI bus, example syntax:
 *    CMRIbus::create(bus, serial, baud[, cycletime[, pin]]);
 * 
 * bus = 0-255
 * serial = serial port to be used (e.g. Serial3)
 * baud = baud rate (9600, 19200, 28800, 57600 or 115200)
 * cycletime = minimum time between successive updates/reads of a node in millisecs (default 500ms)
 * pin = pin number connected to RS485 module's DE and !RE terminals for half-duplex operation (default VPIN_NONE)
 * 
 * Each bus must use a different serial port.
 * 
 * IMPORTANT: If you are using ArduinoCMRI library code by Michael Adams, at the time of writing this library
 * is not compliant with the LCS-9.10.1 specification for CMRInet protocol.  
 * Various work-arounds may be enabled within the driver by adding the following line to your config.h file,
 * to allow nodes running the ArduinoCMRI library to communicate:
 * 
 *  #define ARDUINOCMRI_COMPATIBLE
 * 
 * CMRINode
 * ========
 * To define a CMRI node and associate it with a CMRI bus,
 *    CMRInode::create(firstVPIN, numVPINs, bus, address, type [, inputs, outputs]);
 * 
 * firstVPIN = first vpin in block allocated to this device
 * numVPINs = number of vpins (e.g. 72 for an SMINI node)
 * bus = 0-255
 * address = 0-127
 * type = 'M' for SMINI (fixed 24 inputs and 48 outputs)
 *        'C' for CPNODE (16 to 144 inputs/outputs in groups of 8)
 *        (other types are not supported at this time).
 * inputs = number of inputs (CPNODE only)
 * outputs = number of outputs (CPNODE only)
 * 
 * Reference: "LCS-9.10.1
 *             Layout Control Specification: CMRInet Protocol
 *             Version 1.1 December 2014."
 */

#ifndef IO_CMRI_H
#define IO_CMRI_H

#include "IODevice.h"

/**********************************************************************
 * CMRInode class
 * 
 * This encapsulates the state associated with a single CMRI node, 
 * which includes the address type, number of inputs and outputs, and
 * the states of the inputs and outputs.
 **********************************************************************/
class CMRInode : public IODevice {
private:
  uint8_t _busNo;
  uint8_t _address;
  char _type;
  CMRInode *_next = NULL;
  uint8_t *_inputStates = NULL;
  uint8_t *_outputStates = NULL;
  uint16_t _numInputs = 0;
  uint16_t _numOutputs = 0;
  bool _initialised = false;

public:
  static void create(VPIN firstVpin, int nPins, uint8_t busNo, uint8_t address, char type, uint16_t inputs=0, uint16_t outputs=0) {
    if (checkNoOverlap(firstVpin, nPins)) new CMRInode(firstVpin, nPins, busNo, address, type, inputs, outputs);
  }
  CMRInode(VPIN firstVpin, int nPins, uint8_t busNo, uint8_t address, char type, uint16_t inputs=0, uint16_t outputs=0);

  uint8_t getAddress() {
    return _address;
  }
  CMRInode *getNext() {
    return _next;
  }
  void setNext(CMRInode *node) {
    _next = node;
  }
  bool isInitialised() {
    return _initialised;
  }
  void setInitialised() {
    _initialised = true;
  }

  void _begin() {
    _initialised = false;
  }

  int _read(VPIN vpin) {
    // Return current state from this device
    uint16_t pin = vpin - _firstVpin;
    if (pin < _numInputs) {
      uint8_t mask = 1 << (pin & 0x7);
      int index = pin / 8;
      return (_inputStates[index] & mask) != 0;
    } else
      return 0;
  }

  void _write(VPIN vpin, int value) {
    // Update current state for this device, in preparation the bus transmission
    uint16_t pin = vpin - _firstVpin - _numInputs;
    if (pin < _numOutputs) {
      uint8_t mask = 1 << (pin & 0x7);
      int index = pin / 8;
      if (value)
        _outputStates[index] |= mask;
      else
        _outputStates[index] &= ~mask;
    }
  }

  void saveIncomingData(uint8_t index, uint8_t data) {
    if (index < (_numInputs+7)/8)
      _inputStates[index] = data;
  }

  uint8_t getOutputStates(uint8_t index) {
    if (index < (_numOutputs+7)/8)
      return _outputStates[index];
    else
      return 0;
  }

  uint16_t getNumInputs() {
    return _numInputs;
  }

  uint16_t getNumOutputs() {
    return _numOutputs;
  }

  char getType() {
    return _type; 
  }

  uint8_t getBusNumber() {
    return _busNo;
  }

  void _display() override {
    DIAG(F("CMRInode type:'%c' configured on bus:%d address:%d VPINs:%u-%u (in) %u-%u (out)"),
      _type, _busNo, _address, _firstVpin, _firstVpin+_numInputs-1,
      _firstVpin+_numInputs, _firstVpin+_numInputs+_numOutputs-1);
  }

};

/**********************************************************************
 * CMRIbus class
 * 
 * This encapsulates the properties state of the bus and the
 * transmission and reception of data across that bus.  Each CMRIbus
 * object owns a set of CMRInode objects which represent the nodes
 * attached to that bus.
 **********************************************************************/
class CMRIbus : public IODevice {
private:
  // Here we define the device-specific variables.  
  uint8_t _busNo;
  HardwareSerial *_serial;
  unsigned long _baud;
  VPIN _transmitEnablePin = VPIN_NONE;
  CMRInode *_nodeListStart = NULL, *_nodeListEnd = NULL;
  CMRInode *_currentNode = NULL;

  // Transmitter state machine states
  enum {TD_IDLE, TD_PRETRANSMIT, TD_INIT, TD_TRANSMIT, TD_PROMPT, TD_RECEIVE};
  uint8_t _transmitState = TD_IDLE;
  // Receiver state machine states.
  enum {RD_SYN1, RD_SYN2, RD_STX, RD_ADDR, RD_TYPE, 
      RD_DATA, RD_ESCDATA, RD_SKIPDATA, RD_SKIPESCDATA, RD_ETX};
  uint8_t _receiveState = RD_SYN1;
  uint16_t _receiveDataIndex = 0;  // Index of next data byte to be received.
  CMRIbus *_nextBus = NULL;  // Pointer to next bus instance in list.
  unsigned long _cycleStartTime = 0;
  unsigned long _timeoutStart = 0;
  unsigned long _cycleTime; // target time between successive read/write cycles, microseconds
  unsigned long _timeoutPeriod; // timeout on read responses, in microseconds.
  unsigned long _currentMicros;  // last value of micros() from _loop function.
  unsigned long _postDelay; // delay time after transmission before switching off transmitter (in us)
  unsigned long _byteTransmitTime; // time in us for transmission of one byte

  static CMRIbus *_busList; // linked list of defined bus instances

  // Definition of special characters in CMRInet protocol
  enum : uint8_t {
    NUL = 0x00,
    STX = 0x02,
    ETX = 0x03,
    DLE = 0x10,
    SYN = 0xff,
  };

public:
  static void create(uint8_t busNo, HardwareSerial &serial, unsigned long baud, uint16_t cycleTimeMS=500, VPIN transmitEnablePin=VPIN_NONE) {
    new CMRIbus(busNo, serial, baud, cycleTimeMS, transmitEnablePin);
  } 

  // Device-specific initialisation
  void _begin() override {
    // CMRInet spec states one stop bit, JMRI and ArduinoCMRI use two stop bits
#if defined(ARDUINOCMRI_COMPATIBLE)
    _serial->begin(_baud, SERIAL_8N2);
#else
    _serial->begin(_baud, SERIAL_8N1);
#endif
  #if defined(DIAG_IO)
    _display();
  #endif
  }

  // Loop function (overriding IODevice::_loop(unsigned long))
  void _loop(unsigned long currentMicros) override;

  // Display information about the device
  void _display() override {
    DIAG(F("CMRIbus %d configured, speed=%d baud, cycle=%d ms"), _busNo, _baud, _cycleTime/1000);
  }

  // Locate CMRInode object with specified address.
  CMRInode *findNode(uint8_t address) {
    for (CMRInode *node = _nodeListStart; node != NULL; node = node->getNext()) {
      if (node->getAddress() == address) 
        return node;
    }
    return NULL;
  }

  // Add new CMRInode to the list of nodes for this bus.
  void addNode(CMRInode *newNode) {
    if (!_nodeListStart)
      _nodeListStart = newNode;
    if (!_nodeListEnd) 
      _nodeListEnd = newNode;
    else {
      _nodeListEnd->setNext(newNode);
      _nodeListEnd = newNode;
    }
  }

protected:
  CMRIbus(uint8_t busNo, HardwareSerial &serial, unsigned long baud, uint16_t cycleTimeMS, VPIN transmitEnablePin);
  uint16_t sendData(CMRInode *node);
  uint16_t requestData(CMRInode *node);
  uint16_t sendInitialisation(CMRInode *node);

  // Process any data bytes received from a CMRInode.
  void processIncoming();
  // Process any outgoing traffic that is due.
  void processOutgoing();
  // Enable transmitter
  void enableTransmitter();
  // Disable transmitter and enable receiver
  void disableTransmitter();


public:
  uint8_t getBusNumber() {
    return _busNo;
  }

  static CMRIbus *findBus(uint8_t busNo) {
    for (CMRIbus *bus=_busList; bus!=NULL; bus=bus->_nextBus) {
      if (bus->_busNo == busNo) return bus;
    }
    return NULL;
  }
};

#endif // IO_CMRI_H