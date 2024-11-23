/*
 *  © 2024, Travis Farmer. All rights reserved.
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
 * Modbus
 * =======
 * To define a Modbus, example syntax:
 *    Modbus::create(bus, serial, baud[, cycletime[, pin]]);
 * 
 * bus = 0-255
 * serial = serial port to be used (e.g. Serial3)
 * baud = baud rate (9600, 19200, 28800, 57600 or 115200)
 * cycletime = minimum time between successive updates/reads of a node in millisecs (default 500ms)
 * pin = pin number connected to RS485 module's DE and !RE terminals for half-duplex operation (default VPIN_NONE)
 * 
 * Each bus must use a different serial port.
 * 
 * ModbusNode
 * ========
 * To define a CMRI node and associate it with a CMRI bus,
 *    CMRInode::create(firstVPIN, numVPINs, bus, nodeID, type [, inputs, outputs]);
 * 
 * firstVPIN = first vpin in block allocated to this device
 * numVPINs = number of vpins (e.g. 72 for an SMINI node)
 * bus = 0-255
 * nodeID = 0-127
 * numDiscreteInputs = number of discrete inputs
 * numCoils = number of coils
 * 
 * Reference: "LCS-9.10.1
 *             Layout Control Specification: CMRInet Protocol
 *             Version 1.1 December 2014."
 */

#ifndef IO_MODBUS_H
#define IO_MODBUS_H

#include "IODevice.h"
#include "ModbusRTUMaster.h"
/**********************************************************************
 * Modbusnode class
 * 
 * This encapsulates the state associated with a single Modbus node, 
 * which includes the nodeID, number of discrete inputs and coils, and
 * the states of the discrete inputs and coils.
 **********************************************************************/
class Modbusnode : public IODevice {
private:
  uint8_t _busNo;
  uint8_t _nodeID;
  char _type;
  Modbusnode *_next = NULL;
  bool _initialised = false;
  uint8_t numCoils;
  uint8_t numDiscreteInputs;

public:
  static void create(VPIN firstVpin, int nPins, uint8_t busNo, uint8_t nodeID, uint8_t numCoils=0, uint8_t numDiscreteInputs=0) {
    if (checkNoOverlap(firstVpin, nPins)) new Modbusnode(firstVpin, nPins, busNo, nodeID, numCoils, numDiscreteInputs);
  }
  Modbusnode(VPIN firstVpin, int nPins, uint8_t busNo, uint8_t nodeID, uint8_t numCoils=0, uint8_t numDiscreteInputs=0);
  bool *coils;
  bool *discreteInputs;

  uint8_t getNodeID() {
    return _nodeID;
  }
  uint8_t getNumCoils() {
    return numCoils;
  }
  uint8_t getNumDisInputs() {
    return numDiscreteInputs;
  }
  
  Modbusnode *getNext() {
    return _next;
  }
  void setNext(Modbusnode *node) {
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
    if (pin < numDiscreteInputs) {
      return discreteInputs[pin];
    } else
      return 0;
  }

  void _write(VPIN vpin, int value) {
    // Update current state for this device, in preparation the bus transmission
    uint16_t pin = vpin - _firstVpin - numCoils;
    if (pin < numCoils) {
      if (value)
        coils[pin] = value;
      else
        coils[pin];
    }
  }

  void saveIncomingData(uint8_t index, uint8_t data) {
    if (index < numDiscreteInputs)
      discreteInputs[index] = data;
  }

  uint8_t getOutputStates(uint8_t index) {
    if (index < numCoils)
      return coils[index];
    else
      return 0;
  }

  uint16_t getNumInputs() {
    return numDiscreteInputs;
  }

  uint16_t getNumOutputs() {
    return numCoils;
  }

  char getType() {
    return _type; 
  }

  uint8_t getBusNumber() {
    return _busNo;
  }

  void _display() override {
    DIAG(F("Modbusnode type:'%c' configured on bus:%d nodeID:%d VPINs:%u-%u (in) %u-%u (out)"),
      _type, _busNo, _nodeID, _firstVpin, _firstVpin+numDiscreteInputs-1,
      _firstVpin+numDiscreteInputs, _firstVpin+numDiscreteInputs+numCoils-1);
  }

};

/**********************************************************************
 * Modbus class
 * 
 * This encapsulates the properties state of the bus and the
 * transmission and reception of data across that bus.  Each Modbus
 * object owns a set of Modbusnode objects which represent the nodes
 * attached to that bus.
 **********************************************************************/
class Modbus : public IODevice {
private:
  // Here we define the device-specific variables.  
  uint8_t _busNo;

  unsigned long _baud;
  int16_t _transmitEnablePin = VPIN_NONE;
  Modbusnode *_nodeListStart = NULL, *_nodeListEnd = NULL;
  Modbusnode *_currentNode = NULL;

  uint16_t _receiveDataIndex = 0;  // Index of next data byte to be received.
  Modbus *_nextBus = NULL;  // Pointer to next bus instance in list.
  unsigned long _cycleStartTime = 0;
  unsigned long _timeoutStart = 0;
  unsigned long _cycleTime; // target time between successive read/write cycles, microseconds
  unsigned long _timeoutPeriod; // timeout on read responses, in microseconds.
  unsigned long _currentMicros;  // last value of micros() from _loop function.
  unsigned long _postDelay; // delay time after transmission before switching off transmitter (in us)
  unsigned long _byteTransmitTime; // time in us for transmission of one byte

  static Modbus *_busList; // linked list of defined bus instances

public:
  static void create(uint8_t busNo, HardwareSerial& serial, unsigned long baud, uint16_t cycleTimeMS=500, int16_t transmitEnablePin=VPIN_NONE) {
    new Modbus(busNo, serial, baud, cycleTimeMS, transmitEnablePin);
  }
  HardwareSerial *_serial;
  ModbusRTUMaster *modbusmaster;
  const char* errorStrings[];
  // Device-specific initialisation
  void _begin() override {
    _serial->begin(_baud, SERIAL_8N1);
    modbusmaster->begin(_baud);
  #if defined(DIAG_IO)
    _display();
  #endif
  }

  // Loop function (overriding IODevice::_loop(unsigned long))
  void _loop(unsigned long currentMicros) override;

  // Display information about the device
  void _display() override {
    DIAG(F("Modbus %d configured, speed=%d baud, cycle=%d ms"), _busNo, _baud, _cycleTime/1000);
  }

  // Locate Modbusnode object with specified nodeID.
  Modbusnode *findNode(uint8_t nodeID) {
    for (Modbusnode *node = _nodeListStart; node != NULL; node = node->getNext()) {
      if (node->getNodeID() == nodeID) 
        return node;
    }
    return NULL;
  }

  // Add new Modbusnode to the list of nodes for this bus.
  void addNode(Modbusnode *newNode) {
    if (!_nodeListStart)
      _nodeListStart = newNode;
    if (!_nodeListEnd) 
      _nodeListEnd = newNode;
    else
      _nodeListEnd->setNext(newNode);
  DIAG(F("bus: 260h nodeID: _nodeListStart:%d _nodeListEnd:%d"), _nodeListStart, _nodeListEnd);
  }

protected:
  Modbus(uint8_t busNo, HardwareSerial serial, unsigned long baud, uint16_t cycleTimeMS, int16_t transmitEnablePin);

public:
  
  uint8_t getBusNumber() {
    return _busNo;
  }

  static Modbus *findBus(uint8_t busNo) {
    for (Modbus *bus=_busList; bus!=NULL; bus=bus->_nextBus) {
      if (bus->_busNo == busNo) return bus;
    }
    return NULL;
  }
};

#endif // IO_MODBUS_H
