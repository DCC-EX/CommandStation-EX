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

#include "IO_Modbus.h"
#include "defines.h"


/************************************************************
 * Modbus implementation
 ************************************************************/

// Constructor for Modbus
Modbus::Modbus(uint8_t busNo, HardwareSerial serial, unsigned long baud, uint16_t cycleTimeMS, int16_t transmitEnablePin) {
  _busNo = busNo;
  _baud = baud;
  _serial = &serial;
  _cycleTime = cycleTimeMS * 1000UL; // convert from milliseconds to microseconds.
  _transmitEnablePin = transmitEnablePin;
  if (_transmitEnablePin != VPIN_NONE) {
    pinMode(_transmitEnablePin, OUTPUT);
    ArduinoPins::fastWriteDigital(_transmitEnablePin, 0); // transmitter initially off
  }
  ModbusRTUMaster modbusmaster(*_serial, _transmitEnablePin);
  
  // Add device to HAL device chain
  IODevice::addDevice(this);

  // Add bus to CMRIbus chain.
  _nextBus = _busList;
  _busList = this;
  const char* errorStrings[] = {"success", "invalid id", "invalid buffer", "invalid quantity", "response timeout", "frame error", "crc error", "unknown comm error", "unexpected id", "exception response", "unexpected function code", "unexpected response length", "unexpected byte count", "unexpected address", "unexpected value", "unexpected quantity"};
}


// Main loop function for CMRIbus.
// Work through list of nodes.  For each node, in separate loop entries
// send initialisation message (once only);  then send
// output message;  then send prompt for input data, and 
// process any response data received.
// When the slot time has finished, move on to the next device.
void Modbus::_loop(unsigned long currentMicros) {
  
  _currentMicros = currentMicros;
  if (_currentNode == NULL) {
    // If we're between read/write cycles then don't do anything else.
    if (_currentMicros - _cycleStartTime < _cycleTime) return;
    // ... otherwise start processing the first node in the list
  DIAG(F("Modbusnode: 138  _nodeListEnd:%d "),  _nodeListEnd);    
  DIAG(F("Modbusnode: 139  _currentNode:%d "),  _currentNode);        
    _currentNode = _nodeListStart;
  DIAG(F("Modbusnode: 141  _currentNode:%d "),  _currentNode);    
    _cycleStartTime = _currentMicros;
  }
  if (_currentNode == NULL) return;

  uint8_t error;
  error = modbusmaster->writeMultipleCoils(_currentNode->getNodeID(), 0, _currentNode->coils, _currentNode->getNumCoils());
  if (error != 0) DIAG(F("%02d %04d %04d %s"), _currentNode->getNodeID(), 0, _currentNode->getNumCoils(), errorStrings[error]);

  error = modbusmaster->readDiscreteInputs(_currentNode->getNodeID(), 0, _currentNode->discreteInputs, _currentNode->getNumDisInputs());
  if (error != 0) DIAG(F("%02d %04d %04d %s"), _currentNode->getNodeID(), 0, _currentNode->getNumDisInputs(), errorStrings[error]);
}

// Link to chain of CMRI bus instances
Modbus *Modbus::_busList = NULL;


/************************************************************
 * Modbusnode implementation
 ************************************************************/

// Constructor for Modbusnode object
Modbusnode::Modbusnode(VPIN firstVpin, int nPins, uint8_t busNo, uint8_t nodeID,  uint8_t numCoils, uint8_t numDiscreteInputs) {
  _firstVpin = firstVpin;
  _nPins = nPins;
  _busNo = busNo;
  _nodeID = nodeID;
  coils[numCoils];
  discreteInputs[numDiscreteInputs];
  
  if ((unsigned int)_nPins < numDiscreteInputs + numCoils)
    DIAG(F("Modbusnode: bus:%d nodeID:%d WARNING number of Vpins does not cover all inputs and outputs"), _busNo, _nodeID);

  if (!discreteInputs || !coils) {
    DIAG(F("Modbusnode: ERROR insufficient memory"));
    return;
  }

  // Add this device to HAL device list
  IODevice::addDevice(this);

  // Add Modbusnode to Modbus object.
  Modbus *bus = Modbus::findBus(_busNo);
  if (bus != NULL) {
    bus->addNode(this);
    return;
  }
}
