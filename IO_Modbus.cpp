/*
 *  © 2024, Travis Farmer. All rights reserved.
 *  © 2024, Chris Bulliner. All rights reserved. https://github.com/CMB27
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


void Modbus::setTransactionId(uint16_t transactionId) {
  _setRegister(tcp, 0, transactionId);
}

void Modbus::setProtocolId(uint16_t protocolId) {
  _setRegister(tcp, 2, protocolId);
}

void Modbus::setLength(uint16_t length) {
  if (length < 3 || length > 254) _setRegister(tcp, 4, 0);
  else _setRegister(tcp, 4, length);
}

void Modbus::setUnitId(uint8_t unitId) {
  tcp[6] = unitId;
}

void Modbus::setFunctionCode(uint8_t functionCode) {
  pdu[0] = functionCode;
}

void Modbus::setDataRegister(uint8_t index, uint16_t value) {
  _setRegister(data, index, value);
}



void Modbus::setRtuLen(uint16_t rtuLen) {
  setLength(rtuLen - 2);
}

void Modbus::setTcpLen(uint16_t tcpLen) {
  setLength(tcpLen - 6);
}

void Modbus::setPduLen(uint16_t pduLen) {
  setLength(pduLen + 1);
}

void Modbus::setDataLen(uint16_t dataLen) {
  setLength(dataLen + 2);
}



uint16_t Modbus::getTransactionId() {
  return _getRegister(tcp, 0);
}

uint16_t Modbus::getProtocolId() {
  return _getRegister(tcp, 2);
}

uint16_t Modbus::getLength() {
  uint16_t length = _getRegister(tcp, 4);
  if (length < 3 || length > 254) return 0;
  else return length;
}

uint8_t Modbus::getUnitId() {
  return tcp[6];
}

uint8_t Modbus::getFunctionCode() {
  return pdu[0];
}

uint16_t Modbus::getDataRegister(uint8_t index) {
  return _getRegister(data, index);
}



uint16_t Modbus::getRtuLen() {
  uint16_t len = getLength();
  if (len == 0) return 0;
  else return len + 2;
}

uint16_t Modbus::getTcpLen() {
  uint16_t len = getLength();
  if (len == 0) return 0;
  else return len + 6;
}

uint16_t Modbus::getPduLen() {
  uint16_t len = getLength();
  if (len == 0) return 0;
  else return len - 1;
}

uint16_t Modbus::getDataLen() {
  uint16_t len = getLength();
  if (len == 0) return 0;
  else return len - 2;
}



void Modbus::updateCrc(uint8_t *buf, uint16_t len) {
  uint16_t crc = _calculateCrc(buf, len);
  buf[len] = lowByte(crc);
  buf[len + 1] = highByte(crc);
}

bool Modbus::crcGood(uint8_t *buf, uint16_t len) {
  uint16_t aduCrc = buf[len] | (buf[len + 1] << 8);
  uint16_t calculatedCrc = _calculateCrc(buf, len);
  if (aduCrc == calculatedCrc) return true;
  else return false;
}

void Modbus::_setRegister(uint8_t *buf, uint16_t index, uint16_t value) {
  buf[index] = highByte(value);
  buf[index + 1] = lowByte(value);
}

uint16_t Modbus::_getRegister(uint8_t *buf, uint16_t index) {
  return (buf[index] << 8) | buf[index + 1];
}

uint16_t Modbus::_calculateCrc(uint8_t *buf, uint16_t len) {
  uint16_t value = 0xFFFF;
  for (uint16_t i = 0; i < len; i++) {
    value ^= (uint16_t)buf[i];
    for (uint8_t j = 0; j < 8; j++) {
      bool lsb = value & 1;
      value >>= 1;
      if (lsb == true) value ^= 0xA001;
    }
  }
  return value;
}



uint16_t div8RndUp(uint16_t value) {
  return (value + 7) >> 3;
}

void Modbus::clearRxBuffer() {
  unsigned long startMicros = micros();
  do {
    if (_serialD->available() > 0) {
      startMicros = micros();
      _serialD->read();
    }
  } while (micros() - startMicros < _frameTimeout);
}


/************************************************************
 * Modbus implementation
 ************************************************************/


// Constructor for Modbus
Modbus::Modbus(uint8_t busNo, HardwareSerial &serial, unsigned long baud, uint16_t cycleTimeMS, int8_t txPin, int waitA, int waitB) {
  _baud = baud;
  _serialD = &serial;
  _txPin = txPin;
  _busNo = busNo;
  _cycleTime = cycleTimeMS * 1000UL; // convert from milliseconds to microseconds.
  _waitA = waitA;
  _waitB = waitB;
  if (_waitA < 3) _waitA = 3;
  if (_waitB < 2) _waitB = 2;
  // Add device to HAL device chain
  IODevice::addDevice(this);
  
  // Add bus to Modbus chain.
  _nextBus = _busList;
  _busList = this;
}

// Main loop function for Modbus.
// Work through list of nodes.  For each node, in separate loop entries
// When the slot time has finished, move on to the next device.
void Modbus::_loop(unsigned long currentMicros) {
  _currentMicros = currentMicros;
  
  if (_currentNode == NULL) {
    _currentNode = _nodeListStart;
    
  }

  if (_currentMicros - _cycleStartTime < _cycleTime) return;
  _cycleStartTime = _currentMicros;
  if (_currentNode == NULL) return;
  const char* errorStrings[16] = { "success", "invalid id", "invalid buffer", "invalid quantity", "response timeout", "frame error", "crc error", "unknown comm error", "unexpected id", "exception response", "unexpected function code", "unexpected response length", "unexpected byte count", "unexpected address", "unexpected value", "unexpected quantity" };
  
  bool flagOK = true;
#if defined(MODBUS_STM_COMM)
  ArduinoPins::fastWriteDigital(MODBUS_STM_COMM,HIGH);
#endif



  if (error == MODBUS_RTU_MASTER_WAITING) {
    if (_waitCounter > _waitA) { // retry after 10 cycles of waiting, or user setting waitA.
      _waitCounter = 0;
      _waitCounterB++;
    } else {
      _waitCounter++;
    }
    if (_waitCounterB > _waitB) { // move on to next node if fails 10 times, or user setting waitB.
      _waitCounter = 0;
      _waitCounterB = 0;
      _operationCount = 0;
      
      _currentNode = _currentNode->getNext();
    }
  } else {
    _waitCounter = 0;
    _waitCounterB = 0;
  }
  
  if (error == MODBUS_RTU_MASTER_SUCCESS) { // should have the effect of retrying same opperation until success
    if (_operationCount < 3) { //              unless it fails waitB and moves on to next node. may even
      _operationCount++; //                    improve error recovery...
    } else {
      _operationCount = 0;
      _currentNode = _currentNode->getNext();
    }
  }
#if defined(MODBUS_STM_OK)
  if (flagOK == true) {
    ArduinoPins::fastWriteDigital(MODBUS_STM_OK,HIGH);
  } else {
    ArduinoPins::fastWriteDigital(MODBUS_STM_OK,LOW);
  }
#endif
#if defined(MODBUS_STM_FAIL)
  if (flagOK == false) {
    ArduinoPins::fastWriteDigital(MODBUS_STM_FAIL,HIGH);
  } else {
    ArduinoPins::fastWriteDigital(MODBUS_STM_FAIL,LOW);
  }
#endif
#if defined(MODBUS_STM_COMM)
  ArduinoPins::fastWriteDigital(MODBUS_STM_COMM,LOW);
#endif
  
}

// Link to chain of Modbus instances
Modbus *Modbus::_busList = NULL;


/************************************************************
 * Modbusnode implementation
 ************************************************************/

// Constructor for Modbusnode object
Modbusnode::Modbusnode(VPIN firstVpin, int nPins, uint8_t busNo, uint8_t nodeID) {
  _firstVpin = firstVpin;
  _nPins = nPins;
  _busNo = busNo;
  _nodeID = nodeID;
  if (_nodeID > 255) _nodeID = 255;

  // Add this device to HAL device list
  IODevice::addDevice(this);
  _display();
  // Add Modbusnode to Modbus object.
  Modbus *bus = Modbus::findBus(_busNo);
  if (bus != NULL) {
    bus->addNode(this);
    return;
  }
  
}
