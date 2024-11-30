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
void ModbusADU::setTransactionId(uint16_t transactionId) {
  _setRegister(tcp, 0, transactionId);
}

void ModbusADU::setProtocolId(uint16_t protocolId) {
  _setRegister(tcp, 2, protocolId);
}

void ModbusADU::setLength(uint16_t length) {
  if (length < 3 || length > 254) _setRegister(tcp, 4, 0);
  else _setRegister(tcp, 4, length);
}

void ModbusADU::setUnitId(uint8_t unitId) {
  tcp[6] = unitId;
}

void ModbusADU::setFunctionCode(uint8_t functionCode) {
  pdu[0] = functionCode;
}

void ModbusADU::setDataRegister(uint8_t index, uint16_t value) {
  _setRegister(data, index, value);
}



void ModbusADU::setRtuLen(uint16_t rtuLen) {
  setLength(rtuLen - 2);
}

void ModbusADU::setTcpLen(uint16_t tcpLen) {
  setLength(tcpLen - 6);
}

void ModbusADU::setPduLen(uint16_t pduLen) {
  setLength(pduLen + 1);
}

void ModbusADU::setDataLen(uint16_t dataLen) {
  setLength(dataLen + 2);
}



uint16_t ModbusADU::getTransactionId() {
  return _getRegister(tcp, 0);
}

uint16_t ModbusADU::getProtocolId() {
  return _getRegister(tcp, 2);
}

uint16_t ModbusADU::getLength() {
  uint16_t length = _getRegister(tcp, 4);
  if (length < 3 || length > 254) return 0;
  else return length;
}

uint8_t ModbusADU::getUnitId() {
  return tcp[6];
}

uint8_t ModbusADU::getFunctionCode() {
  return pdu[0];
}

uint16_t ModbusADU::getDataRegister(uint8_t index) {
  return _getRegister(data, index);
}



uint16_t ModbusADU::getRtuLen() {
  uint16_t len = getLength();
  if (len == 0) return 0;
  else return len + 2;
}

uint16_t ModbusADU::getTcpLen() {
  uint16_t len = getLength();
  if (len == 0) return 0;
  else return len + 6;
}

uint16_t ModbusADU::getPduLen() {
  uint16_t len = getLength();
  if (len == 0) return 0;
  else return len - 1;
}

uint16_t ModbusADU::getDataLen() {
  uint16_t len = getLength();
  if (len == 0) return 0;
  else return len - 2;
}



void ModbusADU::updateCrc() {
  uint16_t len = getLength();
  uint16_t crc = _calculateCrc(len);
  rtu[len] = lowByte(crc);
  rtu[len + 1] = highByte(crc);
}

bool ModbusADU::crcGood() {
  uint16_t len = getLength();
  uint16_t aduCrc = rtu[len] | (rtu[len + 1] << 8);
  uint16_t calculatedCrc = _calculateCrc(len);
  if (aduCrc == calculatedCrc) return true;
  else return false;
}



void ModbusADU::prepareExceptionResponse(uint8_t exceptionCode) {
  pdu[0] |= 0x80;
  pdu[1] = exceptionCode;
  setPduLen(2);
}



void ModbusADU::_setRegister(uint8_t *buf, uint16_t index, uint16_t value) {
  buf[index] = highByte(value);
  buf[index + 1] = lowByte(value);
}

uint16_t ModbusADU::_getRegister(uint8_t *buf, uint16_t index) {
  return (buf[index] << 8) | buf[index + 1];
}

uint16_t ModbusADU::_calculateCrc(uint16_t len) {
  uint16_t value = 0xFFFF;
  for (uint16_t i = 0; i < len; i++) {
    value ^= (uint16_t)rtu[i];
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

ModbusRTUComm::ModbusRTUComm(Stream& serial, VPIN dePin, VPIN rePin) : _serial(serial) {
  _dePin = dePin;
  _rePin = rePin;
}

void ModbusRTUComm::begin(unsigned long baud, uint32_t config) {
  unsigned long bitsPerChar;
  switch (config) {
    case SERIAL_8E2:
    case SERIAL_8O2:
      bitsPerChar = 12;
      break;
    case SERIAL_8N2:
    case SERIAL_8E1:
    case SERIAL_8O1:
      bitsPerChar = 11;
      break;
    case SERIAL_8N1:
    default:
      bitsPerChar = 10;
      break;
  }
  if (baud <= 19200) {
    _charTimeout = (bitsPerChar * 2500000) / baud;
    _frameTimeout = (bitsPerChar * 4500000) / baud;
  }
  else {
    _charTimeout = (bitsPerChar * 1000000) / baud + 750;
    _frameTimeout = (bitsPerChar * 1000000) / baud + 1750;
  }
  #if defined(ARDUINO_UNOR4_MINIMA) || defined(ARDUINO_UNOR4_WIFI) || defined(ARDUINO_GIGA) || (defined(ARDUINO_NANO_RP2040_CONNECT) && defined(ARDUINO_ARCH_MBED))
  _postDelay = ((bitsPerChar * 1000000) / baud) + 2;
  #endif
  if (_dePin != VPIN_NONE) {
    pinMode(_dePin, OUTPUT);
    ArduinoPins::fastWriteDigital(_dePin, LOW);
  }
  if (_rePin != VPIN_NONE) {
    pinMode(_rePin, OUTPUT);
    ArduinoPins::fastWriteDigital(_rePin, LOW);
  }
  clearRxBuffer();
}

void ModbusRTUComm::setTimeout(unsigned long timeout) {
  _readTimeout = timeout;
}

ModbusRTUCommError ModbusRTUComm::readAdu(ModbusADU& adu) {
  adu.setRtuLen(0);
  unsigned long startMillis = millis();
  if (!_serial.available()) {
    //if (millis() - startMillis >= _readTimeout) return MODBUS_RTU_COMM_TIMEOUT;
    _waiting_for_read = true;
    if (millis() - startMillis >= _readTimeout) {
      //_serial.flush();
      return MODBUS_RTU_COMM_TIMEOUT;
    } else {
      return MODBUS_RTU_COMM_WAITING;
    }
    
  }
  _waiting_for_read = false;
  uint16_t len = 0;
  unsigned long startMicros = micros();
  do {
    if (_serial.available()) {
      startMicros = micros();
      adu.rtu[len] = _serial.read();
      len++;
    }
  } while (micros() - startMicros <= _charTimeout && len < 256);
  adu.setRtuLen(len);
  while (micros() - startMicros < _frameTimeout);
  if (_serial.available()) {
    adu.setRtuLen(0);
    return MODBUS_RTU_COMM_FRAME_ERROR;
  }
  if (!adu.crcGood()) {
    adu.setRtuLen(0);
    return MODBUS_RTU_COMM_CRC_ERROR;
  }
  return MODBUS_RTU_COMM_SUCCESS;
}

void ModbusRTUComm::writeAdu(ModbusADU& adu) {
  adu.updateCrc();
  if (_dePin != VPIN_NONE) ArduinoPins::fastWriteDigital(_dePin, HIGH);
  if (_rePin != VPIN_NONE) ArduinoPins::fastWriteDigital(_rePin, HIGH);
  _serial.write(adu.rtu, adu.getRtuLen());
  _serial.flush();
  ///delayMicroseconds(_postDelay); // TJF: Commented out as Mbed platforms are not supported
  if (_dePin != VPIN_NONE) ArduinoPins::fastWriteDigital(_dePin, LOW);
  if (_rePin != VPIN_NONE) ArduinoPins::fastWriteDigital(_rePin, LOW);
}

void ModbusRTUComm::clearRxBuffer() {
  unsigned long startMicros = micros();
  do {
    if (_serial.available() > 0) {
      startMicros = micros();
      _serial.read();
    }
  } while (micros() - startMicros < _frameTimeout);
}


void Modbus::setTimeout(unsigned long timeout) {
  _rtuComm.setTimeout(timeout);
}




ModbusRTUMasterError Modbus::readCoils(uint8_t id, uint16_t startAddress, int buf[], uint16_t quantity) {
  return _readValues(id, 1, startAddress, buf, quantity);
}

ModbusRTUMasterError Modbus::readDiscreteInputs(uint8_t id, uint16_t startAddress, int buf[], uint16_t quantity) {
  return _readValues(id, 2, startAddress, buf, quantity);
}

ModbusRTUMasterError Modbus::readHoldingRegisters(uint8_t id, uint16_t startAddress, uint16_t buf[], uint16_t quantity) {
  return _readValues(id, 3, startAddress, buf, quantity);
}

ModbusRTUMasterError Modbus::readInputRegisters(uint8_t id, uint16_t startAddress, uint16_t buf[], uint16_t quantity) {
  return _readValues(id, 4, startAddress, buf, quantity);
}



ModbusRTUMasterError Modbus::writeSingleCoil(uint8_t id, uint16_t address, int value) {
  return _writeSingleValue(id, 5, address, ((value) ? 0xFF00 : 0x0000));
}

ModbusRTUMasterError Modbus::writeSingleHoldingRegister(uint8_t id, uint16_t address, uint16_t value) {
  return _writeSingleValue(id, 6, address, value);
}



ModbusRTUMasterError Modbus::writeMultipleCoils(uint8_t id, uint16_t startAddress, int buf[], uint16_t quantity) {
  const uint8_t functionCode = 15;
  if (id > 247) return MODBUS_RTU_MASTER_INVALID_ID;
  if (!buf) return MODBUS_RTU_MASTER_INVALID_BUFFER;
  if (quantity == 0 || quantity > 1968) return MODBUS_RTU_MASTER_INVALID_QUANTITY;
  ModbusADU adu;
  uint16_t byteCount = div8RndUp(quantity);
  adu.setUnitId(id);
  adu.setFunctionCode(functionCode);
  adu.setDataRegister(0, startAddress);
  adu.setDataRegister(2, quantity);
  adu.data[4] = byteCount;
  for (uint16_t i = 0; i < quantity; i++) {
    bitWrite(adu.data[5 + (i >> 3)], i & 7, buf[i]);
  }
  for (uint16_t i = quantity; i < (byteCount * 8); i++) {
    bitClear(adu.data[5 + (i >> 3)], i & 7);
  }
  adu.setDataLen(5 + byteCount);
  if (_rtuComm._waiting_for_read == false) _rtuComm.writeAdu(adu);
  if (id == 0) return MODBUS_RTU_MASTER_SUCCESS;
  ModbusRTUCommError commError = _rtuComm.readAdu(adu);
  if (commError) return _translateCommError(commError);
  if (adu.getUnitId() != id) return MODBUS_RTU_MASTER_UNEXPECTED_ID;
  if (adu.getFunctionCode() == (functionCode + 0x80)) {
    _exceptionResponse = adu.data[0];
    return MODBUS_RTU_MASTER_EXCEPTION_RESPONSE;
  }
  if (adu.getFunctionCode() != functionCode) return MODBUS_RTU_MASTER_UNEXPECTED_FUNCTION_CODE;
  if (adu.getDataLen() != 4) return MODBUS_RTU_MASTER_UNEXPECTED_LENGTH;
  if (adu.getDataRegister(0) != startAddress) return MODBUS_RTU_MASTER_UNEXPECTED_ADDRESS;
  if (adu.getDataRegister(2) != quantity) return MODBUS_RTU_MASTER_UNEXPECTED_QUANTITY;
  return MODBUS_RTU_MASTER_SUCCESS;
}

ModbusRTUMasterError Modbus::writeMultipleHoldingRegisters(uint8_t id, uint16_t startAddress, uint16_t buf[], uint16_t quantity) {
  uint8_t functionCode = 16;
  if (id > 247) return MODBUS_RTU_MASTER_INVALID_ID;
  if (!buf) return MODBUS_RTU_MASTER_INVALID_BUFFER;
  if (quantity == 0 || quantity > 123) return MODBUS_RTU_MASTER_INVALID_QUANTITY;
  uint16_t byteCount = quantity * 2;
  ModbusADU adu;
  adu.setUnitId(id);
  adu.setFunctionCode(functionCode);
  adu.setDataRegister(0, startAddress);
  adu.setDataRegister(2, quantity);
  adu.data[4] = byteCount;
  for (uint16_t i = 0; i < quantity; i++) {
    adu.setDataRegister(5 + (i * 2), buf[i]);
  }
  adu.setDataLen(5 + byteCount);
  if (_rtuComm._waiting_for_read == false) _rtuComm.writeAdu(adu);
  if (id == 0) return MODBUS_RTU_MASTER_SUCCESS;
  ModbusRTUCommError commError = _rtuComm.readAdu(adu);
  if (commError) return _translateCommError(commError);
  if (adu.getUnitId() != id) return MODBUS_RTU_MASTER_UNEXPECTED_ID;
  if (adu.getFunctionCode() == (functionCode + 0x80)) {
    _exceptionResponse = adu.data[0];
    return MODBUS_RTU_MASTER_EXCEPTION_RESPONSE;
  }
  if (adu.getFunctionCode() != functionCode) return MODBUS_RTU_MASTER_UNEXPECTED_FUNCTION_CODE;
  if (adu.getDataLen() != 4) return MODBUS_RTU_MASTER_UNEXPECTED_LENGTH;
  if (adu.getDataRegister(0) != startAddress) return MODBUS_RTU_MASTER_UNEXPECTED_ADDRESS;
  if (adu.getDataRegister(2) != quantity) return MODBUS_RTU_MASTER_UNEXPECTED_QUANTITY;
  return MODBUS_RTU_MASTER_SUCCESS;
}



uint8_t Modbus::getExceptionResponse() {
  return _exceptionResponse;
}



ModbusRTUMasterError Modbus::_readValues(uint8_t id, uint8_t functionCode, uint16_t startAddress, int buf[], uint16_t quantity) {
  if (id < 1 || id > 247) return MODBUS_RTU_MASTER_INVALID_ID;
  if (!buf) return MODBUS_RTU_MASTER_INVALID_BUFFER;
  if (quantity == 0 || quantity > 2000) return MODBUS_RTU_MASTER_INVALID_QUANTITY;
  ModbusADU adu;
  adu.setUnitId(id);
  adu.setFunctionCode(functionCode);
  adu.setDataRegister(0, startAddress);
  adu.setDataRegister(2, quantity);
  adu.setDataLen(4);
  if (_rtuComm._waiting_for_read == false) _rtuComm.writeAdu(adu);
  ModbusRTUCommError commError = _rtuComm.readAdu(adu);
  if (commError) return _translateCommError(commError);
  if (adu.getUnitId() != id) return MODBUS_RTU_MASTER_UNEXPECTED_ID;
  if (adu.getFunctionCode() == (functionCode + 0x80)) {
    _exceptionResponse = adu.data[0];
    return MODBUS_RTU_MASTER_EXCEPTION_RESPONSE;
  }
  if (adu.getFunctionCode() != functionCode) return MODBUS_RTU_MASTER_UNEXPECTED_FUNCTION_CODE;
  uint16_t byteCount = div8RndUp(quantity);
  if (adu.getDataLen() != (1 + byteCount)) return MODBUS_RTU_MASTER_UNEXPECTED_LENGTH;
  if (adu.data[0] != byteCount) return MODBUS_RTU_MASTER_UNEXPECTED_BYTE_COUNT;
  for (uint16_t i = 0; i < quantity; i++) {
    buf[i] = (int) bitRead(adu.data[1 + (i >> 3)], i & 7)? 1:0;
  }
  return MODBUS_RTU_MASTER_SUCCESS;
}

ModbusRTUMasterError Modbus::_readValues(uint8_t id, uint8_t functionCode, uint16_t startAddress, uint16_t buf[], uint16_t quantity) {
  if (id < 1 || id > 247) return MODBUS_RTU_MASTER_INVALID_ID;
  if (!buf) return MODBUS_RTU_MASTER_INVALID_BUFFER;
  if (quantity == 0 || quantity > 125) return MODBUS_RTU_MASTER_INVALID_QUANTITY;
  ModbusADU adu;
  adu.setUnitId(id);
  adu.setFunctionCode(functionCode);
  adu.setDataRegister(0, startAddress);
  adu.setDataRegister(2, quantity);
  adu.setDataLen(4);
  if (_rtuComm._waiting_for_read == false) _rtuComm.writeAdu(adu);
  ModbusRTUCommError commError = _rtuComm.readAdu(adu);
  if (commError) return _translateCommError(commError);
  if (adu.getUnitId() != id) return MODBUS_RTU_MASTER_UNEXPECTED_ID;
  if (adu.getFunctionCode() == (functionCode + 0x80)) {
    _exceptionResponse = adu.data[0];
    return MODBUS_RTU_MASTER_EXCEPTION_RESPONSE;
  }
  if (adu.getFunctionCode() != functionCode) return MODBUS_RTU_MASTER_UNEXPECTED_FUNCTION_CODE;
  uint16_t byteCount = quantity * 2;
  if (adu.getDataLen() != (1 + byteCount)) return MODBUS_RTU_MASTER_UNEXPECTED_LENGTH;
  if (adu.data[0] != byteCount) return MODBUS_RTU_MASTER_UNEXPECTED_BYTE_COUNT;
  for (uint16_t i = 0; i < quantity; i++) {
    buf[i] = adu.getDataRegister(1 + (i * 2));
  }
  return MODBUS_RTU_MASTER_SUCCESS;
}

ModbusRTUMasterError Modbus::_writeSingleValue(uint8_t id, uint8_t functionCode, uint16_t address, uint16_t value) {
  if (id > 247) return MODBUS_RTU_MASTER_INVALID_ID;
  ModbusADU adu;
  adu.setUnitId(id);
  adu.setFunctionCode(functionCode);
  adu.setDataRegister(0, address);
  adu.setDataRegister(2, value);
  adu.setDataLen(4);
  if (_rtuComm._waiting_for_read == false) _rtuComm.writeAdu(adu);
  if (id == 0) return MODBUS_RTU_MASTER_SUCCESS;
  ModbusRTUCommError commError = _rtuComm.readAdu(adu);
  if (commError) return _translateCommError(commError);
  if (adu.getUnitId() != id) return MODBUS_RTU_MASTER_UNEXPECTED_ID;
  if (adu.getFunctionCode() == (functionCode + 0x80)) {
    _exceptionResponse = adu.data[0];
    return MODBUS_RTU_MASTER_EXCEPTION_RESPONSE;
  }
  if (adu.getFunctionCode() != functionCode) return MODBUS_RTU_MASTER_UNEXPECTED_FUNCTION_CODE;
  if (adu.getDataLen() != 4) return MODBUS_RTU_MASTER_UNEXPECTED_LENGTH;
  if (adu.getDataRegister(0) != address) return MODBUS_RTU_MASTER_UNEXPECTED_ADDRESS;
  if (adu.getDataRegister(2) != value) return MODBUS_RTU_MASTER_UNEXPECTED_VALUE;
  return MODBUS_RTU_MASTER_SUCCESS;
}



ModbusRTUMasterError Modbus::_translateCommError(ModbusRTUCommError commError) {
  switch (commError) {
    case MODBUS_RTU_COMM_SUCCESS:
      return MODBUS_RTU_MASTER_SUCCESS;
    case MODBUS_RTU_COMM_TIMEOUT:
      return MODBUS_RTU_MASTER_RESPONSE_TIMEOUT;
    case MODBUS_RTU_COMM_FRAME_ERROR:
      return MODBUS_RTU_MASTER_FRAME_ERROR;
    case MODBUS_RTU_COMM_CRC_ERROR:
      return MODBUS_RTU_MASTER_CRC_ERROR;
    case MODBUS_RTU_COMM_WAITING:
      return MODBUS_RTU_MASTER_WAITING;
    default:
      return MODBUS_RTU_MASTER_UNKNOWN_COMM_ERROR;
  }
}

/************************************************************
 * Modbus implementation
 ************************************************************/


// Constructor for Modbus
Modbus::Modbus(uint8_t busNo, HardwareSerial &serial, unsigned long baud, uint16_t cycleTimeMS, int8_t txPin) : _rtuComm(serial, txPin) {
  _baud = baud;
  _serialD = &serial;
  _txPin = txPin;
  _rtuComm.setTimeout(500);
  _busNo = busNo;
  _cycleTime = cycleTimeMS * 1000UL; // convert from milliseconds to microseconds.
  
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
  uint8_t error;
  // send reads and writes, DIAG on errors other than 0 (Success), or 3 (Invalid Quantity)
  switch (_operationCount) {
    case 0:
      error = writeMultipleHoldingRegisters(_currentNode->getNodeID(), 0, (uint16_t*) _currentNode->holdingRegisters, _currentNode->getNumHoldingRegisters());
      if (error != 0 && error != MODBUS_RTU_MASTER_WAITING) DIAG(F("ModbusHR: T%d F%d N%d %s"), _currentNode->getNodeID(), 0, _currentNode->getNumHoldingRegisters(), errorStrings[error]);
      if (error != 0 && (error != MODBUS_RTU_MASTER_WAITING || _waitCounter > 2)) flagOK = false;
      break;
    case 1:
      error = writeMultipleCoils(_currentNode->getNodeID(), 0, (int*) _currentNode->coils, _currentNode->getNumCoils());
      if (error != 0 && error != MODBUS_RTU_MASTER_WAITING) DIAG(F("ModbusMC: T%d F%d N%d %s"), _currentNode->getNodeID(), 0, _currentNode->getNumCoils(), errorStrings[error]);
      if (error != 0 && (error != MODBUS_RTU_MASTER_WAITING || _waitCounter > 2)) flagOK = false;
      break;
    case 2:
      error = readDiscreteInputs(_currentNode->getNodeID(), 0, (int*) _currentNode->discreteInputs, _currentNode->getNumDiscreteInputs());
      if (error != 0 && error != MODBUS_RTU_MASTER_WAITING) DIAG(F("ModbusDI: T%d F%d N%d %s"), _currentNode->getNodeID(), 0, _currentNode->getNumDiscreteInputs(), errorStrings[error]);
      if (error != 0 && (error != MODBUS_RTU_MASTER_WAITING || _waitCounter > 2)) flagOK = false;
      break;
    case 3:
      error = readInputRegisters(_currentNode->getNodeID(), 0, (uint16_t*) _currentNode->inputRegisters, _currentNode->getNumInputRegisters());
      if (error != 0 && error != MODBUS_RTU_MASTER_WAITING) DIAG(F("ModbusIR: T%d F%d N%d %s"), _currentNode->getNodeID(), 0, _currentNode->getNumInputRegisters(), errorStrings[error]);
      if (error != 0 && (error != MODBUS_RTU_MASTER_WAITING || _waitCounter > 2)) flagOK = false;
      break;
  }
  if (error == MODBUS_RTU_MASTER_WAITING) {
    if (_waitCounter > 10) { // retry after 10 cycles of waiting.
      _resetWaiting();
      _waitCounter = 0;
      _waitCounterB++;
    } else {
      _waitCounter++;
    }
    if (_waitCounterB > 10) { // move on to next node if fails 10 times.
      _waitCounter = 0;
      _waitCounterB = 0;
      _operationCount = 0;
      _currentNode = _currentNode->getNext();
    }
  } else {
    _waitCounter = 0;
    _waitCounterB = 0;
  }
  
  if (error != MODBUS_RTU_MASTER_WAITING) {
    if (_operationCount < 3) {
      _operationCount++;
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
Modbusnode::Modbusnode(VPIN firstVpin, int nPins, uint8_t busNo, uint8_t nodeID,  uint8_t numCoils, uint8_t numDiscreteInputs, uint8_t numHoldingRegisters, uint8_t numInputRegisters) {
  _firstVpin = firstVpin;
  _nPins = nPins;
  _busNo = busNo;
  _nodeID = nodeID;
  _numCoils = numCoils;
  _numDiscreteInputs = numDiscreteInputs;
  _numHoldingRegisters = numHoldingRegisters;
  _numInputRegisters = numInputRegisters;
  
  
  if ((unsigned int)_nPins < numDiscreteInputs + numCoils)
    DIAG(F("Modbusnode: bus:%d nodeID:%d WARNING number of Vpins does not cover all inputs and outputs"), _busNo, _nodeID);

  if (!discreteInputs || !coils) {
    DIAG(F("Modbusnode: ERROR insufficient memory"));
    return;
  }

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
