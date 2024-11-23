#include "ModbusADU.h"

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