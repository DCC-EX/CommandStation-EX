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
class ModbusADU {
  public:
    uint8_t *rtu = _adu + 6;
    uint8_t *tcp = _adu;
    uint8_t *pdu = _adu + 7;
    uint8_t *data = _adu + 8;

    void setTransactionId(uint16_t transactionId);
    void setProtocolId(uint16_t protocolId);
    void setLength(uint16_t length);
    void setUnitId(uint8_t unitId);
    void setFunctionCode(uint8_t functionCode);
    void setDataRegister(uint8_t index, uint16_t value);

    void setRtuLen(uint16_t rtuLen);
    void setTcpLen(uint16_t tcpLen);
    void setPduLen(uint16_t pduLen);
    void setDataLen(uint16_t dataLen);

    uint16_t getTransactionId();
    uint16_t getProtocolId();
    uint16_t getLength();
    uint8_t getUnitId();
    uint8_t getFunctionCode();
    uint16_t getDataRegister(uint8_t index);

    uint16_t getRtuLen();
    uint16_t getTcpLen();
    uint16_t getPduLen();
    uint16_t getDataLen();

    void updateCrc();
    bool crcGood();

    void prepareExceptionResponse(uint8_t exceptionCode);

  private:
    uint8_t _adu[262];
    void _setRegister(uint8_t *buf, uint16_t index, uint16_t value);
    uint16_t _getRegister(uint8_t *buf, uint16_t index);
    uint16_t _calculateCrc(uint16_t len);
    
};

uint16_t  div8RndUp(uint16_t value);

enum ModbusRTUCommError : uint8_t {
  MODBUS_RTU_COMM_SUCCESS = 0,
  MODBUS_RTU_COMM_TIMEOUT = 1,
  MODBUS_RTU_COMM_FRAME_ERROR = 2,
  MODBUS_RTU_COMM_CRC_ERROR = 3,
  MODBUS_RTU_COMM_WAITING = 4
};

class ModbusRTUComm {
  public:
    ModbusRTUComm(Stream& serial, VPIN dePin = VPIN_NONE, VPIN rePin = VPIN_NONE);
    void begin(unsigned long baud, uint32_t config = SERIAL_8N1);
    void setTimeout(unsigned long timeout);
    ModbusRTUCommError readAdu(ModbusADU& adu);
    bool _waiting_for_read = false;
    void writeAdu(ModbusADU& adu);
    void clearRxBuffer();
    Stream& _serial;
    VPIN _dePin;
    VPIN _rePin;
  private:
    
    unsigned long _charTimeout;
    unsigned long _frameTimeout;
    unsigned long _postDelay = 0UL;
    unsigned long _readTimeout = 0UL;
    unsigned long _startTimeout = 0UL;
};

enum ModbusRTUMasterError : uint8_t {
  MODBUS_RTU_MASTER_SUCCESS = 0,
  MODBUS_RTU_MASTER_INVALID_ID = 1,
  MODBUS_RTU_MASTER_INVALID_BUFFER = 2,
  MODBUS_RTU_MASTER_INVALID_QUANTITY = 3,
  MODBUS_RTU_MASTER_RESPONSE_TIMEOUT = 4,
  MODBUS_RTU_MASTER_FRAME_ERROR = 5,
  MODBUS_RTU_MASTER_CRC_ERROR = 6,
  MODBUS_RTU_MASTER_UNKNOWN_COMM_ERROR = 7,
  MODBUS_RTU_MASTER_UNEXPECTED_ID = 8,
  MODBUS_RTU_MASTER_EXCEPTION_RESPONSE = 9,
  MODBUS_RTU_MASTER_UNEXPECTED_FUNCTION_CODE = 10,
  MODBUS_RTU_MASTER_UNEXPECTED_LENGTH = 11,
  MODBUS_RTU_MASTER_UNEXPECTED_BYTE_COUNT = 12,
  MODBUS_RTU_MASTER_UNEXPECTED_ADDRESS = 13,
  MODBUS_RTU_MASTER_UNEXPECTED_VALUE = 14,
  MODBUS_RTU_MASTER_UNEXPECTED_QUANTITY = 15,
  MODBUS_RTU_MASTER_WAITING = 16
};


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
  static const uint8_t _numCoils=100;
  static const uint8_t _numDiscreteInputs=100;
  static const uint8_t _numHoldingRegisters=100;
  static const uint8_t _numInputRegisters=100;
  uint8_t _numBO=0;
  uint8_t _numBI=0;
  uint8_t _numAO=0;
  uint8_t _numAI=0;
  int dataBO[16];
  int dataBI[16];
  int dataAO[84];
  int dataAI[84];
  int capePinsBI[16];
  int capePinsBO[16];
  int capePinsPU[16];
  int capePinsAO[16];
  int capePinsAI[16];
  int configBPinsO[16];
  int configBPinsI[16];
  int configBPinsPU[16];
  int configAPinsO[16];
  int configAPinsI[16];
  
  void resetInit() {
    for (int i = 0; i < 16; i++) {
      capePinsBI[i] = 0;
      capePinsBO[i] = 0;
      capePinsPU[i] = 0;
      capePinsAO[i] = 0;
      capePinsAI[i] = 0;
      configBPinsO[i] = 0;
      configBPinsI[i] = 0;
      configBPinsPU[i] = 0;
      configAPinsO[i] = 0;
      configAPinsI[i] = 0;
    }
  }

  

  void spitError(int pin) {
    bool isBI = false;
    bool isBO = false;
    bool isPU = false;
    bool isAI = false;
    bool isAO = false;
    int configPinNum = pin / 16;
    int configPinBit = pin % 16;
    if (bitRead(configBPinsI[configPinNum],configPinBit) == true) isBI = true;
    if (bitRead(configBPinsO[configPinNum],configPinBit) == true) isBO = true;
    if (bitRead(configBPinsPU[configPinNum],configPinBit) == true) isPU = true;
    if (bitRead(configAPinsI[configPinNum],configPinBit) == true) isAI = true;
    if (bitRead(configAPinsO[configPinNum],configPinBit) == true) isAO = true;
    if (isBI && isPU) DIAG(F("IO_Modbus config eror: Bool Input with pull-up, pin: %d"),pin);
    if (isBI && !isPU) DIAG(F("IO_Modbus config eror: Bool Input without pull-up, pin: %d"),pin);
    if (isBO) DIAG(F("IO_Modbus config eror: Bool Output, pin: %d"),pin);
    if (isAI) DIAG(F("IO_Modbus config eror: Analog Input, pin: %d"),pin);
    if (isAO) DIAG(F("IO_Modbus config eror: Analog Output, pin: %d"),pin);
    
  }
  
public:
  static void create(VPIN firstVpin, int nPins, uint8_t busNo, uint8_t nodeID) {
    if (checkNoOverlap(firstVpin, nPins)) new Modbusnode(firstVpin, nPins, busNo, nodeID);
  }
  Modbusnode(VPIN firstVpin, int nPins, uint8_t busNo, uint8_t nodeID);
  int *coils[_numCoils];
  int *discreteInputs[_numDiscreteInputs];
  uint16_t *holdingRegisters[_numHoldingRegisters];
  uint16_t *inputRegisters[_numInputRegisters];

  uint8_t getNodeID() {
    return _nodeID;
  }
  uint8_t getNumCoils() {
    return _numCoils;
  }
  uint8_t getNumDiscreteInputs() {
    return _numDiscreteInputs;
  }
  uint8_t getNumHoldingRegisters() {
    return _numHoldingRegisters;
  }
  uint8_t getNumInputRegisters() {
    return _numInputRegisters;
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

  bool addPinBI(VPIN vpin, bool inputPullup) {
    int configPinNum = vpin / 16;
    int configPinBit = vpin % 16;
    bitSet(configBPinsI[configPinNum],configPinBit); // input
    bitWrite(configBPinsPU[configPinNum],configPinBit,inputPullup);
    if (_numBI + _numBO + _numAI + _numAO > _nPins) {
      DIAG(F("IO_Modbus config error: Too many I/O pins vs VPINs: %d"),_numBI + _numBO + _numAI + _numAO);
      return true;
    }
    _numBI++;
    return false;
  }
  
  bool addPinBO(VPIN vpin) {
    int configPinNum = vpin / 16;
    int configPinBit = vpin % 16;
    bitSet(configBPinsO[configPinNum],configPinBit); // input
    if (_numBI + _numBO + _numAI + _numAO > _nPins) {
      DIAG(F("IO_Modbus config error: Too many I/O pins vs VPINs: %d"),_numBI + _numBO + _numAI + _numAO);
      return true;
    }
    _numBO++;
    return false;
  }
  
  bool addPinAI(VPIN vpin) {
    int configPinNum = vpin / 6;
    int configPinBit = vpin % 16;
    bitSet(configAPinsI[configPinNum],configPinBit); // input
    if (_numBI + _numBO + _numAI + _numAO > _nPins) {
      DIAG(F("IO_Modbus config error: Too many I/O pins vs VPINs: %d"),_numBI + _numBO + _numAI + _numAO);
      return true;
    }
    _numAI++;
    return false;
  }
  
  bool addPinAO(VPIN vpin) {
    int configPinNum = vpin / 6;
    int configPinBit = vpin % 16;
    bitSet(configAPinsO[configPinNum],configPinBit); // input
    if (_numBI + _numBO + _numAI + _numAO > _nPins) {
      DIAG(F("IO_Modbus config error: Too many I/O pins vs VPINs: %d"),_numBI + _numBO + _numAI + _numAO);
      return true;
    }
    _numBI++;
    return false;
  }
  
  bool _configure(VPIN vpin, ConfigTypeEnum configType, int paramCount, int params[]) override {
    byte arduinoPin = params[1];
    if (paramCount != 1) return false;
    int pin = vpin - _firstVpin;
    if (configType == CONFIGURE_INPUT) {
      bool inputPullup = false;
      if (params[2] == 1) inputPullup = true;
      bool status = addPinBI(vpin,inputPullup);
      if (status == false) {
        return true;
      } else
        DIAG(F("IO_Modbus Vpin %u, Arduino Pin %d, cannot be used as a digital input pin"), (int)vpin, arduinoPin);
    } else if (configType == CONFIGURE_OUTPUT) {
      bool status = addPinBO(vpin);
      if (status == false) {
        return true;
      } else
        DIAG(F("IO_Modbus Vpin %u, Arduino Pin %d, cannot be used as a digital Output pin"), (int)vpin, arduinoPin);
    } else if (configType == CONFIGURE_SERVO) {
      //blah
    } else if (configType == CONFIGURE_ANALOGOUTPUT) {
      bool status = addPinAO(vpin);
      if (status == false) {
        return true;
      } else
        DIAG(F("IO_Modbus Vpin %u, Arduino Pin %d, cannot be used as a analog Output pin"), (int)vpin, arduinoPin);
    } else if (configType == CONFIGURE_ANALOGINPUT) {
      bool status = addPinAI(vpin);
      if (status == false) {
        return true;
      } else
        DIAG(F("IO_Modbus Vpin %u, Arduino Pin %d, cannot be used as a analog Input pin"), (int)vpin, arduinoPin);
    }
    return false;
  }

  void _begin() override {
    resetInit();
    coils[0] = (int*) 1; // set config mode
    coils[1] = (int*) 1; // set pull capabilities
    
    _initialised = false;
  }

  

  void procData() {
    if (isInitialised()) { // read/write data
      for (int i = 0; i < 16; i++) {
        holdingRegisters[i] = (uint16_t*) dataBO[i];
        dataBI[i] = (int) inputRegisters[i];
      }
      for (int i = 16; i < 84; i++) {
        holdingRegisters[i] = (uint16_t*) dataAO[i];
        dataAI[i] = (int) inputRegisters[i];
      }
    } else { // read/write config
      if (discreteInputs[0] == (int*) 1 && discreteInputs[1] == (int*) 1){ // get capabilities
        for (int i = 0; i < 16; i++) { // read capabilities params
          capePinsBI[i] = (int) inputRegisters[i];
          capePinsBO[i] = (int) inputRegisters[i+16];
          capePinsPU[i] = (int) inputRegisters[i+32];
          capePinsAI[i] = (int) inputRegisters[i+48];
          capePinsAO[i] = (int) inputRegisters[i+64];
        }
        coils[0] = (int*) 1; // config mode
        coils[1] = (int*) 0; // exit cape mode
        for (int i = 0; i < 16; i++) { // load config params
          holdingRegisters[i] = (uint16_t*) configBPinsI[i];
          holdingRegisters[i+16] = (uint16_t*) configBPinsO[i];
          holdingRegisters[i+32] = (uint16_t*) configBPinsPU[i];
          holdingRegisters[i+48] = (uint16_t*) configAPinsI[i];
          holdingRegisters[i+64] = (uint16_t*) configAPinsO[i];
        }
      } else if (discreteInputs[0] == (int*) 1 && discreteInputs[1] == (int*) 0) {
        for (int i = 0; i < 16; i++) {
          if (configBPinsI[i] != (int) inputRegisters[i]) spitError(i);
          if (configBPinsO[i] != (int) inputRegisters[i+16]) spitError(i+16);
          if (configBPinsPU[i] != (int) inputRegisters[i+32]) spitError(i+32);
          if (configAPinsI[i] != (int) inputRegisters[i+48]) spitError(i+48);
          if (configAPinsO[i] != (int) inputRegisters[i+64]) spitError(i+64);
        }
        // todo error checking and set failure mode
        // for now, assume everything is fine, sort this out later if needed
        coils[0] = (int*) 0; // exit config mode
        coils[1] = (int*) 0; // not in cape mode
        setInitialised();
      }
    }
  }

  int _read(VPIN vpin) override {
    // Return current state from this device
    uint16_t pin = vpin - _firstVpin;
    int PinNum = pin / 16;
    int PinBit = pin % 16;
    if (bitRead(configAPinsI[PinNum],PinBit) == true) return bitRead(dataBI[PinNum],PinBit)? 1:0;
    else return 0;
  }

  
  void _write(VPIN vpin, int value) override {
    // Update current state for this device, in preparation the bus transmission
    uint16_t pin = vpin - _firstVpin;
    int PinNum = pin / 16;
    int PinBit = pin % 16;
    if (bitRead(configAPinsO[PinNum], PinBit) == true) {
      if (value == 1) bitSet(dataBO[PinNum], PinBit);
      else bitClear(dataBO[PinNum], PinBit);
    }
  }

  int _readAnalogue(VPIN vpin) {
    // Return acquired data value, e.g.
    uint16_t pin = vpin - _firstVpin;
    int PinNum = pin / 16;
    int PinBit = pin % 16;
    if (bitRead(configAPinsI[PinNum],PinBit) == true) return dataAI[pin];
    else return 0;
  }
  
  void _writeAnalogue(VPIN vpin, int value) {
    uint16_t pin = vpin - _firstVpin;
    int PinNum = pin / 16;
    int PinBit = pin % 16;
    if (bitRead(configAPinsI[PinNum],PinBit) == true) dataAO[pin] = value;
  }

  uint8_t getBusNumber() {
    return _busNo;
  }
  uint8_t getNumBinaryInputsVPINsMin() {
    if (_numDiscreteInputs > 0) return _firstVpin;
    else return 0;
  }
  uint8_t getNumBinaryInputsVPINsMax() {
    if (_numDiscreteInputs > 0) return _firstVpin+_numDiscreteInputs-1;
    else return 0;
  }

  uint8_t getNumBinaryOutputsVPINsMin() {
    if (_numCoils > 0) return _firstVpin+_numDiscreteInputs;
    else return 0;
  }
  uint8_t getNumBinaryOutputsVPINsMax() {
    if (_numCoils > 0) return _firstVpin+_numDiscreteInputs+_numCoils-1;
    else return 0;
  }

  uint8_t getNumAnalogInputsVPINsMin() {
    if (_numInputRegisters > 0) return _firstVpin+_numDiscreteInputs+_numCoils;
    else return 0;
  }
  uint8_t getNumAnalogInputsVPINsMax() {
    if (_numInputRegisters > 0) return _firstVpin+_numDiscreteInputs+_numCoils+_numInputRegisters-1;
    else return 0;
  }

  uint8_t getNumAnalogOutputsVPINsMin() {
    if (_numHoldingRegisters > 0) return _firstVpin+_numDiscreteInputs+_numCoils+_numInputRegisters;
    else return 0;
  }
  uint8_t getNumAnalogOutputsVPINsMax() {
    if (_numHoldingRegisters > 0) return _firstVpin+_numDiscreteInputs+_numCoils+_numInputRegisters+_numHoldingRegisters-1;
    else return 0;
  }
  void _display() override {
    DIAG(F("Modbusnode configured on bus:%d nodeID:%d VPINs:%u-%u (B In) %u-%u (B Out) %u-%u (A In) %u-%u (A Out)"),
      _busNo, _nodeID, getNumBinaryInputsVPINsMin(), getNumBinaryInputsVPINsMax(),
      getNumBinaryOutputsVPINsMin(), getNumBinaryOutputsVPINsMax(),
      getNumAnalogInputsVPINsMin(), getNumAnalogInputsVPINsMax(),
      getNumAnalogOutputsVPINsMin(), getNumAnalogOutputsVPINsMax());
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
  int8_t _txPin;
  Modbusnode *_nodeListStart = NULL, *_nodeListEnd = NULL;
  Modbusnode *_currentNode = NULL;
  uint8_t _exceptionResponse = 0;
  uint8_t getExceptionResponse();
  uint16_t _receiveDataIndex = 0;  // Index of next data byte to be received.
  Modbus *_nextBus = NULL;  // Pointer to next bus instance in list.
  void setTimeout(unsigned long timeout);
  unsigned long _cycleStartTime = 0;
  unsigned long _timeoutStart = 0;
  unsigned long _cycleTime; // target time between successive read/write cycles, microseconds
  unsigned long _timeoutPeriod; // timeout on read responses, in microseconds.
  unsigned long _currentMicros;  // last value of micros() from _loop function.
  unsigned long _postDelay; // delay time after transmission before switching off transmitter (in us)
  unsigned long _byteTransmitTime; // time in us for transmission of one byte
  int _operationCount = 0;
  static Modbus *_busList; // linked list of defined bus instances
  ModbusRTUMasterError _readValues(uint8_t id, uint8_t functionCode, uint16_t startAddress, int buf[], uint16_t quantity);
  ModbusRTUMasterError _readValues(uint8_t id, uint8_t functionCode, uint16_t startAddress, uint16_t buf[], uint16_t quantity);
  ModbusRTUMasterError _writeSingleValue(uint8_t id, uint8_t functionCode, uint16_t address, uint16_t value);
  int _waitCounter = 0;
  int _waitCounterB = 0;
  int _waitA;
  int _waitB;

  void _resetWaiting() {
    _rtuComm._waiting_for_read = false;
  }
public:
  static void create(uint8_t busNo, HardwareSerial& serial, unsigned long baud, uint16_t cycleTimeMS=500, int8_t txPin=-1, int waitA=10, int waitB=10) {
    new Modbus(busNo, serial, baud, cycleTimeMS, txPin, waitA, waitB);
  }
  HardwareSerial *_serialD;
  ModbusRTUComm _rtuComm;
  // Device-specific initialisation
  void _begin() override {
    _serialD->begin(_baud, SERIAL_8N1);
    _rtuComm.begin(_baud, SERIAL_8N1);
  #if defined(MODBUS_STM_OK)
    pinMode(MODBUS_STM_OK, OUTPUT);
    ArduinoPins::fastWriteDigital(MODBUS_STM_OK,LOW);
  #endif
  #if defined(MODBUS_STM_FAIL)
    pinMode(MODBUS_STM_FAIL, OUTPUT);
    ArduinoPins::fastWriteDigital(MODBUS_STM_FAIL,LOW);
  #endif
  #if defined(MODBUS_STM_COMM)
    pinMode(MODBUS_STM_COMM, OUTPUT);
    ArduinoPins::fastWriteDigital(MODBUS_STM_COMM,LOW);
  #endif
  
  #if defined(DIAG_IO)
    _display();
  #endif
  }
  ModbusRTUMasterError _translateCommError(ModbusRTUCommError commError);
  ModbusRTUMasterError readCoils(uint8_t id, uint16_t startAddress, int buf[], uint16_t quantity);
  ModbusRTUMasterError readDiscreteInputs(uint8_t id, uint16_t startAddress, int buf[], uint16_t quantity);
  ModbusRTUMasterError readHoldingRegisters(uint8_t id, uint16_t startAddress, uint16_t buf[], uint16_t quantity);
  ModbusRTUMasterError readInputRegisters(uint8_t id, uint16_t startAddress, uint16_t buf[], uint16_t quantity);

  ModbusRTUMasterError writeSingleCoil(uint8_t id, uint16_t address, int value);
  ModbusRTUMasterError writeSingleHoldingRegister(uint8_t id, uint16_t address, uint16_t value);
  ModbusRTUMasterError writeMultipleCoils(uint8_t id, uint16_t startAddress, int buf[], uint16_t quantity);
  ModbusRTUMasterError writeMultipleHoldingRegisters(uint8_t id, uint16_t startAddress, uint16_t buf[], uint16_t quantity);
  // Loop function (overriding IODevice::_loop(unsigned long))
  void _loop(unsigned long currentMicros) override;

  // Display information about the device
  void _display() override {
    DIAG(F("Modbus Configured on Vpins:%d-%d %S"), _firstVpin, _firstVpin+_nPins-1,
      _deviceState == DEVSTATE_FAILED ? F("OFFLINE") : F("OK"));
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
  //DIAG(F("Modbus: 260h nodeID:%d _nodeListStart:%d _nodeListEnd:%d"), newNode, _nodeListStart, _nodeListEnd);
  }

protected:
  Modbus(uint8_t busNo, HardwareSerial &serial, unsigned long baud, uint16_t cycleTimeMS, int8_t txPin, int waitA, int waitB);

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
