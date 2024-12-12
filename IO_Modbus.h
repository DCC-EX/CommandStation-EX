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
#include <vector>
uint16_t  div8RndUp(uint16_t value);


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
  // EX-IOExpander protocol flags
  enum {
    EXIOINIT = 0xE0,    // Flag to initialise setup procedure
    EXIORDY = 0xE1,     // Flag we have completed setup procedure, also for EX-IO to ACK setup
    EXIODPUP = 0xE2,    // Flag we're sending digital pin pullup configuration
    EXIOVER = 0xE3,     // Flag to get version
    EXIORDAN = 0xE4,    // Flag to read an analogue input
    EXIOWRD = 0xE5,     // Flag for digital write
    EXIORDD = 0xE6,     // Flag to read digital input
    EXIOENAN = 0xE7,    // Flag to enable an analogue pin
    EXIOINITA = 0xE8,   // Flag we're receiving analogue pin mappings
    EXIOPINS = 0xE9,    // Flag we're receiving pin counts for buffers
    EXIOWRAN = 0xEA,   // Flag we're sending an analogue write (PWM)
    EXIOERR = 0xEF,     // Flag we've received an error
  };
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
  enum ProfileType : int {
    Instant = 0,  // Moves immediately between positions (if duration not specified)
    UseDuration = 0, // Use specified duration
    Fast = 1,     // Takes around 500ms end-to-end
    Medium = 2,   // 1 second end-to-end
    Slow = 3,     // 2 seconds end-to-end
    Bounce = 4,   // For semaphores/turnouts with a bit of bounce!!
    NoPowerOff = 0x80, // Flag to be ORed in to suppress power off after move.
  };

  uint8_t _numDigitalPins = 0;
  uint8_t _numAnaloguePins = 0;

  uint8_t _majorVer = 0;
  uint8_t _minorVer = 0;
  uint8_t _patchVer = 0;

  uint8_t* _digitalInputStates  = NULL;
  uint8_t* _analogueInputStates = NULL;
  uint8_t* _analogueInputBuffer = NULL;  // buffer for I2C input transfers
  uint8_t _readCommandBuffer[1];

  uint8_t _digitalPinBytes = 0;   // Size of allocated memory buffer (may be longer than needed)
  uint8_t _analoguePinBytes = 0;  // Size of allocated memory buffer (may be longer than needed)
  uint8_t* _analoguePinMap = NULL;
  I2CRB _i2crb;
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
    if (paramCount != 1) return false;
    int pin = vpin - _firstVpin;
    if (configType == CONFIGURE_INPUT) {
      Modbus* mb = Modbus::findBus(0);
       mb->_CommMode = 2;
       mb->_pullup = params[0];
       mb->_pin = pin;
       mb->_opperation = 1;
    } else if (configType == CONFIGURE_ANALOGINPUT) {
      // TODO:  Consider moving code from _configureAnalogIn() to here and remove _configureAnalogIn
      // from IODevice class definition.  Not urgent, but each virtual function defined
      // means increasing the RAM requirement of every HAL device driver, whether it's relevant
      // to the driver or not.
      return false;
    }
    return false;
  }

  int _configureAnalogIn(VPIN vpin) override {
    int pin = vpin - _firstVpin;
    Modbus* mb = Modbus::findBus(0);
    mb->_CommMode = 2;
    mb->_pin = pin;
    mb ->_opperation = 2;
    
    return false;
  }

  void _begin() override {
    Modbus* mb = Modbus::findBus(0);
    if (mb->_txPin != VPIN_NONE) {
      pinMode(mb->_txPin, OUTPUT);
      ArduinoPins::fastWriteDigital(mb->_txPin, LOW);
    }
    uint8_t receiveBuffer[5];
    uint8_t commandBuffer[7] = {EXIOINIT, _nodeID, (uint8_t)_nPins, (uint8_t)(_firstVpin & 0xFF), (uint8_t)(_firstVpin >> 8)};
    mb->updateCrc(commandBuffer,sizeof(commandBuffer));
     if (mb->_txPin != VPIN_NONE) ArduinoPins::fastWriteDigital(mb->_txPin, HIGH);
    mb->_serialD->write(commandBuffer, sizeof(commandBuffer));
    mb->_serialD->flush();
    if (mb->_txPin != VPIN_NONE) ArduinoPins::fastWriteDigital(mb->_txPin, LOW);
    unsigned long startMillis = millis();
    while (!mb->_serialD->available()) {
      if (millis() - startMillis >= 500) return;
    }
    uint16_t len = 0;
    unsigned long startMicros = micros();
    do {
      if (mb->_serialD->available()) {
        startMicros = micros();
        receiveBuffer[len] = mb->_serialD->read();
        len++;
      }
    } while (micros() - startMicros <= 500 && len < 256);
    if (receiveBuffer[0] == EXIOPINS && mb->crcGood(receiveBuffer,sizeof(receiveBuffer)-2)) {
      _numDigitalPins = receiveBuffer[1];
      _numAnaloguePins = receiveBuffer[2];

      // See if we already have suitable buffers assigned
      if (_numDigitalPins>0) {
        size_t digitalBytesNeeded = (_numDigitalPins + 7) / 8;
        if (_digitalPinBytes < digitalBytesNeeded) {
          // Not enough space, free any existing buffer and allocate a new one
          if (_digitalPinBytes > 0) free(_digitalInputStates);
          if ((_digitalInputStates = (byte*) calloc(digitalBytesNeeded, 1)) != NULL) {
            _digitalPinBytes = digitalBytesNeeded;
          } else {
            DIAG(F("EX-IOExpanderMB node:%d ERROR alloc %d bytes"), _nodeID, digitalBytesNeeded);
            _deviceState = DEVSTATE_FAILED;
            _digitalPinBytes = 0;
            return;
          }
        }
      }
      
      if (_numAnaloguePins>0) {
        size_t analogueBytesNeeded = _numAnaloguePins * 2;
        if (_analoguePinBytes < analogueBytesNeeded) {
          // Free any existing buffers and allocate new ones.
          if (_analoguePinBytes > 0) {
            free(_analogueInputBuffer);
            free(_analogueInputStates);
            free(_analoguePinMap);
          }
          _analogueInputStates = (uint8_t*) calloc(analogueBytesNeeded, 1);
          _analogueInputBuffer = (uint8_t*) calloc(analogueBytesNeeded, 1);
          _analoguePinMap = (uint8_t*) calloc(_numAnaloguePins, 1);
          if (_analogueInputStates  != NULL &&
            _analogueInputBuffer != NULL &&
            _analoguePinMap != NULL) {
            _analoguePinBytes = analogueBytesNeeded;
          } else {
            DIAG(F("EX-IOExpanderMB node:%d ERROR alloc analog pin bytes"), _nodeID);
            _deviceState = DEVSTATE_FAILED;
            _analoguePinBytes = 0;
            return;
          }
        }
      }
    } else {
      DIAG(F("EX-IOExpanderMB node:%d ERROR configuring device (CRC: %s)"), _nodeID, mb->crcGood(receiveBuffer,sizeof(receiveBuffer)-2)? "PASS":"FAIL");
      _deviceState = DEVSTATE_FAILED;
      return;
    }
    commandBuffer[0] = EXIOINITA;
    mb->updateCrc(commandBuffer,sizeof(commandBuffer));
    if (mb->_txPin != VPIN_NONE) ArduinoPins::fastWriteDigital(mb->_txPin, HIGH);
    mb->_serialD->write(commandBuffer, sizeof(commandBuffer));
    mb->_serialD->flush();
    if (mb->_txPin != VPIN_NONE) ArduinoPins::fastWriteDigital(mb->_txPin, LOW);
    startMillis = millis();
    while (!mb->_serialD->available()) {
      if (millis() - startMillis >= 500) return;
    }
    uint16_t len = 0;
    unsigned long startMicros = micros();
    do {
      if (mb->_serialD->available()) {
        startMicros = micros();
        receiveBuffer[len] = mb->_serialD->read();
        len++;
      }
    } while (micros() - startMicros <= 500 && len < 256);
    if (mb->crcGood(receiveBuffer,sizeof(receiveBuffer)-2)) {
      for (int i = 0; i < _numAnaloguePins; i++) {
        _analoguePinMap[i] = receiveBuffer[i];
      }
    }
    uint8_t versionBuffer[5];
    commandBuffer[0] = EXIOVER;
    mb->updateCrc(commandBuffer,sizeof(commandBuffer));
    if (mb->_txPin != VPIN_NONE) ArduinoPins::fastWriteDigital(mb->_txPin, HIGH);
    mb->_serialD->write(commandBuffer, sizeof(commandBuffer));
    mb->_serialD->flush();
    if (mb->_txPin != VPIN_NONE) ArduinoPins::fastWriteDigital(mb->_txPin, LOW);
    startMillis = millis();
    while (!mb->_serialD->available()) {
      if (millis() - startMillis >= 500) return;
    }
    uint16_t len = 0;
    unsigned long startMicros = micros();
    do {
      if (mb->_serialD->available()) {
        startMicros = micros();
        versionBuffer[len] = mb->_serialD->read();
        len++;
      }
    } while (micros() - startMicros <= 500 && len < 256);
    if (mb->crcGood(versionBuffer,sizeof(versionBuffer)-2)) {
      _majorVer = versionBuffer[0];
      _minorVer = versionBuffer[1];
      _patchVer = versionBuffer[2];
      DIAG(F("EX-IOExpander device found, node:%d, Version v%d.%d.%d"), _nodeID, _majorVer, _minorVer, _patchVer);
    }






#ifdef DIAG_IO
    _display();
#endif
    _initialised = false;
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
  uint8_t _adu[262];
  uint16_t _calculateCrc(uint8_t *buf, uint16_t len);
  uint16_t _getRegister(uint8_t *buf, uint16_t index);
  void _setRegister(uint8_t *buf, uint16_t index, uint16_t value);
  unsigned long _baud;
  
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

  int _waitCounter = 0;
  int _waitCounterB = 0;
  int _waitA;
  int _waitB;
// Helper function for error handling
  void reportError(uint8_t status, bool fail=true) {
    DIAG(F("EX-IOExpanderMB Node:%d Error"), _currentNode->getNodeID());
    if (fail)
    _deviceState = DEVSTATE_FAILED;
  }

  
  unsigned long _charTimeout;
  unsigned long _frameTimeout;
  enum {RDS_IDLE, RDS_DIGITAL, RDS_ANALOGUE};  // Read operation states
  uint8_t _readState = RDS_IDLE;
  
  unsigned long _lastDigitalRead = 0;
  unsigned long _lastAnalogueRead = 0;
  const unsigned long _digitalRefresh = 10000UL;    // Delay refreshing digital inputs for 10ms
  const unsigned long _analogueRefresh = 50000UL;   // Delay refreshing analogue inputs for 50ms

  // EX-IOExpander protocol flags
  enum {
    EXIOINIT = 0xE0,    // Flag to initialise setup procedure
    EXIORDY = 0xE1,     // Flag we have completed setup procedure, also for EX-IO to ACK setup
    EXIODPUP = 0xE2,    // Flag we're sending digital pin pullup configuration
    EXIOVER = 0xE3,     // Flag to get version
    EXIORDAN = 0xE4,    // Flag to read an analogue input
    EXIOWRD = 0xE5,     // Flag for digital write
    EXIORDD = 0xE6,     // Flag to read digital input
    EXIOENAN = 0xE7,    // Flag to enable an analogue pin
    EXIOINITA = 0xE8,   // Flag we're receiving analogue pin mappings
    EXIOPINS = 0xE9,    // Flag we're receiving pin counts for buffers
    EXIOWRAN = 0xEA,   // Flag we're sending an analogue write (PWM)
    EXIOERR = 0xEF,     // Flag we've received an error
  };
  int tasks[255][25];
  int taskCnt = 0;
public:
  void addTask(int taskNum, int paranCnt, int *param[]) {
    tasks[taskCnt][0] = taskNum;
    switch(taskNum) {
      case 0: // configure pin
        tasks[taskNum][1] = param[0]; // pin
        tasks[taskNum][2] = param[1]; // configtype
        tasks[taskNum][3] = param[2]; // paramcount
        for (int i=0; i < param[2]; i++) {
          tasks[taskNum][i+4] = param[i+3]; // params
        }
        break;
      case 1: // configure analog in
        tasks[taskNum][1] = param[0]; // pin
        break;
      

    }
  }

  int8_t _txPin;
  uint8_t *rtu = _adu + 6;
  uint8_t *tcp = _adu;
  uint8_t *pdu = _adu + 7;
  uint8_t *data = _adu + 8;
  void updateCrc(uint8_t *buf, uint16_t len);
  bool crcGood(uint8_t *buf, uint16_t len);
  uint16_t getLength();
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
  
  uint8_t getUnitId();
  uint8_t getFunctionCode();
  uint16_t getDataRegister(uint8_t index);

  uint16_t getRtuLen();
  uint16_t getTcpLen();
  uint16_t getPduLen();
  uint16_t getDataLen();
  void clearRxBuffer();
  static void create(uint8_t busNo, HardwareSerial& serial, unsigned long baud, uint16_t cycleTimeMS=500, int8_t txPin=-1, int waitA=10, int waitB=10) {
    new Modbus(busNo, serial, baud, cycleTimeMS, txPin, waitA, waitB);
  }
  HardwareSerial *_serialD;
  // Device-specific initialisation
  void _begin() override {
    _serialD->begin(_baud, SERIAL_8N1);
    unsigned long bitsPerChar = 10;
    if (_baud <= 19200) {
      _charTimeout = (bitsPerChar * 2500000) / _baud;
      _frameTimeout = (bitsPerChar * 4500000) / _baud;
    }
    else {
      _charTimeout = (bitsPerChar * 1000000) / _baud + 750;
      _frameTimeout = (bitsPerChar * 1000000) / _baud + 1750;
    }
    clearRxBuffer();
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
  int _CommMode = 0;
  int _opperation = 0;
  uint16_t _pullup;
  uint16_t _pin;

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
