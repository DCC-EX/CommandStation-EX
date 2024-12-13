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
 * RS485
 * =======
 * To define a RS485, example syntax:
 *    RS485::create(serial, baud[, cycletime[, pin]]);
 * 
 * serial = serial port to be used (e.g. Serial3)
 * baud = baud rate (9600, 19200, 28800, 57600 or 115200)
 * cycletime = minimum time between successive updates/reads of a node in millisecs (default 500ms)
 * pin = pin number connected to RS485 module's DE and !RE terminals for half-duplex operation (default VPIN_NONE)
 *
 * 
 * RS485Node
 * ========
 * To define a RS485 node and associate it with a RS485 bus,
 *    RS485node::create(firstVPIN, numVPINs, nodeID);
 * 
 * firstVPIN = first vpin in block allocated to this device
 * numVPINs = number of vpins
 * nodeID = 0-254
 */

#ifndef IO_RS485_H
#define IO_RS485_H

#include "IODevice.h"

/**********************************************************************
 * RS485node class
 * 
 * This encapsulates the state associated with a single RS485 node, 
 * which includes the nodeID, number of discrete inputs and coils, and
 * the states of the discrete inputs and coils.
 **********************************************************************/
class RS485node : public IODevice {
private:
  uint8_t _busNo;
  uint8_t _nodeID;
  char _type;
  RS485node *_next = NULL;
  bool _initialised = false;

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
  uint8_t _readCommandBuffer[4];

  uint8_t _digitalPinBytes = 0;   // Size of allocated memory buffer (may be longer than needed)
  uint8_t _analoguePinBytes = 0;  // Size of allocated memory buffer (may be longer than needed)
  uint8_t* _analoguePinMap = NULL;

  static void create(VPIN firstVpin, int nPins, uint8_t nodeID) {
    if (checkNoOverlap(firstVpin, nPins)) new RS485node(firstVpin, nPins, nodeID);
  }
  RS485node(VPIN firstVpin, int nPins, uint8_t nodeID);

  uint8_t getNodeID() {
    return _nodeID;
  }
  
  RS485node *getNext() {
    return _next;
  }
  void setNext(RS485node *node) {
    _next = node;
  }
  bool isInitialised() {
    return _initialised;
  }
  void setInitialised() {
    _initialised = true;
  }
  
  bool _configure(VPIN vpin, ConfigTypeEnum configType, int paramCount, int params[]) override {
    if (paramCount != 1) return false;
    int pin = vpin - _firstVpin;
    int pin = vpin - _firstVpin;
    RS485 *bus = RS485::findBus(_busNo);
    int* param[] = {(int*)pin, (int*)configType, (int*)paramCount, (int*)params[0]};
    bus->addTask(_nodeID, 3, 4, param);

  }

  int _configureAnalogIn(VPIN vpin) override {
    int pin = vpin - _firstVpin;
    RS485 *bus = RS485::findBus(_busNo);
    int* params[] = {(int*)pin};
    bus->addTask(_nodeID, 3, 1, params);

    return false;
  }

  void _begin() override {
    RS485 *bus = RS485::findBus(_busNo);
    if (bus->_txPin != VPIN_NONE) {
      pinMode(bus->_txPin, OUTPUT);
      ArduinoPins::fastWriteDigital(bus->_txPin, LOW);
    }
    uint8_t receiveBuffer[5];
    uint8_t commandBuffer[7] = {EXIOINIT, _nodeID, (uint8_t)_nPins, (uint8_t)(_firstVpin & 0xFF), (uint8_t)(_firstVpin >> 8)};
    bus->updateCrc(commandBuffer,5);
     if (bus->_txPin != VPIN_NONE) ArduinoPins::fastWriteDigital(bus->_txPin, HIGH);
    bus->_serialD->write(commandBuffer, 7);
    bus->_serialD->flush();
    if (bus->_txPin != VPIN_NONE) ArduinoPins::fastWriteDigital(bus->_txPin, LOW);
    unsigned long startMillis = millis();
    while (!bus->_serialD->available()) {
      if (millis() - startMillis >= 500) return;
    }
    uint16_t len = 0;
    unsigned long startMicros = micros();
    do {
      if (bus->_serialD->available()) {
        startMicros = micros();
        receiveBuffer[len] = bus->_serialD->read();
        len++;
      }
    } while (micros() - startMicros <= 500 && len < 256);
    if (receiveBuffer[1] == EXIOPINS && bus->crcGood(receiveBuffer,sizeof(receiveBuffer)-2)) {
      if (!bus->testAndStripMasterFlag(receiveBuffer)) DIAG(F("Forgen RS485 Device! no master flag from node %d"),_nodeID);
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
            DIAG(F("EX-IOExpander485 node:%d ERROR alloc %d bytes"), _nodeID, digitalBytesNeeded);
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
            DIAG(F("EX-IOExpander485 node:%d ERROR alloc analog pin bytes"), _nodeID);
            _deviceState = DEVSTATE_FAILED;
            _analoguePinBytes = 0;
            return;
          }
        }
      }
    } else {
      DIAG(F("EX-IOExpander485 node:%d ERROR configuring device (CRC: %s)"), _nodeID, bus->crcGood(receiveBuffer,sizeof(receiveBuffer)-2)? "PASS":"FAIL");
      _deviceState = DEVSTATE_FAILED;
      return;
    }
    commandBuffer[0] = EXIOINITA;
    commandBuffer[1] = _nodeID;
    bus->updateCrc(commandBuffer,2);
    if (bus->_txPin != VPIN_NONE) ArduinoPins::fastWriteDigital(bus->_txPin, HIGH);
    bus->_serialD->write(commandBuffer, 4);
    bus->_serialD->flush();
    if (bus->_txPin != VPIN_NONE) ArduinoPins::fastWriteDigital(bus->_txPin, LOW);
    startMillis = millis();
    while (!bus->_serialD->available()) {
      if (millis() - startMillis >= 500) return;
    }
    uint16_t len = 0;
    unsigned long startMicros = micros();
    do {
      if (bus->_serialD->available()) {
        startMicros = micros();
        receiveBuffer[len] = bus->_serialD->read();
        len++;
      }
    } while (micros() - startMicros <= 500 && len < 256);
    if (bus->crcGood(receiveBuffer,sizeof(receiveBuffer)-2)) {
      if (!bus->testAndStripMasterFlag(receiveBuffer)) DIAG(F("Forgen RS485 Device! no master flag from node %d"),_nodeID);
      for (int i = 0; i < _numAnaloguePins; i++) {
        _analoguePinMap[i] = receiveBuffer[i];
      }
    }
    uint8_t versionBuffer[5];
    commandBuffer[0] = EXIOVER;
    commandBuffer[1] = _nodeID;
    bus->updateCrc(commandBuffer,2);
    if (bus->_txPin != VPIN_NONE) ArduinoPins::fastWriteDigital(bus->_txPin, HIGH);
    bus->_serialD->write(commandBuffer, 4);
    bus->_serialD->flush();
    if (bus->_txPin != VPIN_NONE) ArduinoPins::fastWriteDigital(bus->_txPin, LOW);
    startMillis = millis();
    while (!bus->_serialD->available()) {
      if (millis() - startMillis >= 500) return;
    }
    uint16_t len = 0;
    unsigned long startMicros = micros();
    do {
      if (bus->_serialD->available()) {
        startMicros = micros();
        versionBuffer[len] = bus->_serialD->read();
        len++;
      }
    } while (micros() - startMicros <= 500 && len < 256);
    if (bus->crcGood(versionBuffer,sizeof(versionBuffer)-2)) {
      if (!bus->testAndStripMasterFlag(versionBuffer)) DIAG(F("Forgen RS485 Device! no master flag from node %d"),_nodeID);
      _majorVer = versionBuffer[0];
      _minorVer = versionBuffer[1];
      _patchVer = versionBuffer[2];
      DIAG(F("EX-IOExpander485 device found, node:%d, Version v%d.%d.%d"), _nodeID, _majorVer, _minorVer, _patchVer);
    }
#ifdef DIAG_IO
    _display();
#endif
    _initialised = false;
  }

  
  int _read(VPIN vpin) override {
    if (_deviceState == DEVSTATE_FAILED) return 0;
    int pin = vpin - _firstVpin;
    uint8_t pinByte = pin / 8;
    bool value = bitRead(_digitalInputStates[pinByte], pin - pinByte * 8);
    return value;
  }

  
  void _write(VPIN vpin, int value) override {
    if (_deviceState == DEVSTATE_FAILED) return;
    int pin = vpin - _firstVpin;
    RS485 *bus = RS485::findBus(_busNo);
    int* params[] = {(int*)pin, (int*)value};
    bus->addTask(_nodeID, 3, 2, params);
  }

  int _readAnalogue(VPIN vpin) override {
    if (_deviceState == DEVSTATE_FAILED) return 0;
    int pin = vpin - _firstVpin;
    for (uint8_t aPin = 0; aPin < _numAnaloguePins; aPin++) {
      if (_analoguePinMap[aPin] == pin) {
        uint8_t _pinLSBByte = aPin * 2;
        uint8_t _pinMSBByte = _pinLSBByte + 1;
        return (_analogueInputStates[_pinMSBByte] << 8) + _analogueInputStates[_pinLSBByte];
      }
    }
    return -1;  // pin not found in table
  }
  
  void _writeAnalogue(VPIN vpin, int value, uint8_t profile, uint16_t duration) override {
    uint8_t servoBuffer[7];
    uint8_t responseBuffer[1];

    if (_deviceState == DEVSTATE_FAILED) return;
    int pin = vpin - _firstVpin;
    RS485 *bus = RS485::findBus(_busNo);
    int* params[] = {(int*)pin, (int*)value, (int*)profile, (int*)duration};
    bus->addTask(_nodeID, 3, 4, params);
  }

  uint8_t getBusNumber() {
    return _busNo;
  }
 
  void _display() override {
    DIAG(F("EX-IOExpander485 node:%d v%d.%d.%d Vpins %u-%u %S"), _nodeID, _majorVer, _minorVer, _patchVer, (int)_firstVpin, (int)_firstVpin+_nPins-1, _deviceState == DEVSTATE_FAILED ? F("OFFLINE") : F(""));
  }
  

};

/**********************************************************************
 * RS485 class
 * 
 * This encapsulates the properties state of the bus and the
 * transmission and reception of data across that bus.  Each RS485
 * object owns a set of RS485node objects which represent the nodes
 * attached to that bus.
 **********************************************************************/
class RS485 : public IODevice {
private:
  // Here we define the device-specific variables.  
  uint8_t _busNo;
  unsigned long _baud;
  unsigned long _cycleStartTime = 0;
  unsigned long _timeoutStart = 0;
  unsigned long _cycleTime; // target time between successive read/write cycles, microseconds
  unsigned long _timeoutPeriod; // timeout on read responses, in microseconds.
  unsigned long _currentMicros;  // last value of micros() from _loop function.
  unsigned long _postDelay; // delay time after transmission before switching off transmitter (in us)
  unsigned long _byteTransmitTime; // time in us for transmission of one byte
  int _operationCount = 0;
  int _refreshOperation = 0;

  static RS485 *_busList; // linked list of defined bus instances
  bool waitReceive = false;
  int _waitCounter = 0;
  int _waitCounterB = 0;
  int _waitA;
  int* taskData[25];
  unsigned long _charTimeout;
  unsigned long _frameTimeout;
  enum {RDS_IDLE, RDS_DIGITAL, RDS_ANALOGUE};  // Read operation states
  uint8_t _readState = RDS_IDLE;
  
  unsigned long _lastDigitalRead = 0;
  unsigned long _lastAnalogueRead = 0;
  const unsigned long _digitalRefresh = 10000UL;    // Delay refreshing digital inputs for 10ms
  const unsigned long _analogueRefresh = 50000UL;   // Delay refreshing analogue inputs for 50ms
  int tasks[255][25];

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
  uint16_t _calculateCrc(uint8_t *buf, uint16_t len);
  
  RS485node *_nodeListStart = NULL, *_nodeListEnd = NULL;
  RS485node *_currentNode = NULL;
  uint8_t _exceptionResponse = 0;
  uint8_t getExceptionResponse();
  uint16_t _receiveDataIndex = 0;  // Index of next data byte to be received.
  RS485 *_nextBus = NULL;  // Pointer to next bus instance in list.
  void setTimeout(unsigned long timeout);
  
// Helper function for error handling
  void reportError(uint8_t status, bool fail=true) {
    DIAG(F("EX-IOExpander485 Node:%d Error"), _currentNode->getNodeID());
    if (fail)
    _deviceState = DEVSTATE_FAILED;
  }

  void _moveTasks() {
    // used one in lead, so move forward
    for (int i = 0; i < taskCnt-1; i++) {
      for (int j = 0; j < 25; j++) {
        tasks[i][j] = tasks[i+1][j+1];
      }
    }
    taskCnt--;
  }
public:
  int _CommMode = 0;
  int _opperation = 0;
  uint16_t _pullup;
  uint16_t _pin;
  int8_t _txPin;
  int taskCnt = 0;
  HardwareSerial *_serialD;
  bool testAndStripMasterFlag(uint8_t *buf) {
    if (buf[0] != 0xFF) return false; // why did we not get a master flag? bad node?
    for (int i = 0; i < sizeof(buf)-1; i++) buf[i] = buf[i+1]; // shift array to begining
    return true;
  }
  void addTask(int nodeID, int taskNum, int paramCnt, int *param[]) {
    taskCnt++;
    tasks[taskCnt][0] = nodeID;
    tasks[taskCnt][1] = taskNum;
    tasks[taskCnt][2] = paramCnt;
    switch(taskNum) {
      case 0:
        // empty task
      case 1: // configure pin
        tasks[taskCnt][3] = (int) param[0]; // pin
        tasks[taskCnt][4] = (int) param[1]; // configtype
        tasks[taskCnt][5] = (int) param[2]; // paramcount
        for (int i=0; i < (int) param[2]; i++) {
          tasks[taskCnt][i+6] = (int) param[i+3]; // params
        }
        break;
      case 2: // configure analog in
        tasks[taskCnt][3] = (int) param[0]; // pin
        break;
      case 3: // write pin
        tasks[taskCnt][3] = (int) param[0]; // pin
        tasks[taskCnt][4] = (int) param[1]; // value
        break;
      case 4: // write analog
        tasks[taskCnt][3] = (int) param[0]; // pin
        tasks[taskCnt][4] = (int) param[1]; // value
        tasks[taskCnt][5] = (int) param[2]; // profile
        tasks[taskCnt][6] = (int) param[3]; // duration
        break;
    }
  }
  
  int getNextTask(int *buf[]) {
    int paramCnt = 0;
    for (int i = 0; i < 25; i++) {
      if (i == 0) buf[i] = (int*) tasks[0][i]; // NodeID
      if (i == 1) buf[i] = (int*) tasks[0][i]; // tasknum
      else if (i == 2) paramCnt = tasks[0][i]; // paramcnt
      else {
        buf[i-1] = (int*) tasks[0][i];
      }
    }
    _moveTasks();
  }

  
  void updateCrc(uint8_t *buf, uint16_t len);
  bool crcGood(uint8_t *buf, uint16_t len);
  void clearRxBuffer();
  static void create(HardwareSerial& serial, unsigned long baud, uint16_t cycleTimeMS=500, int8_t txPin=-1, int waitA=10) {
    new RS485(serial, baud, cycleTimeMS, txPin, waitA);
  }
  
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
  #if defined(RS485_STM_OK)
    pinMode(RS485_STM_OK, OUTPUT);
    ArduinoPins::fastWriteDigital(RS485_STM_OK,LOW);
  #endif
  #if defined(RS485_STM_FAIL)
    pinMode(RS485_STM_FAIL, OUTPUT);
    ArduinoPins::fastWriteDigital(RS485_STM_FAIL,LOW);
  #endif
  #if defined(RS485_STM_COMM)
    pinMode(RS485_STM_COMM, OUTPUT);
    ArduinoPins::fastWriteDigital(RS485_STM_COMM,LOW);
  #endif
  
  #if defined(DIAG_IO)
    _display();
  #endif
  }


  // Loop function (overriding IODevice::_loop(unsigned long))
  void _loop(unsigned long currentMicros) override;

  // Display information about the device
  void _display() override {
    DIAG(F("EX-IOExpander485 Configured on Vpins:%d-%d %S"), _firstVpin, _firstVpin+_nPins-1,
      _deviceState == DEVSTATE_FAILED ? F("OFFLINE") : F("OK"));
  }

  // Locate RS485node object with specified nodeID.
  RS485node *findNode(uint8_t nodeID) {
    for (RS485node *node = _nodeListStart; node != NULL; node = node->getNext()) {
      if (node->getNodeID() == nodeID) 
        return node;
    }
    return NULL;
  }

  
  // Add new RS485node to the list of nodes for this bus.
  void addNode(RS485node *newNode) {
    if (!_nodeListStart)
      _nodeListStart = newNode;
    if (!_nodeListEnd) 
      _nodeListEnd = newNode;
    else
      _nodeListEnd->setNext(newNode);
  //DIAG(F("RS485: 260h nodeID:%d _nodeListStart:%d _nodeListEnd:%d"), newNode, _nodeListStart, _nodeListEnd);
  }

protected:
  RS485(HardwareSerial &serial, unsigned long baud, uint16_t cycleTimeMS, int8_t txPin, int waitA);

public:
  
  uint8_t getBusNumber() {
    return _busNo;
  }

  static RS485 *findBus(uint8_t busNo) {
    for (RS485 *bus=_busList; bus!=NULL; bus=bus->_nextBus) {
      if (bus->_busNo == busNo) return bus;
    }
    return NULL;
  }
};


#endif // IO_RS485_H