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
 * EXIO485
 * =======
 * To define a EXIO485, example syntax:
 *    EXIO485::create(busNo, serial, baud[, TxPin]);
 * 
 * busNo = the Bus no of the instance. should = 0, unless more than one bus configured for some reason.
 * serial = serial port to be used (e.g. Serial3)
 * baud = baud rate (9600, 19200, 28800, 57600 or 115200)
 * cycletime = minimum time between successive updates/reads of a node in millisecs (default 500ms)
 * TxPin = pin number connected to EXIO485 module's DE and !RE terminals for half-duplex operation (default -1)
 *       if omitted (default), hardware MUST support full-duplex opperation!
 *
 * 
 * EXIO485Node
 * ========
 * To define a EXIO485 node and associate it with a EXIO485 bus,
 *    EXIO485node::create(firstVPIN, numVPINs, nodeID);
 * 
 * firstVPIN = first vpin in block allocated to this device
 * numVPINs = number of vpins
 * nodeID = 1-252
 */

#ifndef IO_EXIO485_H
#define IO_EXIO485_H

#include "IODevice.h"

class EXIO485;
class EXIO485node;



#ifndef COMMAND_BUFFER_SIZE
 #define COMMAND_BUFFER_SIZE 900
#endif

/**********************************************************************
 * Data Structure
 * 
 * Data Frame:
 * 0xFE : 0xFE : CRC : CRC : ByteCount : DataPacket : 0xFD : 0xFD
 * --------------------------------------------------------------
 * Start Frame : CRC Bytes : Data Size :    Data    : End Frame
 * 
 * Data frame must always start with the Start Frame bytes (two Bytes),
 * follow with the CRC bytes (two bytes), the data byte count
 * (one byte), the Data Packet (variable bytes), and the end Frame
 * Bytes.
 * 
 * 
 * Data Packet:
 * NodeTo : NodeFrom : AddrCode : ~Command Params~
 * -----------------------------------------------
 * NodeTo = where the packet is destined for.
 * NodeFrom = where the packet came from.
 * Address Code = from EXIO enumeration.
 * Command Params:
 * 
 * EXIOINIT:TX CS
 * --------
 * nPins : FirstPinL : FirstPinH
 * -----------------------------
 * nPins = Number of allocated pins.
 * FirstPinL = First VPIN lowByte.
 * FirstPinH = First VPIN highByte.
 * 
 * Sends the allocated pins.
 * 
 * EXIOINITA: Tx CS
 * -=no parameters, just a header=-
 * 
 * requests the analog pin map from the node.
 * 
 * EXIOVER: Tx CS
 * -=no parameters=-
 * 
 * requests the node software version, but as yet to do anything with it
 * 
 * EXIODPUP: Tx CS
 * pin : pullup
 * 
 * pin = VPIN number
 * pullup = 1 - Pullup, 0 - no pullup
 * configures a digital pin for input
 * 
 * EXIOENAN: TX CS
 * pin : FirstPinL : FirstPinH
 * 
 * pin = VPIN number
 * FirstPinL = first pin lowByte
 * FirstPinH = first pin highByte
 * 
 * EXIOWRD: TX CS
 * pin : value
 * 
 * pin = VPIN number
 * value = 1 or 0
 * 
 * EXIOWRAN: TX CS
 * pin : valueL : valueH : profile : durationL : durationH
 * 
 * pin = VPIN Number
 * valueL = value lowByte
 * valueH = value highByte
 * profile = servo profile
 * dueationL = duration lowByte
 * durationH = duration highByte
 * 
 * EXIORDD: TX CS
 * -=No Parameters=-
 * 
 * Requests digital pin states.
 * 
 * EXIORDAN: TX CS
 * -=no parameters=-
 * 
 * Requests analog pin states.
 * 
 * EXIOPINS: TX Node (EXIOINIT)
 * numDigital : numAnalog
 * 
 * numDigital = number of digital capable pins
 * numAnalog = number of analog capable pins
 * 
 * EXIOINITA: TX Node (EXIOINITA)
 * ~analog pin map~
 * 
 * each byte is a analog pin map value, variable length.
 * 
 * EXIORDY/EXIOERR: TX Node (EXIODPUP, EXIOWRD, EXIOENAN, EXIOWRAN)
 * -=no parameters=-
 * 
 * Responds EXIORDY for OK, and EXIOERR for FAIL.
 * 
 * EXIORDAN: TX Node (EXIORDAN)
 * ~analog pin states~
 * 
 * each byte is a pin state value, perhaps in lowByte/higeByte config.
 * 
 * EXIORDD: TX Node (EXIORDD)
 * ~digital pin states~
 * 
 * each byte is a 8-bit grouping of pinstates.
 * 
 * EXIOVER: TX Node (EXIOVER)
 * Major Version : Minor Version : Patch Version
 * 
 * each byte represents a numeric version value.
 **********************************************************************/




/**********************************************************************
 * EXIO485node class
 * 
 * This encapsulates the state associated with a single EXIO485 node, 
 * which includes the nodeID, number of discrete inputs and coils, and
 * the states of the discrete inputs and coils.
 **********************************************************************/
class EXIO485node : public IODevice {
private:
  uint8_t _busNo;
  uint8_t _nodeID;
  char _type;
  EXIO485node *_next = NULL;
  bool _initialised;
  EXIO485 *bus;
  HardwareSerial* _serial;
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
  static const int ARRAY_SIZE = 254;
public:
  static EXIO485node *_nodeList;
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
  uint8_t getnumDigialPins() {
    return _numDigitalPins;
  }
  void setnumDigitalPins(uint8_t value) {
    _numDigitalPins = value;
  }
  uint8_t _numAnaloguePins = 0;
  uint8_t getnumAnalogPins() {
    return _numAnaloguePins;
  }
  void setnumAnalogPins(uint8_t value) {
    _numAnaloguePins = value;
  }
  uint8_t _majorVer = 0;
  uint8_t getMajVer() {
    return _majorVer;
  }
  void setMajVer(uint8_t value) {
    _majorVer = value;
  }
  uint8_t _minorVer = 0;
  uint8_t getMinVer() {
    return _minorVer;
  }
  void setMinVer(uint8_t value) {
    _minorVer = value;
  }
  uint8_t _patchVer = 0;
  uint8_t getPatVer() {
    return _patchVer;
  }
  void setPatVer(uint8_t value) {
    _patchVer = value;
  }
  uint8_t* _digitalInputStates  = NULL;
  uint8_t getdigitalInputStates(int index) {
    return _digitalInputStates[index];
  }
  void setdigitalInputStates(uint8_t value, int index) {
    _digitalInputStates[index] = value;
  }
  bool cleandigitalPinStates(int size) {
    if (_digitalPinBytes > 0) free(_digitalInputStates);
    if ((_digitalInputStates = (byte*) calloc(size, 1)) != NULL) {
      return true;
    } else return false;
  }
  uint8_t* _analogueInputStates = NULL;
  uint8_t getanalogInputStates(int index) {
    return _analogueInputStates[index];
  }
  void setanalogInputStates(uint8_t value, int index) {
    _analogueInputStates[index] = value;
  }

  uint8_t* _analogueInputBuffer = NULL;  // buffer for I2C input transfers
  uint8_t getanalogInpuBuffer(int index) {
    return _analogueInputBuffer[index];
  }
  void setanalogInputBuffer(uint8_t value, int index) {
    _analogueInputBuffer[index] = value;
    memcpy(_analogueInputStates, _analogueInputBuffer, _analoguePinBytes);
  }
  uint8_t _readCommandBuffer[4]; // unused?
  uint8_t _digitalPinBytes = 0;   // Size of allocated memory buffer (may be longer than needed)
  uint8_t getdigitalPinBytes() {
    return _digitalPinBytes;
  }
  void setdigitalPinBytes(uint8_t value) {
    _digitalPinBytes = value;
  }
  uint8_t _analoguePinBytes = 0;  // Size of allocated memory buffer (may be longer than needed)
  uint8_t getanalogPinBytes() {
    return _analoguePinBytes;
  }
  void setanalogPinBytes(uint8_t value) {
    _analoguePinBytes = value;
  }
  uint8_t* _analoguePinMap = NULL;
  uint8_t getanalogPinMap(int index) {
    return _analoguePinMap[index];
  }
  void setanalogPinMap(uint8_t value, int index) {
    _analoguePinMap[index] = value;
  }
  bool cleanAnalogStates(int size) {
    if (_analoguePinBytes > 0) {
      free(_analogueInputBuffer);
      free(_analogueInputStates);
      free(_analoguePinMap);
    }
    _analogueInputStates = (uint8_t*) calloc(size, 1);
    _analogueInputBuffer = (uint8_t*) calloc(size, 1);
    _analoguePinMap = (uint8_t*) calloc(_numAnaloguePins, 1);
    if (_analogueInputStates  != NULL && _analogueInputBuffer != NULL && _analoguePinMap != NULL) return true;
    else return false;
  }
  int  resFlag[255];
  bool _initalized;
  static void create(VPIN firstVpin, int nPins, uint8_t nodeID) {
    if (checkNoOverlap(firstVpin, nPins)) new EXIO485node(firstVpin, nPins, nodeID);
  }
  EXIO485node(VPIN firstVpin, int nPins, uint8_t nodeID);

  uint8_t getNodeID() {
    return _nodeID;
  }
  
  EXIO485node *getNext() {
    return _next;
  }
 
  void setNext(EXIO485node *node) {
    _next = node;
  }
  bool isInitialised() {
    return _initialised;
  }
  void setInitialised() {
    _initialised = true;
  }
  
  bool _configure(VPIN vpin, ConfigTypeEnum configType, int paramCount, int params[]) override;
  int _configureAnalogIn(VPIN vpin) override;
  void _begin() override;
  int _read(VPIN vpin) override;
  void _write(VPIN vpin, int value) override;
  int _readAnalogue(VPIN vpin) override;
  void _writeAnalogue(VPIN vpin, int value, uint8_t profile, uint16_t duration) override;

  uint8_t getBusNumber() {
    return _busNo;
  }
 
  void _display() override {
    DIAG(F("EX-IOExpander485 node:%d Vpins %u-%u %S"), _nodeID, (int)_firstVpin, (int)_firstVpin+_nPins-1, _deviceState == DEVSTATE_FAILED ? F("OFFLINE") : F(""));
  }
  
};

/**********************************************************************
 * EXIO485 class
 * 
 * This encapsulates the properties state of the bus and the
 * transmission and reception of data across that bus.  Each EXIO485
 * object owns a set of EXIO485node objects which represent the nodes
 * attached to that bus.
 **********************************************************************/
class EXIO485 : public IODevice {
private:
  // Here we define the device-specific variables.  
  uint8_t _busNo;
  unsigned long _cycleStartTime = 0;
  unsigned long _cycleStartTimeA = 0;
  unsigned long _timeoutStart = 0;
  unsigned long _retryTime; // target time between successive read/write cycles, microseconds
  unsigned long _timeoutPeriod; // timeout on read responses, in microseconds.
  unsigned long _currentMicros;  // last value of micros() from _loop function.
  unsigned long _postDelay; // delay time after transmission before switching off transmitter (in us)
  unsigned long _byteTransmitTime; // time in us for transmission of one byte
  int _operationCount = 0;
  int _refreshOperation = 0;
  byte bufferLength;
  static const int ARRAY_SIZE = 150;
  int buffer[ARRAY_SIZE]; 
  byte inCommandPayload;
  static EXIO485 *_busList; // linked list of defined bus instances
  bool waitReceive = false;
  int _waitCounter = 0;
  int _waitCounterB = 0;
  int _waitA;
  unsigned long _charTimeout;
  unsigned long _frameTimeout;
  enum {RDS_IDLE, RDS_DIGITAL, RDS_ANALOGUE};  // Read operation states
  uint8_t _readState = RDS_IDLE;
  
  unsigned long _lastDigitalRead = 0;
  unsigned long _lastAnalogueRead = 0;
  const unsigned long _digitalRefresh = 10000UL;    // Delay refreshing digital inputs for 10ms
  const unsigned long _analogueRefresh = 50000UL;   // Delay refreshing analogue inputs for 50ms


  
  
  EXIO485node *_nodeListStart = NULL, *_nodeListEnd = NULL;
  EXIO485node *_currentNode = NULL;
  uint16_t _receiveDataIndex = 0;  // Index of next data byte to be received.
  EXIO485 *_nextBus = NULL;  // Pointer to next bus instance in list.
  
  int byteCounter = 0;
public:
struct Task {
    static const int ARRAY_SIZE = 150;
    long taskID;
    uint8_t commandArray[ARRAY_SIZE];
    int byteCount;
    uint8_t retFlag;
    bool gotCallback;
    bool rxMode;
    int crcPassFail;
    bool completed;
    bool processed;
    unsigned long currentRetryTimer;
  };
  static const int MAX_TASKS = 100; // we don't want to run out of task slots, but memory?
  long taskIDCntr = 1;
  long CurrentTaskID = -1;
  int taskResendCount = 0;
  Task taskBuffer[MAX_TASKS];
  int currentTaskIndex = 0;

  void addTask(const uint8_t* cmd, int byteCount, uint8_t retFlag) {
    // Find an empty slot in the buffer
    int emptySlot = -1;
    for (int i = 0; i < MAX_TASKS; i++) {
      if (taskBuffer[i].completed) {
        emptySlot = i;
        break;
      }
    }
    // If no empty slot found, return (buffer full)
    if (emptySlot == -1) {
      DIAG(F("Task Buffer Full!"));
      return;
    }
    for (int i = 0; i < byteCount; i++) taskBuffer[emptySlot].commandArray[i] = cmd[i];
    taskBuffer[emptySlot].byteCount = byteCount;
    taskBuffer[emptySlot].retFlag = retFlag;
    taskBuffer[emptySlot].rxMode = false;
    taskBuffer[emptySlot].crcPassFail = 0;
    taskBuffer[emptySlot].gotCallback = false;
    taskBuffer[emptySlot].completed = false;
    taskBuffer[emptySlot].processed = false;
    taskBuffer[emptySlot].currentRetryTimer = 0UL;
    taskIDCntr++;
    if (taskIDCntr >= 5000000) taskIDCntr = 1;
    taskBuffer[emptySlot].taskID = taskIDCntr;
    currentTaskIndex = emptySlot; 
  }
  bool hasTasks() {
    for (int i = 0; i < MAX_TASKS; i++) {
      if (!taskBuffer[i].completed) {
        return true; // At least one task is not completed
      }
    }
    return false; // All tasks are completed
  }
  // Function to get a specific task by ID
  Task* getTaskById(int id) {
    for (int i = 0; i < MAX_TASKS; i++) {
      if (taskBuffer[i].taskID == id) {
        return &taskBuffer[i]; // Return a pointer to the task
      }
    }
    return nullptr; // Task not found
  }
  // Function to get the next task (optional)
  long getNextTaskId() {
    for (int i = 0; i < MAX_TASKS; i++) {
      if (!taskBuffer[i].completed) {
        return taskBuffer[i].taskID; 
      }
    }
    return -1; // No tasks available
  }
  // Function to mark a task as completed
  void markTaskCompleted(int id) {
    for (int i = 0; i < MAX_TASKS; i++) {
      if (taskBuffer[i].taskID == id) {
        taskBuffer[i].completed = true; // completed
        taskBuffer[i].taskID = -1; // unassigned
        taskBuffer[i].currentRetryTimer = 0UL; // stop timer
        CurrentTaskID = getNextTaskId();
        break;
      }
    }
  }
  bool flagEnd = false;
  bool flagEnded = false;
  bool flagStart = false;
  bool flagStarted = false;
  bool rxStart = false;
  bool rxEnd = false;
  bool crcPass = false;
  bool flagProc = false;
  uint16_t calculated_crc;
  int byteCount = 100;
  uint8_t received_data[ARRAY_SIZE];
  uint16_t received_crc;
  uint8_t crc[2];
  uint16_t crc16(uint8_t *data, uint16_t length);

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
  static void create(uint8_t busNo, HardwareSerial &serial, unsigned long baud, int8_t txPin=-1) {
    new EXIO485(busNo, serial, baud, txPin);
  }
  HardwareSerial* _serial;
  int _CommMode = 0;
  int _opperation = 0;
  uint16_t _pullup;
  uint16_t _pin;
  int8_t _txPin;
  int8_t getTxPin() {
    return _txPin;
  }
  bool _busy = false;
  void setBusy() {
    _busy = true;
  }
  void clearBusy() {
    _busy = false;
  }
  bool getBusy() {
    return _busy;
  }
  unsigned long _baud;
  int taskCnt = 0;
  uint8_t initBuffer[1] = {0xFE};
  unsigned long taskCounter=0ul;
  // Device-specific initialisation
  void _begin() override {
    _serial->begin(_baud, SERIAL_8N1);
    if (_txPin >0) {
      pinMode(_txPin, OUTPUT);
      digitalWrite(_txPin, LOW);
      
    }
  
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

  // Locate EXIO485node object with specified nodeID.
  EXIO485node *findNode(uint8_t nodeID) {
    for (EXIO485node *node = _nodeListStart; node != NULL; node = node->getNext()) {
      if (node->getNodeID() == nodeID) 
        return node;
    }
    return NULL;
  }

  bool nodesInitialized() {
    bool retval = true;
    for (EXIO485node *node = _nodeListStart; node != NULL; node = node->getNext()) {
      if (node->_initalized == false) 
        retval = false;
    }
    return retval;
  }
  // Add new EXIO485node to the list of nodes for this bus.
  void addNode(EXIO485node *newNode) {
    if (!_nodeListStart)
      _nodeListStart = newNode;
    if (!_nodeListEnd) 
      _nodeListEnd = newNode;
    else
      _nodeListEnd->setNext(newNode);
  //DIAG(F("EXIO485: 260h nodeID:%d _nodeListStart:%d _nodeListEnd:%d"), newNode, _nodeListStart, _nodeListEnd);
  }

protected:
  EXIO485(uint8_t busNo, HardwareSerial &serial, unsigned long baud, int8_t txPin);

public:
  
  uint8_t getBusNumber() {
    return _busNo;
  }
  EXIO485 *getNext() {
    return _nextBus;
  }
  static EXIO485 *findBus(uint8_t busNo) {
    for (EXIO485 *bus = _busList; bus != NULL; bus = bus->getNext()) {
      if (bus->getBusNumber() == busNo) 
        return bus;
    }
    return NULL;
  }
};


#endif // IO_EXIO485_H