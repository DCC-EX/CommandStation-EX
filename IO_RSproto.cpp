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

#include "IO_RSproto.h"
#include "defines.h"

/************************************************************
 * RSproto implementation
 ************************************************************/

// Constructor for RSproto
RSproto::RSproto(HardwareSerial &serial, unsigned long baud, uint16_t cycleTimeMS, int8_t txPin, int waitA) {
  _baud = baud;
  _serialD = &serial;
  _txPin = txPin;
  _busNo = 0;

  _cycleTime = cycleTimeMS * 1000UL; // convert from milliseconds to microseconds.
  _waitA = waitA;
  if (_waitA < 3) _waitA = 3;
  // Add device to HAL device chain
  IODevice::addDevice(this);
  
  // Add bus to RSproto chain.
  _nextBus = _busList;
  _busList = this;
}

/* -= updateCrc =-
//
// add the CRC value from _calculateCrc (2 bytes) to the buffer.
*/
void RSproto::updateCrc(uint8_t *buf, uint16_t len) {
  uint16_t crc = _calculateCrc(buf, len);
  buf[len] = lowByte(crc);
  buf[len + 1] = highByte(crc);
}

/* -= crcGood =-
//
// return TRUE if CRC matched between buffer copy, and calculated.
*/
bool RSproto::crcGood(uint8_t *buf, uint16_t len) {
  uint16_t aduCrc = buf[len] | (buf[len + 1] << 8);
  uint16_t calculatedCrc = _calculateCrc(buf, len);
#if defined(IO_DIAG)
  DIAG(F("CRC is %d Expected %d"),calculatedCrc, aduCrc);
#endif
  if (aduCrc == calculatedCrc) return true;
  else return false;
}

/* -= calculateCrc =-
//
// use bitwise XOR to calculate CRC into a 16-bit byte
*/
uint16_t RSproto::_calculateCrc(uint8_t *buf, uint16_t len) {
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

/* -= clearRxBuffer =-
//
// BLOCKING method to empty stray data in RX buffer
*/
void RSproto::clearRxBuffer() {
  unsigned long startMicros = micros();
  do {
    if (_serialD->available() > 0) {
      startMicros = micros();
      _serialD->read();
    }
  } while (micros() - startMicros < _frameTimeout || !_serialD->available());
}

/* -= _loop =-
//
// Main loop function for RSproto.
// Work through list of nodes.  For each node, in separate loop entries
// When the slot time has finished, move on to the next device.
*/
void RSproto::_loop(unsigned long currentMicros) {
  _currentMicros = currentMicros;
  
  if (_currentNode == NULL) {
    _currentNode = _nodeListStart;
    
  }

  if (_currentMicros - _cycleStartTime < _cycleTime) return;
  _cycleStartTime = _currentMicros;
  if (_currentNode == NULL) return;

  bool flagOK = true;
#if defined(RSproto_STM_COMM)
  ArduinoPins::fastWriteDigital(RSproto_STM_COMM,HIGH);
#endif
if (nodesInitialized()) {
  memcpy(_currentNode->_analogueInputStates, _currentNode->_analogueInputBuffer, _currentNode->_analoguePinBytes); // Copy I2C input buffer to states
  switch (_refreshOperation) {
    case 0:
      if (_currentNode->_numDigitalPins>0 && currentMicros - _lastDigitalRead > _digitalRefresh) { // Delay for digital read refresh
        // Issue new read request for digital states.  As the request is non-blocking, the buffer has to
        // be allocated from heap (object state).
        _currentNode->_readCommandBuffer[0] = _currentNode->getNodeID();
        _currentNode->_readCommandBuffer[1] = EXIORDD;
        updateCrc(_currentNode->_readCommandBuffer,sizeof(_currentNode->_readCommandBuffer)-2);
        if (waitReceive == false) {
            if (_txPin != VPIN_NONE) ArduinoPins::fastWriteDigital(_txPin, HIGH);
            _serialD->write(_currentNode->_readCommandBuffer, sizeof(_currentNode->_readCommandBuffer));
            _serialD->write(initBuffer, 1);
            _serialD->flush();
            if (_txPin != VPIN_NONE) ArduinoPins::fastWriteDigital(_txPin, LOW);
          }
          unsigned long startMillis = millis();
          if (!_serialD->available()) {
            if (waitReceive == true && _waitCounter > _waitA) {
              flagOK = false;
            } else waitReceive = true;
          }
        uint16_t len = 0;
        unsigned long startMicros = micros();
        bool rxDone = false;
    byte tmpByte;
    len = _serialD->readBytesUntil(0xFE,_currentNode->_digitalInputStates, 25);
        if (!true/*crcGood(_currentNode->_digitalInputStates,sizeof(_currentNode->_digitalInputStates)-2)*/) {
          DIAG(F("EX-IOExpander485 CRC error on node %d"), _currentNode->getNodeID());
          flagOK = false;
        }
        if (!testAndStripMasterFlag(_currentNode->_digitalInputStates)) DIAG(F("Foreign RSproto Device! no master flag from node %d"),_currentNode->getNodeID());
        if (!waitReceive) _refreshOperation++;
        _lastDigitalRead = currentMicros;
        _readState = RDS_DIGITAL;
      }
      break;
    case 1:
      if (_currentNode->_numAnaloguePins>0 && currentMicros - _lastAnalogueRead > _analogueRefresh) { // Delay for analogue read refresh
        // Issue new read for analogue input states
        _currentNode->_readCommandBuffer[0] = _currentNode->getNodeID();
        _currentNode->_readCommandBuffer[1] = EXIORDAN;
        updateCrc(_currentNode->_readCommandBuffer,sizeof(_currentNode->_readCommandBuffer)-2);
        if (waitReceive == false) {
            if (_txPin != VPIN_NONE) ArduinoPins::fastWriteDigital(_txPin, HIGH);
            _serialD->write(_currentNode->_readCommandBuffer, sizeof(_currentNode->_readCommandBuffer));
            _serialD->write(initBuffer, 1);
            _serialD->flush();
            if (_txPin != VPIN_NONE) ArduinoPins::fastWriteDigital(_txPin, LOW);
          }
          unsigned long startMillis = millis();
          if (!_serialD->available()) {
            if (waitReceive == true && _waitCounter > _waitA) {
              flagOK = false;
            } else waitReceive = true;
          }
        uint16_t len = 0;
        unsigned long startMicros = micros();
        bool rxDone = false;
    byte tmpByte;
    len = _serialD->readBytesUntil(0xFE,_currentNode->_analogueInputBuffer, 25);
        if (!true/*crcGood(_currentNode->_digitalInputStates,sizeof(_currentNode->_digitalInputStates)-2)*/) {
          DIAG(F("EX-IOExpander485 CRC error on node %d"), _currentNode->getNodeID());
          flagOK = false;
        }
        if (!testAndStripMasterFlag(_currentNode->_digitalInputStates)) DIAG(F("Foreign RSproto Device! no master flag from node %d"),_currentNode->getNodeID());
        if (!waitReceive) _refreshOperation = 0;
        _lastAnalogueRead = currentMicros;
        _readState = RDS_ANALOGUE;
      }
      break;
      if(flagOK && !waitReceive) _currentNode = _currentNode->getNext();
  }
}
#if defined(RSproto_STM_OK)
  if (flagOK == true) {
    ArduinoPins::fastWriteDigital(RSproto_STM_OK,HIGH);
  } else {
    ArduinoPins::fastWriteDigital(RSproto_STM_OK,LOW);
  }
#endif
#if defined(RSproto_STM_FAIL)
  if (flagOK == false) {
    ArduinoPins::fastWriteDigital(RSproto_STM_FAIL,HIGH);
  } else {
    ArduinoPins::fastWriteDigital(RSproto_STM_FAIL,LOW);
  }
#endif
#if defined(RSproto_STM_COMM)
  ArduinoPins::fastWriteDigital(RSproto_STM_COMM,LOW);
#endif
  
}

// Link to chain of RSproto instances, left over from RSproto template.
RSproto *RSproto::_busList = NULL;


/************************************************************
 * RSprotonode implementation
 ************************************************************/

/* -= RSprotonode =-
//
// Constructor for RSprotonode object
*/
RSprotonode::RSprotonode(VPIN firstVpin, int nPins, uint8_t nodeID) {
  _firstVpin = firstVpin;
  _nPins = nPins;
  _busNo = 0;
  _nodeID = nodeID;
  bus = bus->findBus(0);
  _serial = bus->_serialD;
  if (_nodeID > 254) _nodeID = 254;

  // Add this device to HAL device list
  IODevice::addDevice(this);
  _display();
  // Add RSprotonode to RSproto object.
  RSproto *bus = RSproto::findBus(_busNo);
  if (bus != NULL) {
    bus->addNode(this);
    return;
  }
  
}

bool RSprotonode::_configure(VPIN vpin, ConfigTypeEnum configType, int paramCount, int params[]) {
    if (paramCount != 1) return false;
    int pin = vpin - _firstVpin;
    
    uint16_t pullup = params[0];
      uint8_t outBuffer[6] = {_nodeID, EXIODPUP, pin, pullup};
      uint8_t responseBuffer[3];
      bus->_busy = true;
      bus->updateCrc(outBuffer,4);
      if (bus->_txPin != VPIN_NONE) ArduinoPins::fastWriteDigital(bus->_txPin, HIGH);
      _serial->write(outBuffer, 6);
      _serial->write(initBuffer, 1);
      _serial->flush();
      if (bus->_txPin != VPIN_NONE) ArduinoPins::fastWriteDigital(bus->_txPin, LOW);
      unsigned long startMillis = millis();
      while (!_serial->available()) {
        if (millis() - startMillis > 500) return false;
      }
      uint16_t len = 0;
      unsigned long startMicros = micros();
      bool rxDone = false;
    byte tmpByte;
    len = _serial->readBytesUntil(0xFE,responseBuffer, 25);
      bus->_busy = false;
      if (true/*bus->crcGood(responseBuffer,sizeof(responseBuffer)-2)*/) {
        if (!testAndStripMasterFlag(responseBuffer)) DIAG(F("Foreign RSproto Device! no master flag from node %d"),_nodeID);
        if (responseBuffer[0] == EXIORDY) {
        } else {
          DIAG(F("EX-IOExpander485 Vpin %u cannot be used as a digital input pin"), pin);
        }
      } else {
        DIAG(F("EX-IOExpander485 node %d CRC Error"), _nodeID);
      }

  }

  int RSprotonode::_configureAnalogIn(VPIN vpin) {
    int pin = vpin - _firstVpin;
    //RSproto *mainrs = RSproto::findBus(_busNo);
    uint8_t commandBuffer[5] = {(uint8_t) _nodeID, EXIOENAN, (uint8_t) pin};
    uint8_t responseBuffer[3];
    bus->_busy = true;
    bus->updateCrc(commandBuffer,3);
    if (bus->_txPin != VPIN_NONE) ArduinoPins::fastWriteDigital(bus->_txPin, HIGH);
    _serial->write(commandBuffer, 5);
    _serial->write(initBuffer, 1);
    _serial->flush();
    if (bus->_txPin != VPIN_NONE) ArduinoPins::fastWriteDigital(bus->_txPin, LOW);
    unsigned long startMillis = millis();
    while (!_serial->available()) {
      if (millis() - startMillis > 500) return 0;
    }
    uint16_t len = 0;
    unsigned long startMicros = micros();
    bool rxDone = false;
    byte tmpByte;
    len = _serial->readBytesUntil(0xFE,responseBuffer, 25);
    bus->_busy = false;
    if (true/*bus->crcGood(responseBuffer,sizeof(responseBuffer)-2)*/) {
      if (!bus->testAndStripMasterFlag(responseBuffer)) DIAG(F("Foreign RSproto Device! no master flag from node %d"),_nodeID);
      if (responseBuffer[0] != EXIORDY) {
        DIAG(F("EX-IOExpander485: Vpin %u on node %d cannot be used as an analogue input pin"), (int) pin, (int) _nodeID);
      }
    } else {
        DIAG(F("EX-IOExpander485 node %d CRC Error"), (int) _nodeID);
    }
    return false;
  }

void RSprotonode::_begin() {
    //pinMode(bus->_txPin, OUTPUT);
    //ArduinoPins::fastWriteDigital(bus->_txPin, LOW);
    uint8_t receiveBuffer[5];
    
    uint8_t commandBuffer[7] = {_nodeID, EXIOINIT, (uint8_t)_nPins, (_firstVpin & (uint8_t)0xFF), (_firstVpin >> (uint8_t)8)};
    bus->updateCrc(commandBuffer,5);
    
    //_serial->begin(115200);
    //ArduinoPins::fastWriteDigital(bus->_txPin, HIGH);
    digitalWrite(bus->_txPin,HIGH);
    unsigned long startMillis = millis();
    
    _serial->write(commandBuffer, 7);
    _serial->write(initBuffer, 1);
    _serial->flush();
    digitalWrite(bus->_txPin,LOW);
    //ArduinoPins::fastWriteDigital(bus->_txPin, LOW);
    startMillis = millis();
    while (!_serial->available()) {
      if (millis() - startMillis >= 500) return;
    }
    uint16_t len = 0;
    unsigned long startMicros = micros();
    byte tmpByte;
    bool rxDone = false;
    len = _serial->readBytesUntil(0xFE,receiveBuffer, 25);
    DIAG(F("rxcode:%d from node"),receiveBuffer[1]);
    if (receiveBuffer[1] == EXIOPINS /*&& bus->crcGood(receiveBuffer,sizeof(receiveBuffer)-3)*/) {
      if (!bus->testAndStripMasterFlag(receiveBuffer)) DIAG(F("Foreign RSproto Device! no master flag from node %d"),_nodeID);
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
            //_deviceState = DEVSTATE_FAILED;
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
            //_deviceState = DEVSTATE_FAILED;
            _analoguePinBytes = 0;
            return;
          }
        }
      }
    } else {
      DIAG(F("EX-IOExpander485 node:%d ERROR configuring device (CRC: %s)"), _nodeID, bus->crcGood(receiveBuffer,sizeof(receiveBuffer)-2)? "PASS":"FAIL");
      //_deviceState = DEVSTATE_FAILED;
      return;
    }
    commandBuffer[0] = _nodeID;
    commandBuffer[1] = EXIOINITA;
    bus->updateCrc(commandBuffer,2);
    if (bus->_txPin != VPIN_NONE) ArduinoPins::fastWriteDigital(bus->_txPin, HIGH);
    _serial->write(commandBuffer, 4);
    _serial->write(initBuffer, 1);
    _serial->flush();
    if (bus->_txPin != VPIN_NONE) ArduinoPins::fastWriteDigital(bus->_txPin, LOW);
    startMillis = millis();
    while (!_serial->available()) {
      if (millis() - startMillis >= 500) return;
    }
    len = 0;
    startMicros = micros();
    rxDone = false;
    len = _serial->readBytesUntil(0xFE,receiveBuffer, 25);
    
    DIAG(F("rxcode:%d from node"),receiveBuffer[1]);
    if (true/*bus->crcGood(receiveBuffer,sizeof(receiveBuffer)-2)*/) {
      if (!bus->testAndStripMasterFlag(receiveBuffer)) DIAG(F("Foreign RSproto Device! no master flag from node %d"),_nodeID);
      for (int i = 0; i < _numAnaloguePins; i++) {
        _analoguePinMap[i] = receiveBuffer[i];
      }
    }
    uint8_t versionBuffer[5];
    commandBuffer[0] = _nodeID;
    commandBuffer[1] = EXIOVER;
    bus->updateCrc(commandBuffer,2);
    if (bus->_txPin != VPIN_NONE) ArduinoPins::fastWriteDigital(bus->_txPin, HIGH);
    _serial->write(commandBuffer, 4);
    _serial->write(initBuffer, 1);
    _serial->flush();
    if (bus->_txPin != VPIN_NONE) ArduinoPins::fastWriteDigital(bus->_txPin, LOW);
    startMillis = millis();
    while (!_serial->available()) {
      if (millis() - startMillis >= 500) return;
    }
    len = 0;
    startMicros = micros();
    rxDone = false;
    len = _serial->readBytesUntil(0xFE,receiveBuffer, 25);
    
    DIAG(F("rxcode:%d.%d.%d from node"),versionBuffer[1],versionBuffer[2],versionBuffer[3]);
    if (true/*bus->crcGood(versionBuffer,sizeof(versionBuffer)-2)*/) {
      if (!bus->testAndStripMasterFlag(versionBuffer)) DIAG(F("Foreign RSproto Device! no master flag from node %d"),_nodeID);
      _majorVer = versionBuffer[1];
      _minorVer = versionBuffer[2];
      _patchVer = versionBuffer[3];
      DIAG(F("EX-IOExpander485 device found, node:%d, Version v%d.%d.%d"), _nodeID, _majorVer, _minorVer, _patchVer);
    }
#ifdef DIAG_IO
    _display();
#endif
}

int RSprotonode::_read(VPIN vpin) {
    if (_deviceState == DEVSTATE_FAILED) return 0;
    int pin = vpin - _firstVpin;
    uint8_t pinByte = pin / 8;
    bool value = bitRead(_digitalInputStates[pinByte], pin - pinByte * 8);
    return value;
  }
void RSprotonode::_write(VPIN vpin, int value) {
    if (_deviceState == DEVSTATE_FAILED) return;
    int pin = vpin - _firstVpin;
    uint8_t digitalOutBuffer[6];
      uint8_t responseBuffer[3];
      digitalOutBuffer[0] = (uint8_t) _nodeID;
      digitalOutBuffer[1] = EXIOWRD;
      digitalOutBuffer[2] = (uint8_t) pin;
      digitalOutBuffer[3] = (uint8_t) value;
      bus->_busy = true;
      bus->updateCrc(digitalOutBuffer,4);
        if (bus->_txPin != VPIN_NONE) ArduinoPins::fastWriteDigital(bus->_txPin, HIGH);
        _serial->write(digitalOutBuffer, 6);
        _serial->write(initBuffer, 1);
        _serial->flush();
        if (bus->_txPin != VPIN_NONE) ArduinoPins::fastWriteDigital(bus->_txPin, LOW);
        unsigned long startMillis = millis();
        while (!_serial->available()) {
          if (millis() - startMillis >= 500) return;
        }
      uint16_t len = 0;
      unsigned long startMicros = micros();
      bool rxDone = false;
    byte tmpByte;
    len = _serial->readBytesUntil(0xFE,responseBuffer, 25);
      bus->_busy = false;
      if (true/*bus->crcGood(responseBuffer,sizeof(responseBuffer)-2)*/) {
        if (!testAndStripMasterFlag(responseBuffer)) DIAG(F("Foreign RSproto Device! no master flag from node %d"),_nodeID);
        if (responseBuffer[0] != EXIORDY) {
          DIAG(F("EX-IOExpander485 Vpin %u cannot be used as a digital output pin"), pin);
        }
      } else {
          DIAG(F("EX-IOExpander485 node %d CRC Error"), _nodeID);
        }
  }

   bool RSprotonode::testAndStripMasterFlag(uint8_t *buf) {
    if (buf[0] != 0xFF) return false; // why did we not get a master flag? bad node?
    for (int i = 0; i < sizeof(buf)-1; i++) buf[i] = buf[i+1]; // shift array to begining
    return true;
  }
  int RSprotonode::_readAnalogue(VPIN vpin) {
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

  void RSprotonode::_writeAnalogue(VPIN vpin, int value, uint8_t profile, uint16_t duration) {
    uint8_t servoBuffer[7];
    uint8_t responseBuffer[1];

    if (_deviceState == DEVSTATE_FAILED) return;
    int pin = vpin - _firstVpin;
    servoBuffer[0] = (uint8_t) _nodeID;
      servoBuffer[1] = EXIOWRAN;
      servoBuffer[2] = (uint8_t) pin;
      servoBuffer[3] = (uint8_t) value & 0xFF;
      servoBuffer[4] = (uint8_t) value >> 8;
      servoBuffer[5] = (uint8_t) profile;
      servoBuffer[6] = (uint8_t) duration & 0xFF;
      servoBuffer[7] = (uint8_t) duration >> 8;
      bus->_busy = true;
      bus->updateCrc(servoBuffer,8);
        if (bus->_txPin != VPIN_NONE) ArduinoPins::fastWriteDigital(bus->_txPin, HIGH);
        _serial->write(servoBuffer, 10);
        _serial->write(initBuffer, 1);
        _serial->flush();
        if (bus->_txPin != VPIN_NONE) ArduinoPins::fastWriteDigital(bus->_txPin, LOW);
        unsigned long startMillis = millis();
        while (!_serial->available()) {
          if (millis() - startMillis >= 500) return;
        }
      uint16_t len = 0;
      unsigned long startMicros = micros();
      bool rxDone = false;
    byte tmpByte;
    len = _serial->readBytesUntil(0xFE,responseBuffer, 25);
      bus->_busy = false;
      if (!true/*bus->crcGood(responseBuffer,sizeof(responseBuffer)-2)*/) {
        DIAG(F("EX-IOExpander485 node %d CRC Error"), (int) _nodeID);
        //_deviceState = DEVSTATE_FAILED;
      } else {
        if (!bus->testAndStripMasterFlag(responseBuffer)) DIAG(F("Foreign RSproto Device! no master flag from node %d"),_nodeID);
        if (responseBuffer[0] != EXIORDY) {
          DIAG(F("EX-IOExpander485 Vpin %u cannot be used as a servo/PWM pin"), pin);
        }
      }
  }