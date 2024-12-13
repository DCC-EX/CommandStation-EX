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

#include "IO_RS485.h"
#include "defines.h"

/************************************************************
 * RS485 implementation
 ************************************************************/

// Constructor for RS485
RS485::RS485(HardwareSerial &serial, unsigned long baud, uint16_t cycleTimeMS, int8_t txPin, int waitA) {
  _baud = baud;
  _serialD = &serial;
  _txPin = txPin;
  _busNo = 0;

  _cycleTime = cycleTimeMS * 1000UL; // convert from milliseconds to microseconds.
  _waitA = waitA;
  if (_waitA < 3) _waitA = 3;
  // Add device to HAL device chain
  IODevice::addDevice(this);
  
  // Add bus to RS485 chain.
  _nextBus = _busList;
  _busList = this;
}

/* -= updateCrc =-
//
// add the CRC value from _calculateCrc (2 bytes) to the buffer.
*/
void RS485::updateCrc(uint8_t *buf, uint16_t len) {
  uint16_t crc = _calculateCrc(buf, len);
  buf[len] = lowByte(crc);
  buf[len + 1] = highByte(crc);
}

/* -= crcGood =-
//
// return TRUE if CRC matched between buffer copy, and calculated.
*/
bool RS485::crcGood(uint8_t *buf, uint16_t len) {
  uint16_t aduCrc = buf[len] | (buf[len + 1] << 8);
  uint16_t calculatedCrc = _calculateCrc(buf, len);
  if (aduCrc == calculatedCrc) return true;
  else return false;
}

/* -= calculateCrc =-
//
// use bitwise XOR to calculate CRC into a 16-bit byte
*/
uint16_t RS485::_calculateCrc(uint8_t *buf, uint16_t len) {
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
void RS485::clearRxBuffer() {
  unsigned long startMicros = micros();
  do {
    if (_serialD->available() > 0) {
      startMicros = micros();
      _serialD->read();
    }
  } while (micros() - startMicros < _frameTimeout);
}

/* -= _loop =-
//
// Main loop function for RS485.
// Work through list of nodes.  For each node, in separate loop entries
// When the slot time has finished, move on to the next device.
*/
void RS485::_loop(unsigned long currentMicros) {
  _currentMicros = currentMicros;
  
  if (_currentNode == NULL) {
    _currentNode = _nodeListStart;
    
  }

  if (_currentMicros - _cycleStartTime < _cycleTime) return;
  _cycleStartTime = _currentMicros;
  if (_currentNode == NULL) return;

  bool flagOK = true;
#if defined(RS485_STM_COMM)
  ArduinoPins::fastWriteDigital(RS485_STM_COMM,HIGH);
#endif

if (taskCnt > 0) {
  // run through tasks
  if (!waitReceive) getNextTask(taskData);
  switch((int) taskData[0]) {
    case 0:
      // protection for pulling empty task
      break;
    case 1: // configure pin
      if (taskData[4] == (int*) CONFIGURE_INPUT) {
        uint8_t pullup = (uint8_t) taskData[6];
        uint8_t outBuffer[6] = {EXIODPUP, (uint8_t) taskData[0], (uint8_t)taskData[3], pullup};
        uint8_t responseBuffer[3];
        updateCrc(outBuffer,4);
        if (waitReceive == false) {
          if (_txPin != VPIN_NONE) ArduinoPins::fastWriteDigital(_txPin, HIGH);
          _serialD->write(outBuffer, 6);
          _serialD->flush();
          if (_txPin != VPIN_NONE) ArduinoPins::fastWriteDigital(_txPin, LOW);
        }
        unsigned long startMillis = millis();
        if (!_serialD->available()) {
          if (waitReceive == true && _waitCounter > _waitA) {
            flagOK = false;
          } else waitReceive = true;
        }
        waitReceive = false;
        uint16_t len = 0;
        unsigned long startMicros = micros();
        do {
          if (_serialD->available()) {
            startMicros = micros();
            responseBuffer[len] = _serialD->read();
            len++;
          }
        } while (micros() - startMicros <= 500 && len < 256);
        if (crcGood(responseBuffer,sizeof(responseBuffer)-2)) {
          if (!testAndStripMasterFlag(responseBuffer)) DIAG(F("Foreign RS485 Device! no master flag from node %d"),_currentNode->getNodeID());
          if (responseBuffer[0] == EXIORDY) {
          } else {
            DIAG(F("EX-IOExpander485 Vpin %u cannot be used as a digital input pin"), (int)taskData[3]);
          }
        } else {
          DIAG(F("EX-IOExpander485 node %d CRC Error"), (int) taskData[0]);
          flagOK = false;
        }
      } else if (taskData[3] == (int*) CONFIGURE_ANALOGINPUT) {
        // TODO:  Consider moving code from _configureAnalogIn() to here and remove _configureAnalogIn
        // from IODevice class definition.  Not urgent, but each virtual function defined
        // means increasing the RAM requirement of every HAL device driver, whether it's relevant
        // to the driver or not.
      }
      break;
    case 2: // configure analog in
      uint8_t commandBuffer[5] = {EXIOENAN, (uint8_t) taskData[0], (uint8_t) taskData[3]};
      uint8_t responseBuffer[3];
      updateCrc(commandBuffer,3);
      if (waitReceive == false) {
          if (_txPin != VPIN_NONE) ArduinoPins::fastWriteDigital(_txPin, HIGH);
          _serialD->write(commandBuffer, 5);
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
      do {
        if (_serialD->available()) {
          startMicros = micros();
          responseBuffer[len] = _serialD->read();
          len++;
        }
      } while (micros() - startMicros <= 500 && len < 256);

      if (crcGood(responseBuffer,sizeof(responseBuffer)-2)) {
        if (!testAndStripMasterFlag(responseBuffer)) DIAG(F("Foreign RS485 Device! no master flag from node %d"),_currentNode->getNodeID());
        if (responseBuffer[0] != EXIORDY) {
          DIAG(F("EX-IOExpander485: Vpin %u on node %d cannot be used as an analogue input pin"), (int) taskData[3], (int) taskData[0]);
        }
      } else {
          DIAG(F("EX-IOExpander485 node %d CRC Error"), (int) taskData[0]);
          flagOK = false;
        }
      break;
    case 3: // write pin
      uint8_t digitalOutBuffer[6];
      uint8_t responseBuffer[3];
      digitalOutBuffer[0] = EXIOWRD;
      digitalOutBuffer[1] = (uint8_t) taskData[0];
      digitalOutBuffer[2] = (uint8_t) taskData[3];
      digitalOutBuffer[3] = (uint8_t) taskData[4];
      updateCrc(digitalOutBuffer,4);
      if (waitReceive == false) {
          if (_txPin != VPIN_NONE) ArduinoPins::fastWriteDigital(_txPin, HIGH);
          _serialD->write(digitalOutBuffer, 6);
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
      do {
        if (_serialD->available()) {
          startMicros = micros();
          responseBuffer[len] = _serialD->read();
          len++;
        }
      } while (micros() - startMicros <= 500 && len < 256);
      if (crcGood(responseBuffer,sizeof(responseBuffer)-2)) {
        if (!testAndStripMasterFlag(responseBuffer)) DIAG(F("Foreign RS485 Device! no master flag from node %d"),_currentNode->getNodeID());
        if (responseBuffer[0] != EXIORDY) {
          DIAG(F("EX-IOExpander485 Vpin %u cannot be used as a digital output pin"), (int)taskData[3]);
        }
      } else {
          DIAG(F("EX-IOExpander485 node %d CRC Error"), (int) taskData[0]);
          flagOK = false;
        }
      break;
    case 4:
      uint8_t servoBuffer[10];
      uint8_t responseBuffer[3];
#ifdef DIAG_IO
      DIAG(F("EX-IOExpander485 Servo: WriteAnalogue Vpin:%u Value:%d Profile:%d Duration:%d %S"), 
        vpin, value, profile, duration, _deviceState == DEVSTATE_FAILED?F("DEVSTATE_FAILED"):F(""));
#endif
      servoBuffer[0] = EXIOWRAN;
      servoBuffer[1] = (uint8_t) taskData[0];
      servoBuffer[2] = (uint8_t) taskData[3];
      servoBuffer[3] = (uint8_t) taskData[4] & 0xFF;
      servoBuffer[4] = (uint8_t) taskData[4] >> 8;
      servoBuffer[5] = (uint8_t) taskData[5];
      servoBuffer[6] = (uint8_t) taskData[6] & 0xFF;
      servoBuffer[7] = (uint8_t) taskData[6] >> 8;
      updateCrc(servoBuffer,8);
      if (waitReceive == false) {
          if (_txPin != VPIN_NONE) ArduinoPins::fastWriteDigital(_txPin, HIGH);
          _serialD->write(servoBuffer, 10);
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
      do {
        if (_serialD->available()) {
          startMicros = micros();
          responseBuffer[len] = _serialD->read();
          len++;
        }
      } while (micros() - startMicros <= 500 && len < 256);
      if (!crcGood(responseBuffer,sizeof(responseBuffer)-2)) {
        DIAG(F("EX-IOExpander485 node %d CRC Error"), (int) taskData[0]);
        flagOK = false;
        //_deviceState = DEVSTATE_FAILED;
      } else {
        if (!testAndStripMasterFlag(responseBuffer)) DIAG(F("Foreign RS485 Device! no master flag from node %d"),_currentNode->getNodeID());
        if (responseBuffer[0] != EXIORDY) {
          DIAG(F("EX-IOExpander485 Vpin %u cannot be used as a servo/PWM pin"), (int) taskData[3]);
        }
      }
  }
} else {
  memcpy(_currentNode->_analogueInputStates, _currentNode->_analogueInputBuffer, _currentNode->_analoguePinBytes); // Copy I2C input buffer to states
  switch (_refreshOperation) {
    case 0:
      if (_currentNode->_numDigitalPins>0 && currentMicros - _lastDigitalRead > _digitalRefresh) { // Delay for digital read refresh
        // Issue new read request for digital states.  As the request is non-blocking, the buffer has to
        // be allocated from heap (object state).
        _currentNode->_readCommandBuffer[0] = EXIORDD;
        _currentNode->_readCommandBuffer[1] = _currentNode->getNodeID();
        updateCrc(_currentNode->_readCommandBuffer,sizeof(_currentNode->_readCommandBuffer)-2);
        if (waitReceive == false) {
            if (_txPin != VPIN_NONE) ArduinoPins::fastWriteDigital(_txPin, HIGH);
            _serialD->write(_currentNode->_readCommandBuffer, sizeof(_currentNode->_readCommandBuffer));
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
        do {
          if (_serialD->available()) {
            startMicros = micros();
            _currentNode->_digitalInputStates[len] = _serialD->read();
            len++;
          }
        } while (micros() - startMicros <= 500 && len < (_currentNode->_numDigitalPins+7)/8);
        if (!crcGood(_currentNode->_digitalInputStates,sizeof(_currentNode->_digitalInputStates)-2)) {
          DIAG(F("EX-IOExpander485 CRC error on node %d"), _currentNode->getNodeID());
          flagOK = false;
        }
        if (!testAndStripMasterFlag(_currentNode->_digitalInputStates)) DIAG(F("Foreign RS485 Device! no master flag from node %d"),_currentNode->getNodeID());
        if (!waitReceive) _refreshOperation++;
        _lastDigitalRead = currentMicros;
        _readState = RDS_DIGITAL;
      }
      break;
    case 1:
      if (_currentNode->_numAnaloguePins>0 && currentMicros - _lastAnalogueRead > _analogueRefresh) { // Delay for analogue read refresh
        // Issue new read for analogue input states
        _currentNode->_readCommandBuffer[0] = EXIORDAN;
        _currentNode->_readCommandBuffer[1] = _currentNode->getNodeID();
        updateCrc(_currentNode->_readCommandBuffer,sizeof(_currentNode->_readCommandBuffer)-2);
        if (waitReceive == false) {
            if (_txPin != VPIN_NONE) ArduinoPins::fastWriteDigital(_txPin, HIGH);
            _serialD->write(_currentNode->_readCommandBuffer, sizeof(_currentNode->_readCommandBuffer));
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
        do {
          if (_serialD->available()) {
            startMicros = micros();
            _currentNode->_analogueInputBuffer[len] = _serialD->read();
            len++;
          }
        } while (micros() - startMicros <= 500 && len < _currentNode->_numAnaloguePins * 2);
        if (!crcGood(_currentNode->_digitalInputStates,sizeof(_currentNode->_digitalInputStates)-2)) {
          DIAG(F("EX-IOExpander485 CRC error on node %d"), _currentNode->getNodeID());
          flagOK = false;
        }
        if (!testAndStripMasterFlag(_currentNode->_digitalInputStates)) DIAG(F("Foreign RS485 Device! no master flag from node %d"),_currentNode->getNodeID());
        if (!waitReceive) _refreshOperation = 0;
        _lastAnalogueRead = currentMicros;
        _readState = RDS_ANALOGUE;
      }
      break;
  }
  if(flagOK && !waitReceive) _currentNode = _currentNode->getNext();
}

#if defined(RS485_STM_OK)
  if (flagOK == true) {
    ArduinoPins::fastWriteDigital(RS485_STM_OK,HIGH);
  } else {
    ArduinoPins::fastWriteDigital(RS485_STM_OK,LOW);
  }
#endif
#if defined(RS485_STM_FAIL)
  if (flagOK == false) {
    ArduinoPins::fastWriteDigital(RS485_STM_FAIL,HIGH);
  } else {
    ArduinoPins::fastWriteDigital(RS485_STM_FAIL,LOW);
  }
#endif
#if defined(RS485_STM_COMM)
  ArduinoPins::fastWriteDigital(RS485_STM_COMM,LOW);
#endif
  
}

// Link to chain of RS485 instances, left over from RS485 template.
RS485 *RS485::_busList = NULL;


/************************************************************
 * RS485node implementation
 ************************************************************/

/* -= RS485node =-
//
// Constructor for RS485node object
*/
RS485node::RS485node(VPIN firstVpin, int nPins, uint8_t nodeID) {
  _firstVpin = firstVpin;
  _nPins = nPins;
  _busNo = 0;
  _nodeID = nodeID;
  if (_nodeID > 254) _nodeID = 254;

  // Add this device to HAL device list
  IODevice::addDevice(this);
  _display();
  // Add RS485node to RS485 object.
  RS485 *bus = RS485::findBus(_busNo);
  if (bus != NULL) {
    bus->addNode(this);
    return;
  }
  
}
