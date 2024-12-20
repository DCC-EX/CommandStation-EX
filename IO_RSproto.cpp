/*
 *  © 2024, Travis Farmer. All rights reserved.
 *  © 2021 Chris Harlow
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

static const byte PAYLOAD_FALSE = 0;
static const byte PAYLOAD_NORMAL = 1;
static const byte PAYLOAD_STRING = 2;

taskBuffer * taskBuffer::first=NULL;

taskBuffer::taskBuffer(unsigned long taskID, uint8_t *commandBuffer, int byteCount)
{
  _taskID = taskID;
  _byteCount = byteCount;
  memset(commandArray, 0, byteCount);
  memcpy(commandArray, commandBuffer, byteCount);

  next=first;
  first=this;
  
  RSproto *bus = RSproto::findBus(0);
  if (bus != NULL) {
    bus->addTask(this);
    return;
  }
}

taskBuffer::~taskBuffer()
{
  // destructor
}

void RSproto::remove_nulls(char *str, int len) {
    int i, j = 0;
    for (i = 0; i<len; i++) {
        if (str[i] != '\0') {
            str[j++] = str[i];
        }
    }
    str[j] = '\0'; // Ensure the string is null-terminated
}

int RSproto::getCharsLeft(char *str, char position) {
  int pos;
  char result[25];
  memset(result, '\0', 25);
  for (int i = 0; str[i] != '\0'; i++) {
    if (str[i] == position) {
      pos = i;
      break; // Exit the loop once the character is found
    }
  }
  if (pos >= 0 && pos < strlen(str)) {
    for (int i = 0; i < strlen(str); i++) {
      if (i < pos) result[i] = str[i];
    }
  }
  if (result != NULL) return atoi(result);
  else return 0;
}

void taskBuffer::doCommand(unsigned long taskID, uint8_t *commandBuffer, int byteCount) {
  // add commands here to be sent
  new taskBuffer(taskID, commandBuffer, byteCount);
  
}

void RSproto::parseRx(uint8_t * outArray) {
  int nodeFr = outArray[1];
  int AddrCode = outArray[2];
  DIAG(F("From: %i, To: %i"), nodeFr,outArray[0]);
  RSprotonode *node = findNode(nodeFr);
  switch (AddrCode) {
    case EXIOPINS:
      {node->_numDigitalPins = outArray[3];
      node->_numAnaloguePins = outArray[4];

      // See if we already have suitable buffers assigned
      if (node->_numDigitalPins>0) {
        size_t digitalBytesNeeded = (node->_numDigitalPins + 7) / 8;
        if (node->_digitalPinBytes < digitalBytesNeeded) {
          // Not enough space, free any existing buffer and allocate a new one
          if (node->_digitalPinBytes > 0) free(node->_digitalInputStates);
          if ((node->_digitalInputStates = (byte*) calloc(digitalBytesNeeded, 1)) != NULL) {
            node->_digitalPinBytes = digitalBytesNeeded;
          } else {
            DIAG(F("EX-IOExpander485 node:%d ERROR alloc %d bytes"), nodeFr, digitalBytesNeeded);
            //_deviceState = DEVSTATE_FAILED;
            node->_digitalPinBytes = 0;
            return;
          }
        }
      }
      
      if (node->_numAnaloguePins>0) {
        size_t analogueBytesNeeded = node->_numAnaloguePins * 2;
        if (node->_analoguePinBytes < analogueBytesNeeded) {
          // Free any existing buffers and allocate new ones.
          if (node->_analoguePinBytes > 0) {
            free(node->_analogueInputBuffer);
            free(node->_analogueInputStates);
            free(node->_analoguePinMap);
          }
          node->_analogueInputStates = (uint8_t*) calloc(analogueBytesNeeded, 1);
          node->_analogueInputBuffer = (uint8_t*) calloc(analogueBytesNeeded, 1);
          node->_analoguePinMap = (uint8_t*) calloc(node->_numAnaloguePins, 1);
          if (node->_analogueInputStates  != NULL &&
            node->_analogueInputBuffer != NULL &&
            node->_analoguePinMap != NULL) {
            node->_analoguePinBytes = analogueBytesNeeded;
          } else {
            DIAG(F("EX-IOExpander485 node:%d ERROR alloc analog pin bytes"), nodeFr);
            //_deviceState = DEVSTATE_FAILED;
            node->_analoguePinBytes = 0;
            return;
          }
        }
      }
      node->resFlag = 1;
      break;}
    case EXIOINITA: {
      for (int i = 0; i < node->_numAnaloguePins; i++) {
        node->_analoguePinMap[i] = outArray[i+3];
      }
      node->resFlag = 1;
      break;
    }
    case EXIOVER: {
      node->_majorVer = outArray[3];
      node->_minorVer = outArray[4];
      node->_patchVer = outArray[5];
      node->resFlag = 1;
      break;
    }
    case EXIORDY: {
      node->resFlag = 1;
      break;
    }
    case EXIOERR: {
      node->resFlag = -1;
      break;
    }
    case EXIORDD: {
      for (int i = 0; i < (node->_numDigitalPins+7)/8; i++) {
        node->_digitalInputStates[i] = outArray[i+3];
      }
      node->resFlag = 1;
      break;
    }
    case EXIORDAN: {
      for (int i = 0; i < node->_numAnaloguePins; i++) {
        node->_analogueInputBuffer[i] = outArray[i+3];
      }
      node->resFlag = 1;
      break;
    }
  }
}

/************************************************************
 * RSproto implementation
 ************************************************************/

// Constructor for RSproto
RSproto::RSproto(uint8_t busNo, HardwareSerial &serial, unsigned long baud, int8_t txPin, int cycleTime) {
  _serial = &serial;
  _baud = baud;
  
  _txPin = txPin;
  _busNo = busNo;
  _cycleTime = cycleTime;
  bufferLength=0;
  inCommandPayload=PAYLOAD_FALSE;
  // Add device to HAL device chain
  IODevice::addDevice(this);
  
  // Add bus to RSproto chain.
  _nextBus = _busList;
  _busList = this;
}

/* -= _loop =-
//
// Main loop function for RSproto.
// Work through list of nodes.  For each node, in separate loop entries
// When the slot time has finished, move on to the next device.
*/

// CRC-16 implementation (replace with your preferred CRC library if needed)
uint16_t RSproto::crc16(uint8_t *data, uint16_t length) {
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++) {
      bool bit = ((crc & 0x0001) != 0);
      crc >>= 1;
      if (bit) {
        crc ^= 0xA001;
      }
    }
  }
  return crc;
}

void RSproto::sendInstantCommand(uint8_t *buf, int byteCount) {
  // Calculate CRC for response data
  uint16_t response_crc = crc16((uint8_t*)buf, byteCount-1);
  if (_txPin != -1) digitalWrite(_txPin,HIGH);
  // Send response data with CRC
  _serial->write(0xFE);
  _serial->write(0xFE);
  _serial->write(response_crc >> 8);
  _serial->write(response_crc & 0xFF);
  _serial->write(byteCount);
  for (int i = 0; i < byteCount; i++) {
    _serial->write(buf[i]);

  }
  _serial->write(0xFD);
  _serial->write(0xFD);
  _serial->flush();
  if (_txPin != -1) digitalWrite(_txPin,LOW);
  // delete task command after sending, for now

  //int received_data[ARRAY_SIZE];
  uint16_t received_crc;
  while(_serial->available()) {
    if (_serial->available()) {
      uint8_t received_data[ARRAY_SIZE];
    
      uint16_t calculated_crc;
      int byteCount = 100;
      
      int curByte = _serial->read();
      
      if (curByte == 0xFE && flagStart == false) flagStart = true;
      else if ( curByte == 0xFE && flagStart == true) {
        byteCounter = 0;
        flagStarted = true;
        flagStart = false;
        flagEnded = false;
        rxStart = true;
        rxEnd = false;
        crcPass = false;
      }else if (flagStarted) {
        crc[0] = curByte;
        byteCounter++;
        flagStarted = false;
      } else if (byteCounter == 1) {
        crc[1] = curByte;
        received_crc  = (crc[0] << 8) | crc[1];
        byteCounter++;
      } else if (byteCounter == 2) {
        byteCount = curByte;
        byteCounter++;
      } else if (flagEnded == false && byteCounter >= 3) {
        received_data[byteCounter-3] = curByte;
        byteCounter++;
      }
      if (curByte == 0xFD && flagEnd == false) flagEnd = true;
      else if ( curByte == 0xFD && flagEnd == true) {
        flagEnded = true;
        flagEnd = false;
        rxEnd = true;
        byteCount = byteCounter;
        byteCounter = 0;
      }
      if (flagEnded) {
        calculated_crc = crc16((uint8_t*)received_data, byteCount-6);
        if (received_crc == calculated_crc) {
          crcPass = true;
          DIAG(F("CRC PASS"));
        }
        flagEnded = false;
      }
      // Check CRC validity
      if (crcPass) {
        // Data received successfully, process it (e.g., print)
        int nodeTo = (received_data[1] << 8) | received_data[0];
        if (nodeTo == 0) { // for master. master does not retransmit, or a loop will runaway.
          parseRx(received_data);
        }
      } else {
        //DIAG(F("IO_RSproto: CRC Error!"));
      }
    }
  }
}

void RSproto::_loop(unsigned long currentMicros) {
  _currentMicros = currentMicros;
  if (_busy == true) return;
  if (_currentTask == NULL) {
    _currentTask = _taskListStart;
    
  }

  if (_currentMicros - _cycleStartTime < _cycleTime) return;
  _cycleStartTime = _currentMicros;
  if (_currentTask != NULL && _currentTask->_byteCount > 0) {
    
    // Calculate CRC for response data
    
    uint16_t response_crc = crc16((uint8_t*)_currentTask->commandArray, _currentTask->_byteCount-6);
    if (_txPin != -1) digitalWrite(_txPin,HIGH);
    // Send response data with CRC
    _serial->write(0xFE);
    _serial->write(0xFE);
    _serial->write(response_crc >> 8);
    _serial->write(response_crc & 0xFF);
    _serial->write(_currentTask->_byteCount);
    for (int i = 0; i < _currentTask->_byteCount; i++) {
      _serial->write(_currentTask->commandArray[i]);
    }
    _serial->write(0xFD);
    _serial->write(0xFD);
    _serial->flush();
    if (_txPin != -1) digitalWrite(_txPin,LOW);
    // delete task command after sending, for now
    memset(_currentTask->commandArray, 0, _currentTask->_byteCount);
    _currentTask->_byteCount = 0;
  }

  if (_serial->available()) {
    uint8_t received_data[ARRAY_SIZE];
    
    uint16_t calculated_crc;
    int byteCount = 100;
    
    uint8_t byte_array[byteCount];
    int curByte = _serial->read();
    
    if (curByte == 0xFE && flagStart == false) flagStart = true;
    else if ( curByte == 0xFE && flagStart == true) {
      byteCounter = 0;
      flagStarted = true;
      flagStart = false;
      flagEnded = false;
      rxStart = true;
      rxEnd = false;
      crcPass = false;
    }else if (flagStarted) {
      crc[0] = curByte;
      byteCounter++;
      flagStarted = false;
    } else if (byteCounter == 1) {
      crc[1] = curByte;
      received_crc  = (crc[0] << 8) | crc[1];
      byteCounter++;
    } else if (byteCounter == 2) {
      byteCount = curByte;
      byteCounter++;
    } else if (flagEnded == false && byteCounter >= 3) {
      received_data[byteCounter-3] = curByte;
      byteCounter++;
    }
    if (curByte == 0xFD && flagEnd == false) flagEnd = true;
    else if ( curByte == 0xFD && flagEnd == true) {
      flagEnded = true;
      flagEnd = false;
      rxEnd = true;
      byteCount = byteCounter;
      byteCounter = 0;
    }
    if (flagEnded) {
      calculated_crc = crc16((uint8_t*)received_data, byteCount-6);
      if (received_crc == calculated_crc) {
        DIAG(F("CRC PASS"));
        crcPass = true;
      }else DIAG(F("CRC Fail %x %x"),received_crc,calculated_crc);
      flagEnded = false;
    }
    // Check CRC validity
    if (crcPass) {
      // Data received successfully, process it (e.g., print)
      int nodeTo = received_data[0];
      if (nodeTo == 0) { // for master. master does not retransmit, or a loop will runaway.
        parseRx(received_data);
      }
    } else {
      //DIAG(F("IO_RSproto: CRC Error!"));
    }
  }

  task->getNext();
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
  //bus = bus->findBus(0);
  //_serial = bus->_serialD;
  if (_nodeID > 252) _nodeID = 252; // cannot have a node with the frame flags
  if (_nodeID < 1) _nodeID = 1; // cannot have a node with the master ID

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
    
    uint8_t pullup = (uint8_t)params[0];
    uint8_t buff[ARRAY_SIZE];
    buff[0] = (_nodeID);
    buff[1] = (0);
    buff[2] = (EXIODPUP);
    buff[3] = (pin);
    buff[4] = (pullup);
    unsigned long startMillis = millis();
    RSproto *bus = RSproto::findBus(0);
    bus->_busy = true;
    bus->sendInstantCommand(buff, 5);
    bus->_busy = false;
    while (resFlag == 0 && millis() - startMillis < 500); // blocking for now
    if (resFlag != 1) {
      DIAG(F("EX-IOExpander485 Vpin %u cannot be used as a digital input pin"), pin);
      return false;
    }
    resFlag = 0;
    return true;
  }

  int RSprotonode::_configureAnalogIn(VPIN vpin) {
    int pin = vpin - _firstVpin;
    //RSproto *mainrs = RSproto::findBus(_busNo);
    uint8_t buff[ARRAY_SIZE];
    buff[0] = (_nodeID);
    buff[1] = (0);
    buff[2] = (EXIOENAN);
    buff[3] = (pin);
    buff[4] = highByte(_firstVpin);
    buff[5] = lowByte(_firstVpin);
    unsigned long startMillis = millis();
    RSproto *bus = RSproto::findBus(0);
    bus->_busy = true;
    bus->sendInstantCommand(buff, 6);
    bus->_busy = false;
    while (resFlag == 0 && millis() - startMillis < 500); // blocking for now
    if (resFlag != 1) {
      DIAG(F("EX-IOExpander485 Vpin %u cannot be used as a digital input pin"), pin);
      return false;
    }
    resFlag = 0;
    return true;
  }

void RSprotonode::_begin() {
  uint8_t buff[ARRAY_SIZE];
  buff[0] = (_nodeID);
  buff[1] = (0);
  buff[2] = (EXIOINIT);
  buff[3] = (_nPins);
  buff[4] = ((_firstVpin & 0xFF));
  buff[5] = ((_firstVpin >> 8));
  unsigned long startMillis = millis();
  RSproto *bus = RSproto::findBus(0);
  bus->_busy = true;
  bus->sendInstantCommand(buff, 6);
  bus->_busy = false;
  while (resFlag == 0 && millis() - startMillis < 1000); // blocking for now
  if (resFlag != 1) {
    DIAG(F("EX-IOExpander485 Node:%d ERROR EXIOINIT"), _nodeID);
  }
  resFlag = 0;
  buff[0] = (_nodeID);
  buff[1] = (0);
  buff[2] = (EXIOINITA);
  startMillis = millis();
  bus->_busy = true;
      bus->sendInstantCommand(buff,3);
      bus->_busy = false;
  while (resFlag == 0 && millis() - startMillis < 1000); // blocking for now
  if (resFlag != 1) {
    DIAG(F("EX-IOExpander485 Node:%d ERROR EXIOINITA"), _nodeID);
  }
  resFlag = 0;
  buff[0] = (_nodeID);
  buff[1] = (0);
  buff[2] = (EXIOVER);
  startMillis = millis();
   bus->_busy = true;
      bus->sendInstantCommand(buff,3);
      bus->_busy = false;
  while (resFlag == 0 && millis() - startMillis < 1000); // blocking for now
  if (resFlag != 1) {
    DIAG(F("EX-IOExpander485 Node:%d ERROR EXIOVER"), _nodeID);
  } else DIAG(F("EX-IOExpander device found, Node:%d, Version v%d.%d.%d"), _nodeID, _majorVer, _minorVer, _patchVer);
  resFlag = 0;


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
    uint8_t buff[ARRAY_SIZE];
    buff[0] = (_nodeID);
    buff[1] = (0);
    buff[2] = (EXIOWRD);
    buff[3] = (pin);
    buff[4] = (value);
    unsigned long startMillis = millis();
    RSproto *bus = RSproto::findBus(0);
      task->doCommand(bus->taskCounter++, buff,5);
    while (resFlag == 0 && millis() - startMillis < 500); // blocking for now
    if (resFlag != 1) {
      DIAG(F("EX-IOExpander485 Node:%d ERROR EXIOVER"), _nodeID);
    }
    resFlag = 0;
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
    int pin = vpin - _firstVpin;
    uint8_t buff[ARRAY_SIZE];
    buff[0] = (_nodeID);
    buff[1] = (0);
    buff[2] = (EXIOWRAN);
    buff[3] = (pin);
    buff[4] = highByte(value);
    buff[5] = lowByte(value);
    buff[6] = (profile);
    buff[7] = highByte(duration);
    buff[8] = lowByte(duration);
    unsigned long startMillis = millis();
    RSproto *bus = RSproto::findBus(0);
      task->doCommand(bus->taskCounter++, buff,9);
    while (resFlag == 0 && millis() - startMillis < 500); // blocking for now
    if (resFlag != 1) {
      DIAG(F("EX-IOExpander485 Node:%d ERROR EXIOVER"), _nodeID);
    }
    resFlag = 0;
  }