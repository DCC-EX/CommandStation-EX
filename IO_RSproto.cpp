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


/************************************************************
 * RSproto implementation
 ************************************************************/

// Constructor for RSproto
RSproto::RSproto(uint8_t busNo, HardwareSerial &serial, unsigned long baud, int8_t txPin, int cycleTime) {
  _serial = &serial;
  _baud = baud;
  
  _txPin = txPin;
  _busNo = busNo;
  _cycleTime = cycleTime * 1000UL;
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

void RSproto::_loop(unsigned long currentMicros) {
  _currentMicros = currentMicros;
  //if (_busy == true) return;
 
  
  if ( hasTasks()){
    Task* currentTask = getTaskById(getNextTaskId());
  if (currentTask == nullptr) return;
    if (!currentTask->rxMode) { // Check if a task was found
      currentTask->crcPassFail = 0;
      uint16_t response_crc = crc16((uint8_t*)currentTask->commandArray, currentTask->byteCount-1);
      if (_txPin != -1) digitalWrite(_txPin,HIGH);
      // Send response data with CRC
      _serial->write(0xFE);
      _serial->write(0xFE);
      _serial->write(response_crc >> 8);
      _serial->write(response_crc & 0xFF);
      _serial->write(currentTask->byteCount);
      for (int i = 0; i < currentTask->byteCount; i++) {
        _serial->write(currentTask->commandArray[i]);
      }
      _serial->write(0xFD);
      _serial->write(0xFD);
      _serial->flush();
      if (_txPin != -1) digitalWrite(_txPin,LOW);
      // delete task command after sending, for now
      currentTask->rxMode = true;
      
    } else if (currentTask->rxMode) {
      if (_serial->available() && currentTask->rxMode) {
        
        
        uint16_t calculated_crc;
        int byteCount = 100;
        
        uint8_t byte_array[byteCount];
        int curByte = _serial->read();
        
        if (curByte == 0xFE && flagStart == false) flagStart = true;
        else if ( curByte == 0xFE && flagStart == true) {
          flagProc = false;
          byteCounter = 0;
          flagStarted = true;
          flagStart = false;
          flagEnded = false;
          rxStart = true;
          rxEnd = false;
          crcPass = false;
          memset(received_data, 0, ARRAY_SIZE);
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
            //DIAG(F("Loop CRC PASS"));
            crcPass = true;
            currentTask->crcPassFail = 1;
          }else {
            //DIAG(F("Loop CRC Fail %x %x"),received_crc,calculated_crc);
            currentTask->processed = true;

            currentTask->crcPassFail = -1;
          } 
          flagEnded = false;
        

        }
        // Check CRC validity
        if (crcPass) {
          // Data received successfully, process it (e.g., print)
          int nodeTo = received_data[0];
          if (nodeTo == 0) { // for master. master does not retransmit, or a loop will runaway.
            flagProc = true;
            currentTask->gotCallback = true;
          
          }
          
        } else {
          //DIAG(F("IO_RSproto: CRC Error!"));
        }
      }

      // temp debug
      

      if (flagProc) {
        int nodeTo = received_data[0];
        int nodeFr = received_data[1];
        RSprotonode *node = findNode(nodeFr);
        //DIAG(F("Node from %i %i"), nodeFr, node->getNodeID());
        int AddrCode = received_data[2];
        //DIAG(F("From: %i, To: %i | %i %i %i %i %i"), nodeFr,nodeTo, received_data[3], received_data[4], received_data[5], received_data[6],received_data[7]);
        //return;
        
        switch (AddrCode) {
          case EXIOPINS:
            {node->setnumDigitalPins(received_data[3]);
            node->setnumAnalogPins(received_data[4]);

            // See if we already have suitable buffers assigned
            if (node->getnumDigialPins()>0) {
              size_t digitalBytesNeeded = (node->getnumDigialPins() + 7) / 8;
              if (node->getdigitalPinBytes() < digitalBytesNeeded) {
                // Not enough space, free any existing buffer and allocate a new one
                if (node->cleandigitalPinStates(digitalBytesNeeded)) {
                  node->setdigitalPinBytes(digitalBytesNeeded);
                } else {
                  DIAG(F("EX-IOExpander485 node:%d ERROR alloc %d bytes"), nodeFr, digitalBytesNeeded);
                  //_deviceState = DEVSTATE_FAILED;
                  node->setdigitalPinBytes(0);
                }
              }
            }
            
            if (node->getnumAnalogPins()>0) {
              size_t analogueBytesNeeded = node->getnumAnalogPins() * 2;
              if (node->getanalogPinBytes() < analogueBytesNeeded) {
                // Free any existing buffers and allocate new ones.
                
                if (node->cleanAnalogStates(analogueBytesNeeded)) {
                  node->setanalogPinBytes(analogueBytesNeeded);
                } else {
                  DIAG(F("EX-IOExpander485 node:%d ERROR alloc analog pin bytes"), nodeFr);
                  //_deviceState = DEVSTATE_FAILED;
                  node->setanalogPinBytes(0);
                }
              }
            }
            currentTask->processed = true;
            node->resFlag[currentTask->retFlag] = 1;
            break;}
          case EXIOINITA: {
            for (int i = 0; i < node->getnumAnalogPins(); i++) {
              node->setanalogPinMap(received_data[i+3], i);
            }
            currentTask->processed = true;
            node->resFlag[currentTask->retFlag] = 1;
            break;
          }
          case EXIOVER: {
            node->setMajVer(received_data[3]);
            node->setMinVer(received_data[4]);
            node->setPatVer(received_data[5]);
            DIAG(F("EX-IOExpander485: Found node %i v%i.%i.%i"),nodeFr, node->getMajVer(), node->getMinVer(), node->getPatVer());
            //if (!_currentNode->isInitialised()) {
              //_currentNode->setInitialised();
              //DIAG(F("EX-IOExpander485: Initialized  Node:%d "),  nodeFr);         
            //}
            currentTask->processed = true;
            node->resFlag[currentTask->retFlag] = 1;
            break;
          }
          case EXIORDY: {
            currentTask->processed = true;
            node->resFlag[currentTask->retFlag] = 1;
            break;
          }
          case EXIOERR: {
            currentTask->processed = true;
            node->resFlag[currentTask->retFlag] = -1;
            DIAG(F("Some sort of error was received... WHAT DID YOU DO!"));
            break;
          }
          case EXIORDD: {
            for (int i = 0; i < (node->_numDigitalPins+7)/8; i++) {
              node->setanalogInputStates(received_data[i+3], i);
            }
            currentTask->processed = true;
            node->resFlag[currentTask->retFlag] = 1;
            break;
          }
          case EXIORDAN: {
            for (int i = 0; i < node->_numAnaloguePins; i++) {
              node->setanalogInputBuffer(received_data[i+3], i);
            }
            currentTask->processed = true;
            node->resFlag[currentTask->retFlag] = 1;
            break;
          }
        }
        
      }
      flagProc = false;

    }
    if (currentTask->processed) {
      markTaskCompleted(currentTask->taskID);
    }
  }
  

  if (_currentMicros - _cycleStartTime >= _cycleTime/* && _currentNode->isInitialised()*/) {
    _cycleStartTime = _currentMicros;
    if (_currentNode == NULL) _currentNode = _nodeListStart;
    RSproto *bus = RSproto::findBus(0);
    uint8_t buffB[3];
    buffB[0] = (_currentNode->getNodeID());
    buffB[1] = (0);
    buffB[2] = (EXIORDD);
    bus->setBusy();
    bus->addTask(buffB, 3, EXIORDD);

    buffB[0] = (_currentNode->getNodeID());
    buffB[1] = (0);
    buffB[2] = (EXIORDD);
    bus->setBusy();
    bus->addTask(buffB, 3, EXIORDAN);
    _currentNode = _currentNode->getNext();
    //DIAG(F("Polling"));
  }
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
  _initialised = false;
  memset(resFlag, 0, 255);
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
    bus->setBusy();
    bus->addTask(buff, 5, EXIODPUP);
    
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
    bus->setBusy();
    bus->addTask(buff, 6, EXIOENAN);
    
    return false;
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
  bus->initTask();
  bus->setBusy();
  bus->addTask(buff, 6, EXIOINIT);
  
  buff[0] = (_nodeID);
  buff[1] = (0);
  buff[2] = (EXIOINITA);
  startMillis = millis();
  bus->setBusy();
  bus->addTask(buff, 3, EXIOINITA);
  
  buff[0] = (_nodeID);
  buff[1] = (0);
  buff[2] = (EXIOVER);
  startMillis = millis();
   bus->setBusy();
  bus->addTask(buff, 3, EXIOVER);
  

  setInitialised();
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
    bus->setBusy();
    bus->addTask(buff, 5, EXIOWRD);
    
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
    bus->setBusy();
    bus->addTask(buff, 9, EXIOWRAN);
    
  }