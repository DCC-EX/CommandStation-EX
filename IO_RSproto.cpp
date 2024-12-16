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

taskBuffer::taskBuffer(Stream * myserial)
{
  serial = myserial;
  next=first;
  first=this;
  bufferLength=0;
  inCommandPayload=PAYLOAD_FALSE;
}

taskBuffer::~taskBuffer()
{
  // destructor
}

void taskBuffer::doCommand(uint8_t *commandBuffer, int commandSize) {
  for (taskBuffer * t=first;t;t=t->next) t->doCommand2(commandBuffer,commandSize);
}

void taskBuffer::doCommand2(uint8_t *commandBuffer, int commandSize) {
  // process commands here to be sent
 
  //_serial->begin(115200);
  //ArduinoPins::fastWriteDigital(bus->_txPin, HIGH);
  if (_txPin != -1) digitalWrite(_txPin,HIGH);
  serial->write(commandBuffer, 7);
  //serial->flush();
  if (_txPin != -1) digitalWrite(_txPin,LOW);
}

void taskBuffer::init(HardwareSerial &hwSerial, unsigned long baud, int8_t txPin) {
  hwSerial.begin(baud, SERIAL_8N1);
  new taskBuffer(&hwSerial);
  for (taskBuffer * t=first;t;t=t->next) t->_txPin = txPin;
  if (txPin != -1) pinMode(txPin, OUTPUT);
  if (txPin != -1) digitalWrite(txPin, LOW);
}

void taskBuffer::loop() {
  for (taskBuffer * t=first;t;t=t->next) t->loop2();
}

void taskBuffer::loop2() {
  // process received commands here
  while (serial->available()) {
    char ch = serial->read();
    if (!inCommandPayload) {
      if (ch == STARTBYTE) {
        inCommandPayload = PAYLOAD_NORMAL;
        bufferLength = 0;
        buffer[0] = '\0';
      }
    } else { // if (inCommandPayload)
      if (bufferLength <  (COMMAND_BUFFER_SIZE-1))
        buffer[bufferLength++] = ch;
      if (inCommandPayload > PAYLOAD_NORMAL) {
        if (inCommandPayload > 32 + 2) {    // String way too long
          ch = ENDBYTE;                         // we end this nonsense
          inCommandPayload = PAYLOAD_NORMAL;
          DIAG(F("Parse error: Unbalanced string"));
          // fall through to ending parsing below
        } else if (ch == '"') {               // String end
          inCommandPayload = PAYLOAD_NORMAL;
          continue; // do not fall through
        } else
          inCommandPayload++;
      }
      if (inCommandPayload == PAYLOAD_NORMAL) {
        if (ch == ENDBYTE) {
          buffer[bufferLength] = '\0';
          parseRx(buffer); 
          inCommandPayload = PAYLOAD_FALSE;
          break;
        } else if (ch == '"') {
          inCommandPayload = PAYLOAD_STRING;
        }
      }
    }
  }
}

void taskBuffer::parseRx(uint8_t *buf) {
  // pass on what we got
  bool found = (buf[0] != STARTBYTE);
  for (byte *b=buf; b[0] != '\0'; b++) {
    if (found) {
      parseOne(b);
      found=false;
    }
    if (b[0] == STARTBYTE)
      found = true;
  }
}

void taskBuffer::parseOne(uint8_t *buf) {
  // finaly, process the darn data
  while (buf[0] == '<' || buf[0] == ' ')
    buf++; // strip off any number of < or spaces
  
  uint8_t toNode = buf[0];
  if (toNode != 0) return; // not for master
  uint8_t fromNode = buf[1];
  if (fromNode == 0) return; // why did out own data come round the ring back to us?
  uint8_t opcode = buf[2];
  RSproto *bus = RSproto::findBus(0);
  RSprotonode *node = bus->findNode(fromNode);
  switch (opcode) {
    case EXIOPINS:
      {node->_numDigitalPins = buf[3];
      node->_numAnaloguePins = buf[4];

      // See if we already have suitable buffers assigned
      if (node->_numDigitalPins>0) {
        size_t digitalBytesNeeded = (node->_numDigitalPins + 7) / 8;
        if (node->_digitalPinBytes < digitalBytesNeeded) {
          // Not enough space, free any existing buffer and allocate a new one
          if (node->_digitalPinBytes > 0) free(node->_digitalInputStates);
          if ((node->_digitalInputStates = (byte*) calloc(digitalBytesNeeded, 1)) != NULL) {
            node->_digitalPinBytes = digitalBytesNeeded;
          } else {
            DIAG(F("EX-IOExpander485 node:%d ERROR alloc %d bytes"), fromNode, digitalBytesNeeded);
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
            DIAG(F("EX-IOExpander485 node:%d ERROR alloc analog pin bytes"), fromNode);
            //_deviceState = DEVSTATE_FAILED;
            node->_analoguePinBytes = 0;
            return;
          }
        }
      }
      node->resFlag = 1;
      break;}
    case EXIOINITA: {
      for (int i = 3; i < node->_numAnaloguePins; i++) {
        node->_analoguePinMap[i] = buf[i];
      }
      node->resFlag = 1;
      break;
    }
    case EXIOVER: {
      node->_majorVer = buf[3];
      node->_minorVer = buf[4];
      node->_patchVer = buf[5];
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
      for (int i = 3; i < (node->_numDigitalPins+7)/8; i++) {
        node->_digitalInputStates[i-3] = buf[i];
      }
      node->resFlag = 1;
      break;
    }
    case EXIORDAN: {
      for (int i = 3; i < node->_numAnaloguePins*2; i++) {
        node->_analogueInputBuffer[i-3] = buf[i];
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
RSproto::RSproto(uint8_t busNo, HardwareSerial &serial, unsigned long baud, int8_t txPin) {
  _serial = &serial;
  _baud = baud;
  _txPin = txPin;
  _busNo = busNo;
  task->init(serial, baud, txPin);
  if (_waitA < 3) _waitA = 3;
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
void RSproto::_loop(unsigned long currentMicros) {
  _currentMicros = currentMicros;
  
  //if (_currentNode == NULL) {
   // _currentNode = _nodeListStart;
    
  //}

  //if (_currentMicros - _cycleStartTime < _cycleTime) return;
  //_cycleStartTime = _currentMicros;
  //if (_currentNode == NULL) return;
  task->loop();
  
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
      uint8_t outBuffer[] = {0xFD, _nodeID, 0, EXIODPUP, (uint8_t)pin, pullup, 0xFE};
      unsigned long startMillis = millis();
      task->doCommand(outBuffer,7);
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
    uint8_t commandBuffer[] = {0xFD, _nodeID, 0, EXIOENAN, (uint8_t) pin, 0xFE};
    unsigned long startMillis = millis();
    task->doCommand(commandBuffer, 6);
    while (resFlag == 0 && millis() - startMillis < 500); // blocking for now
    if (resFlag != 1) {
      DIAG(F("EX-IOExpander485 Vpin %u cannot be used as a digital input pin"), pin);
      return false;
    }
    resFlag = 0;
    return true;
  }

void RSprotonode::_begin() {
  uint8_t commandBuffer[] = {0xFD, _nodeID, 0, EXIOINIT, (uint8_t)_nPins, (uint8_t)(_firstVpin & 0xFF), (uint8_t)(_firstVpin >> 8), 0xFE};
    unsigned long startMillis = millis();
  task->doCommand(commandBuffer,8);
  while (resFlag == 0 && millis() - startMillis < 1000); // blocking for now
  if (resFlag != 1) {
    DIAG(F("EX-IOExpander485 Node:%d ERROR EXIOINIT"), _nodeID);
  }
  resFlag = 0;
  commandBuffer[0] = 0xFD;
  commandBuffer[1] = _nodeID;
  commandBuffer[2] = 0;
  commandBuffer[3] = EXIOINITA;
  commandBuffer[4] = 0xFE;
  startMillis = millis();
  task->doCommand(commandBuffer,5);
  while (resFlag == 0 && millis() - startMillis < 1000); // blocking for now
  if (resFlag != 1) {
    DIAG(F("EX-IOExpander485 Node:%d ERROR EXIOINITA"), _nodeID);
  }
  resFlag = 0;
  commandBuffer[0] = 0xFD;
  commandBuffer[1] = _nodeID;
  commandBuffer[2] = 0;
  commandBuffer[3] = EXIOVER;
  commandBuffer[4] = 0xFE;
  startMillis = millis();
  task->doCommand(commandBuffer,3);
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
    uint8_t digitalOutBuffer[7];
    digitalOutBuffer[0] = 0xFD;
    digitalOutBuffer[1] = _nodeID;
  digitalOutBuffer[2] = 0;
  digitalOutBuffer[3] = EXIOWRD;
    digitalOutBuffer[4] = (uint8_t) pin;
    digitalOutBuffer[5] = (uint8_t) value;
    digitalOutBuffer[6] = 0xFE;
    unsigned long startMillis = millis();
    task->doCommand(digitalOutBuffer,7);
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
    uint8_t servoBuffer[11];
    int pin = vpin - _firstVpin;
    servoBuffer[0] = 0xFD;
    servoBuffer[1] = _nodeID;
    servoBuffer[2] = 0;
    servoBuffer[3] = EXIOWRAN;
    servoBuffer[4] = (uint8_t) pin;
    servoBuffer[5] = (uint8_t) value & 0xFF;
    servoBuffer[6] = (uint8_t) value >> 8;
    servoBuffer[7] = (uint8_t) profile;
    servoBuffer[8] = (uint8_t) duration & 0xFF;
    servoBuffer[9] = (uint8_t) duration >> 8;
    servoBuffer[10] = 0xFE;
    unsigned long startMillis = millis();
    task->doCommand(servoBuffer,11);
    while (resFlag == 0 && millis() - startMillis < 500); // blocking for now
    if (resFlag != 1) {
      DIAG(F("EX-IOExpander485 Node:%d ERROR EXIOVER"), _nodeID);
    }
    resFlag = 0;
  }