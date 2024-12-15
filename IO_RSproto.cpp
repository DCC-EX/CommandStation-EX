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

static const byte PAYLOAD_FALSE = 0;
static const byte PAYLOAD_NORMAL = 1;
static const byte PAYLOAD_STRING = 2;

taskBuffer::taskBuffer(Stream * myserial)
{
  // constructor
  next=first;
  first=this;
  serial = myserial;
}

taskBuffer::~taskBuffer()
{
  // destructor
}

/* -= updateCrc =-
//
// add the CRC value from _calculateCrc (2 bytes) to the buffer.
*/
void taskBuffer::updateCrc(uint8_t *crcBuf, uint8_t *buf, uint16_t len) {
  if (sizeof(crcBuf) != 2) return;
  uint16_t crc = _calculateCrc(buf, len);
  crcBuf[0] = lowByte(crc);
  crcBuf[1] = highByte(crc);
}

/* -= crcGood =-
//
// return TRUE if CRC matched between buffer copy, and calculated.
*/
bool taskBuffer::crcGood(uint8_t *buf, uint16_t len) {
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
uint16_t taskBuffer::_calculateCrc(uint8_t *buf, uint16_t len) {
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

void taskBuffer::doCommand(uint8_t *commandBuffer, int commandSize) {
  for (taskBuffer * t=first;t;t=t->next) t->doCommand2(nodeID,commandBuffer,commandSize);
}

void taskBuffer::doCommand2(uint8_t *commandBuffer, int commandSize) {
  // process commands here to be sent
  uint8_t crcBuffer[2];
  updateCrc(crcBuffer, commandBuffer, commandSize);
  
  //_serial->begin(115200);
  //ArduinoPins::fastWriteDigital(bus->_txPin, HIGH);
  digitalWrite(_txPin,HIGH);
  unsigned long startMillis = millis();
  
  serial->write(commandBuffer, 7);
  serial->write(endChar, 1);
  serial->flush();
  digitalWrite(_txPin,LOW);
}

void taskBuffer::init(unsigned long baud, uint16_t cycleTimeMS, int8_t txPin) {
#ifdef RS485_SERIAL
  RS485_SERIAL.begin(baud, SERIAL_8N1);
  new taskBuffer(&RS485_SERIAL);
#endif
  for (taskBuffer * t=first;t;t=t->next) t->_txPin = txPin;
  pinMode(txPin, OUTPUT);
  digitalWrite(txPin, LOW);
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
  uint8_t opcode = buf[2];
  
  RSprotonode *node = RSprotonode::findNode(fromNode);
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
      break;}
    case EXIOPINS: {
      for (int i = 3; i < node->_numAnaloguePins; i++) {
        node->_analoguePinMap[i] = buf[i];
      }
      break;
    }
    case EXIOVER: {
      node->_majorVer = buf[3];
      node->_minorVer = buf[4];
      node->_patchVer = buf[5];
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
      break;
    }
    case EXIORDAN: {
      for (int i = 3; i < node->_numAnaloguePins*2; i++) {
        node->_analogueInputBuffer[i-3] = buf[i];
      }
      break;
    }
  }
}

/************************************************************
 * RSproto implementation
 ************************************************************/

// Constructor for RSproto
RSproto::RSproto(HardwareSerial &serial, unsigned long baud, uint16_t cycleTimeMS, int8_t txPin, int waitA) {
  _baud = baud;
  _serialD = &serial;
  _txPin = txPin;
  _busNo = 0;
  task->init(baud, cycleTimeMS, txPin);
  _cycleTime = cycleTimeMS * 1000UL; // convert from milliseconds to microseconds.
  _waitA = waitA;
  if (_waitA < 3) _waitA = 3;
  // Add device to HAL device chain
  IODevice::addDevice(this);
  
  // Add bus to RSproto chain.
  _nextBus = _busList;
  _busList = this;
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
      uint8_t outBuffer[6] = {EXIODPUP, pin, pullup};
      task->doCommand(outBuffer,3);
      unsigned long startMillis = millis();
      while (resFlag == 0 && millis() - startMillis < 500); // blocking for now
        if (resFlag != 1) {
          DIAG(F("EX-IOExpander485 Vpin %u cannot be used as a digital input pin"), pin);
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