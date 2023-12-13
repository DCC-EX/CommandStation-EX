/*
 *  © 2023, Neil McKechnie. All rights reserved.
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

#include "IO_CMRI.h"
#include "defines.h"

/************************************************************
 * CMRIbus implementation
 ************************************************************/

// Constructor for CMRIbus
CMRIbus::CMRIbus(uint8_t busNo, HardwareSerial &serial, unsigned long baud, uint16_t cycleTimeMS, VPIN transmitEnablePin) {
  _busNo = busNo;
  _serial = &serial;
  _baud = baud;
  _cycleTime = cycleTimeMS * 1000UL; // convert from milliseconds to microseconds.
  _transmitEnablePin = transmitEnablePin;
  if (_transmitEnablePin != VPIN_NONE) {
    pinMode(_transmitEnablePin, OUTPUT);
    ArduinoPins::fastWriteDigital(_transmitEnablePin, 0); // transmitter initially off
  }

  // Max message length is 256+6=262 bytes.  
  // Each byte is one start bit, 8 data bits and 1 or 2 stop bits, assume 11 bits per byte.  
  // Calculate timeout based on treble this time.
  _timeoutPeriod = 3 * 11 * 262 * 1000UL / (_baud / 1000UL);
#if defined(ARDUINOCMRI_COMPATIBLE)
  // NOTE: The ArduinoCMRI library, unless modified, contains a 'delay(50)' between 
  // receiving the end of the prompt message and starting to send the response.  This
  // is allowed for below.
  _timeoutPeriod += 50000UL;
#endif
  
  // Calculate the time in microseconds to transmit one byte (11 bits max).
  _byteTransmitTime = 1000000UL * 11 / _baud;
  // Postdelay is only required if we need to allow for data still being sent when
  // we want to switch off the transmitter.  The flush() method of HardwareSerial
  // ensures that the data has completed being sent over the line.
  _postDelay = 0;
  
  // Add device to HAL device chain
  IODevice::addDevice(this);

  // Add bus to CMRIbus chain.
  _nextBus = _busList;
  _busList = this;
}


// Main loop function for CMRIbus.
// Work through list of nodes.  For each node, in separate loop entries
// send initialisation message (once only);  then send
// output message;  then send prompt for input data, and 
// process any response data received.
// When the slot time has finished, move on to the next device.
void CMRIbus::_loop(unsigned long currentMicros) {
  
  _currentMicros = currentMicros;

  while (_serial->available())
    processIncoming();

  // Send any data that needs sending.
  processOutgoing();

}

// Send output data to the bus for nominated CMRInode
uint16_t CMRIbus::sendData(CMRInode *node) {
  uint16_t numDataBytes = (node->getNumOutputs()+7)/8;
  _serial->write(SYN);
  _serial->write(SYN);
  _serial->write(STX);
  _serial->write(node->getAddress() + 65);
  _serial->write('T'); // T for Transmit data message
  uint16_t charsSent = 6; // include header and trailer
  for (uint8_t index=0; index<numDataBytes; index++) {
    uint8_t value = node->getOutputStates(index);
    if (value == DLE || value == STX || value == ETX) {
      _serial->write(DLE); 
      charsSent++;
    }
    _serial->write(value);
    charsSent++;
  }
  _serial->write(ETX);
  return charsSent;  // number of characters sent
}

// Send request for input data to nominated CMRInode.
uint16_t CMRIbus::requestData(CMRInode *node) {
  _serial->write(SYN);
  _serial->write(SYN);
  _serial->write(STX);
  _serial->write(node->getAddress() + 65);
  _serial->write('P'); // P for Poll message
  _serial->write(ETX);
  return 6;  // number of characters sent
}

// Send initialisation message
uint16_t CMRIbus::sendInitialisation(CMRInode *node) {
  _serial->write(SYN);
  _serial->write(SYN);
  _serial->write(STX);
  _serial->write(node->getAddress() + 65);
  _serial->write('I'); // I for initialise message
  _serial->write(node->getType());  // NDP
  _serial->write((uint8_t)0);  // dH
  _serial->write((uint8_t)0);  // dL
  _serial->write((uint8_t)0);  // NS
  _serial->write(ETX);
  return 10;  // number of characters sent
}

void CMRIbus::processOutgoing() {
  uint16_t charsSent = 0;
  if (_currentNode == NULL) {
    // If we're between read/write cycles then don't do anything else.
    if (_currentMicros - _cycleStartTime < _cycleTime) return;
    // ... otherwise start processing the first node in the list
    _currentNode = _nodeListStart;
    _transmitState = TD_INIT;
    _cycleStartTime = _currentMicros;
  }
  if (_currentNode == NULL) return;
  switch (_transmitState) {
    case TD_IDLE:
    case TD_INIT:
      enableTransmitter();
      if (!_currentNode->isInitialised()) {
        charsSent = sendInitialisation(_currentNode);
        _currentNode->setInitialised();
        _transmitState = TD_TRANSMIT;
        delayUntil(_currentMicros+_byteTransmitTime*charsSent);
        break;
      }
      /* fallthrough */
    case TD_TRANSMIT:
      charsSent = sendData(_currentNode);
      _transmitState = TD_PROMPT;
      // Defer next entry for as long as it takes to transmit the characters, 
      // to allow output queue to empty.  Allow 2 bytes extra.
      delayUntil(_currentMicros+_byteTransmitTime*(charsSent+2));
      break;
    case TD_PROMPT:
      charsSent = requestData(_currentNode);
      disableTransmitter();
      _transmitState = TD_RECEIVE;
      _timeoutStart = _currentMicros; // Start timeout on response
      break;
    case TD_RECEIVE: // Waiting for response / timeout
      if (_currentMicros - _timeoutStart > _timeoutPeriod) { 
        // End of time slot allocated for responses.
        _transmitState = TD_IDLE;
        // Reset state of receiver
        _receiveState = RD_SYN1;
        // Move to next node
        _currentNode = _currentNode->getNext();
      }
      break;
  }
}

// Process any data bytes received from a CMRInode.
void CMRIbus::processIncoming() {
  int data = _serial->read();
  if (data < 0) return;     // No characters to read

  if (_transmitState != TD_RECEIVE || !_currentNode) return;   // Not waiting for input, so ignore.

  uint8_t nextState = RD_SYN1;  // default to resetting state machine
  switch(_receiveState) {
    case RD_SYN1: 
      if (data == SYN) nextState = RD_SYN2; 
      break;
    case RD_SYN2:
      if (data == SYN) nextState = RD_STX; else nextState = RD_SYN2;
      break;
    case RD_STX:
      if (data == STX) nextState = RD_ADDR;
      break;
    case RD_ADDR:
      // If address doesn't match, then ignore everything until next SYN-SYN-STX.
      if (data == _currentNode->getAddress() + 65) nextState = RD_TYPE;
      break;
    case RD_TYPE:
      _receiveDataIndex = 0;  // Initialise data pointer
      if (data == 'R') nextState = RD_DATA;
      break;
    case RD_DATA: // data body
      if (data == DLE) // escape next character
        nextState = RD_ESCDATA;
      else if (data == ETX) { // end of data
        // End of data message.  Protocol has all data in one
        // message, so we don't need to wait any more.  Allow
        // transmitter to proceed with next node in list.
        _currentNode = _currentNode->getNext();
        _transmitState = TD_IDLE;
      } else {
        // Not end yet, so save data byte
        _currentNode->saveIncomingData(_receiveDataIndex++, data);
        nextState = RD_DATA; // wait for more data
      }
      break;
    case RD_ESCDATA: // escaped data byte
      _currentNode->saveIncomingData(_receiveDataIndex++, data);
      nextState = RD_DATA;
      break;
  }
  _receiveState = nextState;
}

// If configured for half duplex RS485, switch RS485 interface
// into transmit mode.
void CMRIbus::enableTransmitter() {
  if (_transmitEnablePin != VPIN_NONE) 
    ArduinoPins::fastWriteDigital(_transmitEnablePin, 1);
  // If we need a delay before we start the packet header, 
  // we can send a character or two to synchronise the 
  // transmitter and receiver.
  // SYN characters should be used, but a bug in the
  // ArduinoCMRI library causes it to ignore the packet if
  // it's preceded by an odd number of SYN characters.
  // So send a SYN followed by a NUL in that case.
  _serial->write(SYN);
#if defined(ARDUINOCMRI_COMPATIBLE)
  _serial->write(NUL);  // Reset the ArduinoCMRI library's parser
#endif
}

// If configured for half duplex RS485, switch RS485 interface
// into receive mode.
void CMRIbus::disableTransmitter() {
  // Wait until all data has been transmitted.  On the standard
  // AVR driver, this waits until the FIFO is empty and all
  // data has been sent over the link.
  _serial->flush();
  // If we don't trust the 'flush' function and think the 
  // data's still in transit, then wait a bit longer.
  if (_postDelay > 0)
    delayMicroseconds(_postDelay);
  // Hopefully, we can now safely switch off the transmitter.
  if (_transmitEnablePin != VPIN_NONE) 
    ArduinoPins::fastWriteDigital(_transmitEnablePin, 0);
}    

// Link to chain of CMRI bus instances
CMRIbus *CMRIbus::_busList = NULL;


/************************************************************
 * CMRInode implementation
 ************************************************************/

// Constructor for CMRInode object
CMRInode::CMRInode(VPIN firstVpin, int nPins, uint8_t busNo, uint8_t address, char type, uint16_t inputs, uint16_t outputs) {
  _firstVpin = firstVpin;
  _nPins = nPins;
  _busNo = busNo;
  _address = address;
  _type = type;

  switch (_type) {
    case 'M': // SMINI, fixed 24 inputs and 48 outputs
      _numInputs = 24;
      _numOutputs = 48;
      break;
    case 'C': // CPNODE with 16 to 144 inputs/outputs using 8-bit cards
      _numInputs = inputs;
      _numOutputs = outputs;
      break;
    case 'N': // Classic USIC and SUSIC using 24 bit i/o cards
    case 'X': // SUSIC using 32 bit i/o cards
    default:
      DIAG(F("CMRInode: bus:%d address:%d ERROR unsupported type %c"), _busNo, _address, _type);
      return; // Don't register device.
  }
  if ((unsigned int)_nPins < _numInputs + _numOutputs)
    DIAG(F("CMRInode: bus:%d address:%d WARNING number of Vpins does not cover all inputs and outputs"), _busNo, _address);

  // Allocate memory for states
  _inputStates = (uint8_t *)calloc((_numInputs+7)/8, 1);
  _outputStates = (uint8_t *)calloc((_numOutputs+7)/8, 1);
  if (!_inputStates || !_outputStates) {
    DIAG(F("CMRInode: ERROR insufficient memory"));
    return;
  }

  // Add this device to HAL device list
  IODevice::addDevice(this);

  // Add CMRInode to CMRIbus object.
  CMRIbus *bus = CMRIbus::findBus(_busNo);
  if (bus != NULL) {
    bus->addNode(this);
    return;
  }
}

