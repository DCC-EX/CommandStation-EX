/*
 *  © 2021, Neil McKechnie. All rights reserved.
 *
 *  This file is part of CommandStation-EX
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

#include <stdarg.h>
#include "I2CManager.h"
#include "DIAG.h"

// Include target-specific portions of I2CManager class
#if defined(I2C_USE_WIRE) 
#include "I2CManager_Wire.h"
#elif defined(ARDUINO_ARCH_AVR)
#include "I2CManager_NonBlocking.h"
#include "I2CManager_AVR.h"       // Uno/Nano/Mega2560
#elif defined(ARDUINO_ARCH_MEGAAVR) 
#include "I2CManager_NonBlocking.h"
#include "I2CManager_Mega4809.h"  // NanoEvery/UnoWifi
#else
#define I2C_USE_WIRE
#include "I2CManager_Wire.h"      // Other platforms
#endif


// If not already initialised, initialise I2C
void I2CManagerClass::begin(void) {
  //setTimeout(25000); // 25 millisecond timeout
  if (!_beginCompleted) {
    _beginCompleted = true;
    _initialise();

    // Probe and list devices.
    bool found = false;
    for (byte addr=1; addr<127; addr++) {
      if (exists(addr)) {
        found = true; 
        DIAG(F("I2C Device found at x%x"), addr);
      }
    }
    if (!found) DIAG(F("No I2C Devices found"));
  }
}

// Set clock speed to the lowest requested one. If none requested,
//  the Wire default is 100kHz.
void I2CManagerClass::setClock(uint32_t speed) {
  if (speed < _clockSpeed && !_clockSpeedFixed) {
    _clockSpeed = speed;
  }
  _setClock(_clockSpeed);
}

// Force clock speed to that specified.  It can then only 
// be overridden by calling Wire.setClock directly.
void I2CManagerClass::forceClock(uint32_t speed) {
  if (!_clockSpeedFixed) {
    _clockSpeed = speed;
    _clockSpeedFixed = true;
    _setClock(_clockSpeed);
  }
}

// Check if specified I2C address is responding (blocking operation)
// Returns I2C_STATUS_OK (0) if OK, or error code.
uint8_t I2CManagerClass::checkAddress(uint8_t address) {
  return write(address, NULL, 0);
}


/***************************************************************************
 *  Write a transmission to I2C using a list of data (blocking operation)
 ***************************************************************************/
uint8_t I2CManagerClass::write(uint8_t address, uint8_t nBytes, ...) {
  uint8_t buffer[nBytes];
  va_list args;
  va_start(args, nBytes);
  for (uint8_t i=0; i<nBytes; i++)
    buffer[i] = va_arg(args, int);
  va_end(args);
  return write(address, buffer, nBytes);
}

/***************************************************************************
 *  Initiate a write to an I2C device (blocking operation)
 ***************************************************************************/
uint8_t I2CManagerClass::write(uint8_t i2cAddress, const uint8_t writeBuffer[], uint8_t writeLen) {
  I2CRB req;
  uint8_t status = write(i2cAddress, writeBuffer, writeLen, &req);
  return finishRB(&req, status);
}

/***************************************************************************
 *  Initiate a write from PROGMEM (flash) to an I2C device (blocking operation)
 ***************************************************************************/
uint8_t I2CManagerClass::write_P(uint8_t i2cAddress, const uint8_t * data, uint8_t dataLen) {
  I2CRB req;
  uint8_t status = write_P(i2cAddress, data, dataLen, &req);
  return finishRB(&req, status);
}

/***************************************************************************
 *  Initiate a write (optional) followed by a read from the I2C device (blocking operation)
 ***************************************************************************/
uint8_t I2CManagerClass::read(uint8_t i2cAddress, uint8_t *readBuffer, uint8_t readLen, 
    const uint8_t *writeBuffer, uint8_t writeLen)
{
  I2CRB req;
  uint8_t status = read(i2cAddress, readBuffer, readLen, writeBuffer, writeLen, &req);
  return finishRB(&req, status);
}

/***************************************************************************
 *  Overload of read() to allow command to be specified as a series of bytes (blocking operation)
 ***************************************************************************/
uint8_t I2CManagerClass::read(uint8_t address, uint8_t readBuffer[], uint8_t readSize, 
                                  uint8_t writeSize, ...) {
  va_list args;
  // Copy the series of bytes into an array.
  va_start(args, writeSize);
  uint8_t writeBuffer[writeSize];
  for (uint8_t i=0; i<writeSize; i++)
    writeBuffer[i] = va_arg(args, int);
  va_end(args);
  return read(address, readBuffer, readSize, writeBuffer, writeSize);
}

/***************************************************************************
 * Finish off request block by posting status, etc. (blocking operation)
 ***************************************************************************/
uint8_t I2CManagerClass::finishRB(I2CRB *rb, uint8_t status) {
  if ((status == I2C_STATUS_OK) && rb)
    status = rb->wait();
  return status;
}

/***************************************************************************
 * Get a message corresponding to the error status
 ***************************************************************************/
const FSH *I2CManagerClass::getErrorMessage(uint8_t status) {
  switch (status) {
    case I2C_STATUS_OK: return F("OK");
    case I2C_STATUS_TRUNCATED: return F("Transmission truncated");
    case I2C_STATUS_NEGATIVE_ACKNOWLEDGE: return F("No response from device (address NAK)");
    case I2C_STATUS_TRANSMIT_ERROR: return F("Transmit error (data NAK)");
    case I2C_STATUS_OTHER_TWI_ERROR: return F("Other Wire/TWI error");
    case I2C_STATUS_TIMEOUT: return F("Timeout");
    case I2C_STATUS_ARBITRATION_LOST: return F("Arbitration lost");
    case I2C_STATUS_BUS_ERROR: return F("I2C bus error");
    case I2C_STATUS_UNEXPECTED_ERROR: return F("Unexpected error");
    case I2C_STATUS_PENDING: return F("Request pending");
    default: return F("Error code not recognised");
  }
}

/***************************************************************************
 *  Declare singleton class instance.
 ***************************************************************************/
I2CManagerClass I2CManager = I2CManagerClass();


/////////////////////////////////////////////////////////////////////////////
// Helper functions associated with I2C Request Block
/////////////////////////////////////////////////////////////////////////////

/***************************************************************************
 *  Block waiting for request block to complete, and return completion status.
 *  Since such a loop could potentially last for ever if the RB status doesn't
 *  change, we set a high limit (1sec, 1000ms) on the wait time and, if it
 *  hasn't changed by that time we assume it's not going to, and just return
 *  a timeout status.  This means that CS will not lock up.
 ***************************************************************************/
uint8_t I2CRB::wait() {
  unsigned long waitStart = millis();
  do {
    I2CManager.loop();
    // Rather than looping indefinitely, let's set a very high timeout (1s).
    if ((millis() - waitStart) > 1000UL) { 
      DIAG(F("I2C TIMEOUT I2C:x%x I2CRB:x%x"), i2cAddress, this);
      status = I2C_STATUS_TIMEOUT;
      // Note that, although the timeout is posted, the request may yet complete.
      // TODO: Ideally we would like to cancel the request.
      return status;
    }
  } while (status==I2C_STATUS_PENDING);
  return status;
}

/***************************************************************************
 *  Check whether request is still in progress.
 ***************************************************************************/
bool I2CRB::isBusy() {
  I2CManager.loop();
  return (status==I2C_STATUS_PENDING);
}

/***************************************************************************
 *  Helper functions to fill the I2CRequest structure with parameters.
 ***************************************************************************/
void I2CRB::setReadParams(uint8_t i2cAddress, uint8_t *readBuffer, uint8_t readLen) {
  this->i2cAddress = i2cAddress;
  this->writeLen = 0;
  this->readBuffer = readBuffer;
  this->readLen = readLen;
  this->operation = OPERATION_READ;
  this->status = I2C_STATUS_OK;
}

void I2CRB::setRequestParams(uint8_t i2cAddress, uint8_t *readBuffer, uint8_t readLen, 
    const uint8_t *writeBuffer, uint8_t writeLen) {
  this->i2cAddress = i2cAddress;
  this->writeBuffer = writeBuffer;
  this->writeLen = writeLen;
  this->readBuffer = readBuffer;
  this->readLen = readLen;
  this->operation = OPERATION_REQUEST;
  this->status = I2C_STATUS_OK;
}

void I2CRB::setWriteParams(uint8_t i2cAddress, const uint8_t *writeBuffer, uint8_t writeLen) {
  this->i2cAddress = i2cAddress;
  this->writeBuffer = writeBuffer;
  this->writeLen = writeLen;
  this->readLen = 0;
  this->operation = OPERATION_SEND;
  this->status = I2C_STATUS_OK;
}

