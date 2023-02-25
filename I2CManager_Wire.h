/*
 *  © 2023, Neil McKechnie. All rights reserved.
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

#ifndef I2CMANAGER_WIRE_H
#define I2CMANAGER_WIRE_H

#include <Arduino.h>
#include <Wire.h>
#include "I2CManager.h"

// This module is only compiled if I2C_USE_WIRE is defined, so define it here
// to get intellisense to work correctly.
#if !defined(I2C_USE_WIRE)
#define I2C_USE_WIRE
#endif

// Older versions of Wire don't have setWireTimeout function.  AVR does.
#ifdef ARDUINO_ARCH_AVR
#define WIRE_HAS_TIMEOUT
#endif

/***************************************************************************
 *  Initialise I2C interface software
 ***************************************************************************/
void I2CManagerClass::_initialise() {
  Wire.begin();
#if defined(WIRE_HAS_TIMEOUT) 
  Wire.setWireTimeout(_timeout, true);
#endif
}

/***************************************************************************
 *  Set I2C clock speed.  Normally 100000 (Standard) or 400000 (Fast)
 *   on Arduino.  Mega4809 supports 1000000 (Fast+) too.
 ***************************************************************************/
void I2CManagerClass::_setClock(unsigned long i2cClockSpeed) {
  Wire.setClock(i2cClockSpeed);
}

/***************************************************************************
 *  Set I2C timeout value in microseconds.  The timeout applies to each
 *   Wire call separately, i.e. in a write+read, the timer is reset before the
 *   read is started.
 ***************************************************************************/
void I2CManagerClass::setTimeout(unsigned long value) {
  _timeout = value;
#if defined(WIRE_HAS_TIMEOUT) 
  Wire.setWireTimeout(value, true);
#endif
}

/********************************************************
 * Helper function for I2C Multiplexer operations
 ********************************************************/
#ifdef I2C_EXTENDED_ADDRESS
static uint8_t muxSelect(I2CAddress address) {
  // Select MUX sub bus.
  I2CMux muxNo = address.muxNumber();
  I2CSubBus subBus = address.subBus();
  if (muxNo != I2CMux_None) {
    Wire.beginTransmission(I2C_MUX_BASE_ADDRESS+muxNo); 
    uint8_t data =  (subBus == SubBus_All) ? 0xff :
                    (subBus == SubBus_None) ? 0x00 :
#if defined(I2CMUX_PCA9547)
                    0x08 | subBus;
#elif defined(I2CMUX_PCA9542) || defined(I2CMUX_PCA9544)
                    0x04 | subBus;   // NB Only 2 or 4 subbuses respectively
#else
                    // Default behaviour for most MUXs is to use a mask
                    // with a bit set for the subBus to be enabled
                    1 << subBus;
#endif
    Wire.write(&data, 1);
    return Wire.endTransmission(true);  // have to release I2C bus for it to work
  }
  return I2C_STATUS_OK;
}
#endif


/***************************************************************************
 *  Initiate a write to an I2C device (blocking operation on Wire)
 ***************************************************************************/
uint8_t I2CManagerClass::write(I2CAddress address, const uint8_t buffer[], uint8_t size, I2CRB *rb) {
  uint8_t status, muxStatus;
  uint8_t retryCount = 0;
  // If request fails, retry up to the defined limit, unless the NORETRY flag is set
  // in the request block.
  do {
    status = muxStatus = I2C_STATUS_OK;
#ifdef I2C_EXTENDED_ADDRESS
    if (address.muxNumber() != I2CMux_None)
      muxStatus = muxSelect(address);
#endif
    // Only send new transaction if address is non-zero.
    if (muxStatus == I2C_STATUS_OK && address != 0) {
      Wire.beginTransmission(address);
      if (size > 0) Wire.write(buffer, size);
      status = Wire.endTransmission();
    }
#ifdef I2C_EXTENDED_ADDRESS
    // Deselect MUX if there's more than one MUX present, to avoid having multiple ones selected
    if (_muxCount > 1 && muxStatus == I2C_STATUS_OK 
          && address.deviceAddress() != 0 && address.muxNumber() != I2CMux_None) {
      muxSelect({address.muxNumber(), SubBus_None});
    }
    if (muxStatus != I2C_STATUS_OK) status = muxStatus;
#endif
  } while (!(status == I2C_STATUS_OK
    || ++retryCount > MAX_I2C_RETRIES || rb->operation & OPERATION_NORETRY));
  rb->status = status;
  return I2C_STATUS_OK;
}

/***************************************************************************
 *  Initiate a write from PROGMEM (flash) to an I2C device (blocking operation on Wire)
 ***************************************************************************/
uint8_t I2CManagerClass::write_P(I2CAddress address, const uint8_t buffer[], uint8_t size, I2CRB *rb) {
  uint8_t ramBuffer[size];
  const uint8_t *p1 = buffer;
  for (uint8_t i=0; i<size; i++)
    ramBuffer[i] = GETFLASH(p1++);
  return write(address, ramBuffer, size, rb);
}

/***************************************************************************
 *  Initiate a write (optional) followed by a read from the I2C device (blocking operation on Wire)
 *  If fewer than the number of requested bytes are received, status is I2C_STATUS_TRUNCATED.
 ***************************************************************************/
uint8_t I2CManagerClass::read(I2CAddress address, uint8_t readBuffer[], uint8_t readSize,
                              const uint8_t writeBuffer[], uint8_t writeSize, I2CRB *rb)
{
  uint8_t status, muxStatus;
  uint8_t nBytes = 0;
  uint8_t retryCount = 0;
  // If request fails, retry up to the defined limit, unless the NORETRY flag is set
  // in the request block.
  do {
    status = muxStatus = I2C_STATUS_OK;
#ifdef I2C_EXTENDED_ADDRESS
    if (address.muxNumber() != I2CMux_None) {
      muxStatus = muxSelect(address);
    }
#endif
    // Only start new transaction if address is non-zero.
    if (muxStatus == I2C_STATUS_OK && address != 0) {
      if (writeSize > 0) {
        Wire.beginTransmission(address);
        Wire.write(writeBuffer, writeSize);
        status = Wire.endTransmission(false); // Don't free bus yet
      }
      if (status == I2C_STATUS_OK) {
#ifdef WIRE_HAS_TIMEOUT
        Wire.clearWireTimeoutFlag();
        Wire.requestFrom(address, (size_t)readSize);
        if (!Wire.getWireTimeoutFlag()) {
          while (Wire.available() && nBytes < readSize) 
            readBuffer[nBytes++] = Wire.read();
          if (nBytes < readSize) status = I2C_STATUS_TRUNCATED;
        } else {
          status = I2C_STATUS_TIMEOUT;
        }
#else
        Wire.requestFrom(address, (size_t)readSize);
          while (Wire.available() && nBytes < readSize) 
            readBuffer[nBytes++] = Wire.read();
          if (nBytes < readSize) status = I2C_STATUS_TRUNCATED;
#endif
      }
    }
#ifdef I2C_EXTENDED_ADDRESS
    // Deselect MUX if there's more than one MUX present, to avoid having multiple ones selected
    if (_muxCount > 1 && muxStatus == I2C_STATUS_OK && address != 0 && address.muxNumber() != I2CMux_None) {
      muxSelect({address.muxNumber(), SubBus_None});
    }
    if (muxStatus != I2C_STATUS_OK) status = muxStatus;
#endif

  } while (!((status == I2C_STATUS_OK) 
    || ++retryCount > MAX_I2C_RETRIES || rb->operation & OPERATION_NORETRY));

  rb->nBytes = nBytes;
  rb->status = status;
  return I2C_STATUS_OK;
}


/***************************************************************************
 *  Function to queue a request block and initiate operations.
 * 
 * For the Wire version, this executes synchronously.
 * The read/write/write_P functions return I2C_STATUS_OK always, and the 
 * completion status of the operation is in the request block, as for
 * the non-blocking version.
 ***************************************************************************/
void I2CManagerClass::queueRequest(I2CRB *req) {
  switch (req->operation & OPERATION_MASK) {
    case OPERATION_READ:
      read(req->i2cAddress, req->readBuffer, req->readLen, NULL, 0, req);
      break;
    case OPERATION_SEND:
      write(req->i2cAddress, req->writeBuffer, req->writeLen, req);
      break;
    case OPERATION_SEND_P:
      write_P(req->i2cAddress, req->writeBuffer, req->writeLen, req);
      break;
    case OPERATION_REQUEST:
      read(req->i2cAddress, req->readBuffer, req->readLen, req->writeBuffer, req->writeLen, req);
      break;
  }
}

/***************************************************************************
 *  Loop function, for general background work
 ***************************************************************************/
void I2CManagerClass::loop() {}

#endif