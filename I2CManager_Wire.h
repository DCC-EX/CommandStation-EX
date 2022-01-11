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

/***************************************************************************
 *  Initialise I2C interface software
 ***************************************************************************/
void I2CManagerClass::_initialise() {
  Wire.begin();
}

/***************************************************************************
 *  Set I2C clock speed.  Normally 100000 (Standard) or 400000 (Fast)
 *   on Arduino.  Mega4809 supports 1000000 (Fast+) too.
 ***************************************************************************/
void I2CManagerClass::_setClock(unsigned long i2cClockSpeed) {
  Wire.setClock(i2cClockSpeed);
}

/***************************************************************************
 *  Initiate a write to an I2C device (blocking operation on Wire)
 ***************************************************************************/
uint8_t I2CManagerClass::write(uint8_t address, const uint8_t buffer[], uint8_t size, I2CRB *rb) {
  Wire.beginTransmission(address);
  if (size > 0) Wire.write(buffer, size);
  rb->status = Wire.endTransmission();
  return I2C_STATUS_OK;
}

/***************************************************************************
 *  Initiate a write from PROGMEM (flash) to an I2C device (blocking operation on Wire)
 ***************************************************************************/
uint8_t I2CManagerClass::write_P(uint8_t address, const uint8_t buffer[], uint8_t size, I2CRB *rb) {
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
uint8_t I2CManagerClass::read(uint8_t address, uint8_t readBuffer[], uint8_t readSize,
                              const uint8_t writeBuffer[], uint8_t writeSize, I2CRB *rb)
{
  uint8_t status = I2C_STATUS_OK;
  uint8_t nBytes = 0;
  if (writeSize > 0) {
    Wire.beginTransmission(address);
    Wire.write(writeBuffer, writeSize);
    status = Wire.endTransmission(false); // Don't free bus yet
  }
  if (status == I2C_STATUS_OK) {
    Wire.requestFrom(address, (size_t)readSize);
    while (Wire.available() && nBytes < readSize) 
      readBuffer[nBytes++] = Wire.read();
    if (nBytes < readSize) status = I2C_STATUS_TRUNCATED;
  }
  rb->nBytes = nBytes;
  rb->status = status;
  return I2C_STATUS_OK;
}

/***************************************************************************
 *  Function to queue a request block and initiate operations.
 * 
 * For the Wire version, this executes synchronously, but the status is
 * returned in the I2CRB as for the asynchronous version.
 ***************************************************************************/
void I2CManagerClass::queueRequest(I2CRB *req) {
  switch (req->operation) {
    case OPERATION_READ:
      req->status = read(req->i2cAddress, req->readBuffer, req->readLen, NULL, 0, req);
      break;
    case OPERATION_SEND:
      req->status = write(req->i2cAddress, req->writeBuffer, req->writeLen, req);
      break;
    case OPERATION_SEND_P:
      req->status = write_P(req->i2cAddress, req->writeBuffer, req->writeLen, req);
      break;
    case OPERATION_REQUEST:
      req->status = read(req->i2cAddress, req->readBuffer, req->readLen, req->writeBuffer, req->writeLen, req);
      break;
  }
}

/***************************************************************************
 *  Loop function, for general background work
 ***************************************************************************/
void I2CManagerClass::loop() {}

// Loop function
void I2CManagerClass::checkForTimeout() {}


#endif