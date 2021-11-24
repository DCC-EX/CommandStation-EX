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

#ifndef I2CMANAGER_NONBLOCKING_H
#define I2CMANAGER_NONBLOCKING_H

#include <Arduino.h>
#include "I2CManager.h"
#if defined(I2C_USE_INTERRUPTS)
#include <util/atomic.h>
#else
#define ATOMIC_BLOCK(x) 
#define ATOMIC_RESTORESTATE
#endif

// This module is only compiled if I2C_USE_WIRE is not defined, so undefine it here
// to get intellisense to work correctly.
#if defined(I2C_USE_WIRE)
#undef I2C_USE_WIRE
#endif

/***************************************************************************
 * Initialise the I2CManagerAsync class.
 ***************************************************************************/
void I2CManagerClass::_initialise()
{
  queueHead = queueTail = NULL;
  state = I2C_STATE_FREE;
  I2C_init();
}

/***************************************************************************
 *  Set I2C clock speed.  Normally 100000 (Standard) or 400000 (Fast)
 *   on Arduino.  Mega4809 supports 1000000 (Fast+) too.
 ***************************************************************************/
void I2CManagerClass::_setClock(unsigned long i2cClockSpeed) {
  I2C_setClock(i2cClockSpeed);
}

/***************************************************************************
 * Helper function to start operations, if the I2C interface is free and
 * there is a queued request to be processed.
 ***************************************************************************/
void I2CManagerClass::startTransaction() { 
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    if ((state == I2C_STATE_FREE) && (queueHead != NULL)) {
      state = I2C_STATE_ACTIVE;
      currentRequest = queueHead;
      rxCount = txCount = 0;
      // Copy key fields to static data for speed.
      operation = currentRequest->operation;
      // Start the I2C process going.
      I2C_sendStart();
      startTime = micros();
    }
  }
}

/***************************************************************************
 *  Function to queue a request block and initiate operations.
 ***************************************************************************/
void I2CManagerClass::queueRequest(I2CRB *req) {
  req->status = I2C_STATUS_PENDING;
  req->nextRequest = NULL;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    if (!queueTail) 
      queueHead = queueTail = req;  // Only item on queue
    else
      queueTail = queueTail->nextRequest = req; // Add to end
    startTransaction();
  }

}

/***************************************************************************
 *  Initiate a write to an I2C device (non-blocking operation)
 ***************************************************************************/
uint8_t I2CManagerClass::write(uint8_t i2cAddress, const uint8_t *writeBuffer, uint8_t writeLen, I2CRB *req) {
  // Make sure previous request has completed.
  req->wait();
  req->setWriteParams(i2cAddress, writeBuffer, writeLen);
  queueRequest(req);
  return I2C_STATUS_OK;
}

/***************************************************************************
 *  Initiate a write from PROGMEM (flash) to an I2C device (non-blocking operation)
 ***************************************************************************/
uint8_t I2CManagerClass::write_P(uint8_t i2cAddress, const uint8_t * writeBuffer, uint8_t writeLen, I2CRB *req) {
  // Make sure previous request has completed.
  req->wait();
  req->setWriteParams(i2cAddress, writeBuffer, writeLen);
  req->operation = OPERATION_SEND_P;
  queueRequest(req);
  return I2C_STATUS_OK;
}

/***************************************************************************
 *  Initiate a read from the I2C device, optionally preceded by a write 
 *   (non-blocking operation)
 ***************************************************************************/
uint8_t I2CManagerClass::read(uint8_t i2cAddress, uint8_t *readBuffer, uint8_t readLen, 
    const uint8_t *writeBuffer, uint8_t writeLen, I2CRB *req)
{
  // Make sure previous request has completed.
  req->wait();
  req->setRequestParams(i2cAddress, readBuffer, readLen, writeBuffer, writeLen);
  queueRequest(req);
  return I2C_STATUS_OK;
}

/***************************************************************************
 * checkForTimeout() function, called from isBusy() and wait() to cancel
 * requests that are taking too long to complete.
 * This function doesn't fully work as intended so is not currently called.
 * Instead we check for an I2C hang-up and report an error from
 * I2CRB::wait(), but we aren't able to recover from the hang-up.  Such faults
 * may be caused by an I2C wire short for example.
 ***************************************************************************/
void I2CManagerClass::checkForTimeout() {
  unsigned long currentMicros = micros();
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    I2CRB *t = queueHead;
    if (state==I2C_STATE_ACTIVE && t!=0 && t==currentRequest && timeout > 0) {
      // Check for timeout
      if (currentMicros - startTime > timeout) { 
        // Excessive time. Dequeue request
        queueHead = t->nextRequest;
        if (!queueHead) queueTail = NULL;
        currentRequest = NULL;
        // Post request as timed out.
        t->status = I2C_STATUS_TIMEOUT;
        // Reset TWI interface so it is able to continue
        // Try close and init, not entirely satisfactory but sort of works...
        I2C_close();  // Shutdown and restart twi interface
        I2C_init();
        state = I2C_STATE_FREE;
        
        // Initiate next queued request if any.
        startTransaction();
      }
    }
  }
}

/***************************************************************************
 *  Loop function, for general background work
 ***************************************************************************/
void I2CManagerClass::loop() {
#if !defined(I2C_USE_INTERRUPTS)
  handleInterrupt();
#endif
  // Timeout is now reported in I2CRB::wait(), not here.
  // I've left the code, commented out, as a reminder to look at this again
  // in the future.
  //checkForTimeout();
}

/***************************************************************************
 * Interupt handler.  Call I2C state machine, and dequeue request
 * if completed.
 ***************************************************************************/
void I2CManagerClass::handleInterrupt() {

  // Update hardware state machine
  I2C_handleInterrupt();

  // Enable interrupts to minimise effect on other interrupt code
  interrupts();

  // Check if current request has completed.  If there's a current request
  // and state isn't active then state contains the completion status of the request.
  if (state != I2C_STATE_ACTIVE && currentRequest != NULL) {
    // Remove completed request from head of queue
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      I2CRB * t = queueHead;
      if (t == queueHead) {
        queueHead = t->nextRequest;
        if (!queueHead) queueTail = queueHead;
        t->nBytes = rxCount;
        t->status = state;
        
        // I2C state machine is now free for next request
        currentRequest = NULL;
        state = I2C_STATE_FREE;

        // Start next request (if any)
        I2CManager.startTransaction();
      }
    }
  }
}

// Fields in I2CManager class specific to Non-blocking implementation.
I2CRB * volatile I2CManagerClass::queueHead = NULL;
I2CRB * volatile I2CManagerClass::queueTail = NULL;
I2CRB * volatile I2CManagerClass::currentRequest = NULL;
volatile uint8_t I2CManagerClass::state = I2C_STATE_FREE;
volatile uint8_t I2CManagerClass::txCount;
volatile uint8_t I2CManagerClass::rxCount;
volatile uint8_t I2CManagerClass::operation;
volatile uint8_t I2CManagerClass::bytesToSend;
volatile uint8_t I2CManagerClass::bytesToReceive;
volatile unsigned long I2CManagerClass::startTime;
unsigned long I2CManagerClass::timeout = 0;

#endif