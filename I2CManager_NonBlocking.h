/*
 *  © 2023, Neil McKechnie
 *  © 2022 Paul M Antoine
 *  All rights reserved.
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

// Support for atomic isolation (i.e. a block with interrupts disabled).
// E.g. 
//       ATOMIC_BLOCK() {
//         doSomethingWithInterruptsDisabled();
//       }
// This has the advantage over simple noInterrupts/Interrupts that the
// original interrupt state is restored when the block finishes.
//
// (This should really be defined in an include file somewhere more global, so 
// it can replace use of noInterrupts/interrupts in other parts of DCC-EX.
//
static inline uint8_t _deferInterrupts(void) {
  noInterrupts();
  return 1;
}
static inline void _conditionalEnableInterrupts(bool *wasEnabled) {
  if (*wasEnabled) interrupts();
}
#define ATOMIC_BLOCK(x) \
for (bool _int_saved __attribute__((__cleanup__(_conditionalEnableInterrupts))) \
            =_getInterruptState(),_ToDo=_deferInterrupts(); _ToDo; _ToDo=0)

#if defined(__AVR__) // Nano, Uno, Mega2580, NanoEvery, etc.
  static inline bool _getInterruptState(void) {
    return bitRead(SREG, SREG_I);  // true if enabled, false if disabled
  }
#elif defined(__arm__)  // STM32, SAMD, Teensy
  static inline bool _getInterruptState( void ) {
    uint32_t reg;
    __asm__ __volatile__ ("MRS %0, primask" : "=r" (reg) );
    return !(reg & 1);  // true if interrupts enabled, false otherwise
  }
#else
  #warning "ATOMIC_BLOCK() not defined for this target type, I2C interrupts disabled"
  #define ATOMIC_BLOCK(x) // expand to nothing.
  #ifdef I2C_USE_INTERRUPTS
    #undef I2C_USE_INTERRUPTS
  #endif
#endif


// This module is only compiled if I2C_USE_WIRE is not defined, so undefine it here
// to get intellisense to work correctly.
#if defined(I2C_USE_WIRE)
#undef I2C_USE_WIRE
#endif

enum MuxPhase: uint8_t {
  MuxPhase_OFF = 0,
  MuxPhase_PROLOG,
  MuxPhase_PAYLOAD,
  MuxPhase_EPILOG,
} ;


/***************************************************************************
 * Initialise the I2CManagerAsync class.
 ***************************************************************************/
void I2CManagerClass::_initialise()
{
  queueHead = queueTail = NULL;
  state = I2C_STATE_FREE;
  I2C_init();
  _setClock(_clockSpeed);
}

/***************************************************************************
 *  Set I2C clock speed.  Normally 100000 (Standard) or 400000 (Fast)
 *   on Arduino.  Mega4809 supports 1000000 (Fast+) too.
 *   This function saves the desired clock speed and the startTransaction
 *   function acts on it before a new transaction, to avoid speed changes
 *   during an I2C transaction.
 ***************************************************************************/
void I2CManagerClass::_setClock(unsigned long i2cClockSpeed) {
  pendingClockSpeed = i2cClockSpeed;
}

/***************************************************************************
 * Start an I2C transaction, if the I2C interface is free and
 * there is a queued request to be processed.
 * If there's an I2C clock speed change pending, then implement it before 
 * starting the operation.
 ***************************************************************************/
void I2CManagerClass::startTransaction() {
  ATOMIC_BLOCK() {
    if ((state == I2C_STATE_FREE) && (queueHead != NULL)) {
      state = I2C_STATE_ACTIVE;
      completionStatus = I2C_STATUS_OK;
      // Check for pending clock speed change
      if (pendingClockSpeed) {
        // We're about to start a new I2C transaction, so set clock now.
        I2C_setClock(pendingClockSpeed);
        pendingClockSpeed = 0;
      }
      startTime = micros();
      currentRequest = queueHead;
      rxCount = txCount = 0;

      // Start the I2C process going.
#if defined(I2C_EXTENDED_ADDRESS)
      I2CMux muxNumber = currentRequest->i2cAddress.muxNumber();
      if (muxNumber != I2CMux_None) {
        muxPhase = MuxPhase_PROLOG;
        uint8_t subBus = currentRequest->i2cAddress.subBus();
        muxData[0] = (subBus == SubBus_All) ? 0xff :
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
        deviceAddress = I2C_MUX_BASE_ADDRESS + muxNumber;
        sendBuffer = &muxData[0];
        bytesToSend = 1;
        bytesToReceive = 0;
        operation = OPERATION_SEND;
      } else {
        // Send/receive payload for device only.
        muxPhase = MuxPhase_OFF;
        deviceAddress = currentRequest->i2cAddress;
        sendBuffer = currentRequest->writeBuffer;
        bytesToSend = currentRequest->writeLen;
        receiveBuffer = currentRequest->readBuffer;
        bytesToReceive = currentRequest->readLen;
        operation = currentRequest->operation & OPERATION_MASK;
      } 
#else
      deviceAddress = currentRequest->i2cAddress;
      sendBuffer = currentRequest->writeBuffer;
      bytesToSend = currentRequest->writeLen;
      receiveBuffer = currentRequest->readBuffer;
      bytesToReceive = currentRequest->readLen;
      operation = currentRequest->operation & OPERATION_MASK;
#endif
      I2C_sendStart();
    }
  }
}

/***************************************************************************
 *  Function to queue a request block and initiate operations.
 ***************************************************************************/
void I2CManagerClass::queueRequest(I2CRB *req) {

  if (((req->operation & OPERATION_MASK) == OPERATION_READ) && req->readLen == 0)
    return;  // Ignore null read

  req->status = I2C_STATUS_PENDING;
  req->nextRequest = NULL;
  ATOMIC_BLOCK() {
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
uint8_t I2CManagerClass::write(I2CAddress i2cAddress, const uint8_t *writeBuffer, uint8_t writeLen, I2CRB *req) {
  // Make sure previous request has completed.
  req->wait();
  req->setWriteParams(i2cAddress, writeBuffer, writeLen);
  queueRequest(req);
  return I2C_STATUS_OK;
}

/***************************************************************************
 *  Initiate a write from PROGMEM (flash) to an I2C device (non-blocking operation)
 ***************************************************************************/
uint8_t I2CManagerClass::write_P(I2CAddress i2cAddress, const uint8_t * writeBuffer, uint8_t writeLen, I2CRB *req) {
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
uint8_t I2CManagerClass::read(I2CAddress i2cAddress, uint8_t *readBuffer, uint8_t readLen, 
    const uint8_t *writeBuffer, uint8_t writeLen, I2CRB *req)
{
  // Make sure previous request has completed.
  req->wait();
  req->setRequestParams(i2cAddress, readBuffer, readLen, writeBuffer, writeLen);
  queueRequest(req);
  return I2C_STATUS_OK;
}

/***************************************************************************
 *  Set I2C timeout value in microseconds.  The timeout applies to the entire
 *   I2CRB request, e.g. where a write+read is performed, the timer is not
 *   reset before the read.
 ***************************************************************************/
void I2CManagerClass::setTimeout(unsigned long value) { 
  _timeout = value; 
};

/***************************************************************************
 * checkForTimeout() function, called from isBusy() and wait() to cancel
 * requests that are taking too long to complete.  Such faults
 * may be caused by an I2C wire short for example.
 ***************************************************************************/
void I2CManagerClass::checkForTimeout() {
  ATOMIC_BLOCK() {
    I2CRB *t = queueHead;
    if (state==I2C_STATE_ACTIVE && t!=0 && t==currentRequest && _timeout > 0) {
      // Check for timeout
      int32_t elapsed = micros() - startTime;
      if (elapsed > (int32_t)_timeout) { 
#ifdef DIAG_IO
        //DIAG(F("I2CManager Timeout on %s"), t->i2cAddress.toString());
#endif
        // Excessive time. Dequeue request
        queueHead = t->nextRequest;
        if (!queueHead) queueTail = NULL;
        currentRequest = NULL;
        bytesToReceive = bytesToSend = 0;
        // Post request as timed out.
        t->status = I2C_STATUS_TIMEOUT;
        // Reset TWI interface so it is able to continue
        // Try close and init, not entirely satisfactory but sort of works...
        I2C_close();  // Shutdown and restart twi interface

        // If SDA is stuck low, issue up to 9 clock pulses to attempt to free it.
        pinMode(SCL, INPUT_PULLUP);
        pinMode(SDA, INPUT_PULLUP);
        for (int i=0; !digitalRead(SDA) && i<9; i++) {
          digitalWrite(SCL, 0); 
          pinMode(SCL, OUTPUT);         // Force clock low
          delayMicroseconds(10);        // ... for 5us
          pinMode(SCL, INPUT_PULLUP);   // ... then high
          delayMicroseconds(10);        // ... for 5us (100kHz Clock)
        }
        // Whether that's succeeded or not, now try reinitialising.
        I2C_init();
        _setClock(_clockSpeed);
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
  // Call function to monitor for stuck I2C operations.
  checkForTimeout();
}

/***************************************************************************
 * Interupt handler.  Call I2C state machine, and dequeue request
 * if completed.
 ***************************************************************************/
void I2CManagerClass::handleInterrupt() {

  // Update hardware state machine
  I2C_handleInterrupt();

  // Check if current request has completed.  If there's a current request
  // and state isn't active then state contains the completion status of the request.
  if (state == I2C_STATE_COMPLETED && currentRequest != NULL && currentRequest == queueHead) {
    // Operation has completed.
    if (completionStatus == I2C_STATUS_OK || ++retryCounter > MAX_I2C_RETRIES
      || currentRequest->operation & OPERATION_NORETRY) 
    {
      // Status is OK, or has failed and retry count exceeded, or failed and retries disabled.
#if defined(I2C_EXTENDED_ADDRESS)
      if (muxPhase == MuxPhase_PROLOG ) {
        overallStatus = completionStatus;
        uint8_t rbAddress = currentRequest->i2cAddress.deviceAddress();
        if (completionStatus == I2C_STATUS_OK && rbAddress != 0) {
          // Mux request OK, start handling application request.
          muxPhase = MuxPhase_PAYLOAD;
          deviceAddress = rbAddress;
          sendBuffer = currentRequest->writeBuffer;
          bytesToSend = currentRequest->writeLen;
          receiveBuffer = currentRequest->readBuffer;
          bytesToReceive = currentRequest->readLen;
          operation = currentRequest->operation & OPERATION_MASK;
          state = I2C_STATE_ACTIVE;
          I2C_sendStart();
          return;
        } 
      } else if (muxPhase == MuxPhase_PAYLOAD) {
        // Application request completed, now send epilogue to mux
        overallStatus = completionStatus;
        currentRequest->nBytes = rxCount;  // Save number of bytes read into rb
        if (_muxCount == 1) {
          // Only one MUX, don't need to deselect subbus
          muxPhase = MuxPhase_OFF;
        } else {
          muxPhase = MuxPhase_EPILOG;
          deviceAddress = I2C_MUX_BASE_ADDRESS + currentRequest->i2cAddress.muxNumber();
          muxData[0] = 0x00;
          sendBuffer = &muxData[0];
          bytesToSend = 1;
          bytesToReceive = 0;
          operation = OPERATION_SEND;
          state = I2C_STATE_ACTIVE;
          I2C_sendStart();
          return;
        }
      } else if (muxPhase == MuxPhase_EPILOG) {
        // Epilog finished, ignore completionStatus
        muxPhase = MuxPhase_OFF;
      } else
        overallStatus = completionStatus;
#else
      overallStatus = completionStatus;
      currentRequest->nBytes = rxCount;
#endif
          
      // Remove completed request from head of queue
      I2CRB * t = queueHead;
      if (t == currentRequest) {
        queueHead = t->nextRequest;
        if (!queueHead) queueTail = queueHead;
        t->status = overallStatus;
        
        // I2C state machine is now free for next request
        currentRequest = NULL;
        state = I2C_STATE_FREE;
      }
      retryCounter = 0;
    } else {
      // Status is failed and retry permitted.
      // Retry previous request.
      state = I2C_STATE_FREE;  
    }
  }

  if (state == I2C_STATE_FREE && queueHead != NULL) {
    // Allow any pending interrupts before starting the next request.
    //interrupts();
    // Start next request
    I2CManager.startTransaction();
  }
}

#endif