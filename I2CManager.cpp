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
#elif defined(ARDUINO_ARCH_SAMD)
#include "I2CManager_NonBlocking.h"
#include "I2CManager_SAMD.h"      // SAMD21 for now... SAMD51 as well later
#elif defined(ARDUINO_ARCH_STM32)
#include "I2CManager_NonBlocking.h"
#include "I2CManager_STM32.h"      // STM32F411RE for now... more later
#else
#define I2C_USE_WIRE
#include "I2CManager_Wire.h"      // Other platforms
#endif


// Helper function for listing device types
static const FSH * guessI2CDeviceType(uint8_t address) {
  if (address >= 0x20 && address <= 0x26)
    return F("GPIO Expander");
  else if (address == 0x27)
    return F("GPIO Expander or LCD Display");
  else if (address == 0x29)
    return F("Time-of-flight sensor");
  else if (address >= 0x3c && address <= 0x3d)
    return F("OLED Display");
  else if (address >= 0x48 && address <= 0x57) // SC16IS752x UART detection
    return F("SC16IS75x UART");
  else if (address >= 0x48 && address <= 0x4f)
    return F("Analogue Inputs or PWM");
  else if (address >= 0x40 && address <= 0x4f)
    return F("PWM");
  else if (address >= 0x50 && address <= 0x5f) 
    return F("EEPROM"); 
  else if (address == 0x68) 
    return F("Real-time clock"); 
  else if (address >= 0x70 && address <= 0x77)
    return F("I2C Mux");
  else
    return F("?");
}

// If not already initialised, initialise I2C
void I2CManagerClass::begin(void) {
  if (!_beginCompleted) {
    _beginCompleted = true;

    // Check for short-circuit or floating lines (no pull-up) on I2C before enabling I2C
    const FSH *message = F("WARNING: Check I2C %S line for short/pullup");
    pinMode(SDA, INPUT);
    if (!digitalRead(SDA))
      DIAG(message, F("SDA"));
    pinMode(SCL, INPUT);
    if (!digitalRead(SCL))
      DIAG(message, F("SCL"));

    // Now initialise I2C
    _initialise();

    #if defined(I2C_USE_WIRE)
    DIAG(F("I2CManager: Using Wire library"));
    #endif

    // Probe and list devices.  Use standard mode 
    //  (clock speed 100kHz) for best device compatibility.
    _setClock(100000);
    uint32_t originalTimeout = _timeout;
    setTimeout(1000);       // use 1ms timeout for probes

  #if defined(I2C_EXTENDED_ADDRESS)
    // First count the multiplexers and switch off all subbuses
    _muxCount = 0;
    for (uint8_t muxNo=I2CMux_0; muxNo <= I2CMux_7; muxNo++) {
      if (I2CManager.muxSelectSubBus({(I2CMux)muxNo, SubBus_None})==I2C_STATUS_OK)
        _muxCount++;
    }
  #endif

    // Enumerate devices that are visible
    bool found = false;
    for (uint8_t addr=0x08; addr<0x78; addr++) {
      if (exists(addr)) {
        found = true; 
        DIAG(F("I2C Device found at 0x%x, %S?"), addr, guessI2CDeviceType(addr));
      }
    }

#if defined(I2C_EXTENDED_ADDRESS)
    // Enumerate all I2C devices that are connected via multiplexer, 
    // i.e. that respond when only one multiplexer has one subBus enabled
    // and the device doesn't respond when the mux subBus is disabled.
    // If any probes time out, then assume that the subbus is dead and
    // don't do any more on that subbus.
    for (uint8_t muxNo=I2CMux_0; muxNo <= I2CMux_7; muxNo++) {
      uint8_t muxAddr = I2C_MUX_BASE_ADDRESS + muxNo;
      if (exists(muxAddr)) {
        // Select Mux Subbus
        for (uint8_t subBus=0; subBus<=SubBus_No; subBus++) {
          muxSelectSubBus({(I2CMux)muxNo, (I2CSubBus)subBus});
          for (uint8_t addr=0x08; addr<0x78; addr++) {
            uint8_t status = checkAddress(addr);
            if (status == I2C_STATUS_OK) {
              // De-select subbus
              muxSelectSubBus({(I2CMux)muxNo, SubBus_None});
              if (!exists(addr)) {
                // Device responds when subbus selected but not when
                // subbus disabled - ergo it must be on subbus!
                found = true; 
                DIAG(F("I2C Device found at {I2CMux_%d,SubBus_%d,0x%x}, %S?"), 
                  muxNo, subBus, addr, guessI2CDeviceType(addr));
              }
              // Re-select subbus
              muxSelectSubBus({(I2CMux)muxNo, (I2CSubBus)subBus});
            } else if (status == I2C_STATUS_TIMEOUT) {
              // Bus stuck, skip to next one.
              break;
            }
          }
        }
        // Deselect all subBuses for this mux.  Otherwise its devices will continue to
        // respond when other muxes are being probed.
        I2CManager.muxSelectSubBus({(I2CMux)muxNo, SubBus_None});  // Deselect Mux
      } 
    }
#endif
    if (!found) DIAG(F("No I2C Devices found"));
    _setClock(_clockSpeed);
    setTimeout(originalTimeout);      // set timeout back to original
  }
}

// Set clock speed to the lowest requested one. If none requested,
//  the Wire default is 100kHz.
void I2CManagerClass::setClock(uint32_t speed) {
  if (speed < _clockSpeed && !_clockSpeedFixed) {
    _clockSpeed = speed;
    DIAG(F("I2C clock speed set to %l Hz"), _clockSpeed);
  }
  _setClock(_clockSpeed);
}

// Force clock speed to that specified.
void I2CManagerClass::forceClock(uint32_t speed) {
  _clockSpeed = speed;
  _clockSpeedFixed = true;
  _setClock(_clockSpeed);
  DIAG(F("I2C clock speed forced to %l Hz"), _clockSpeed);
}

// Check if specified I2C address is responding (blocking operation)
// Returns I2C_STATUS_OK (0) if OK, or error code.
// Suppress retries.  If it doesn't respond first time it's out of the running.
uint8_t I2CManagerClass::checkAddress(I2CAddress address) {
  I2CRB rb;
  rb.setWriteParams(address, NULL, 0);
  rb.suppressRetries(true);
  queueRequest(&rb);
  return rb.wait();
}


/***************************************************************************
 *  Write a transmission to I2C using a list of data (blocking operation)
 ***************************************************************************/
uint8_t I2CManagerClass::write(I2CAddress address, uint8_t nBytes, ...) {
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
uint8_t I2CManagerClass::write(I2CAddress i2cAddress, const uint8_t writeBuffer[], uint8_t writeLen) {
  I2CRB req;
  uint8_t status = write(i2cAddress, writeBuffer, writeLen, &req);
  return finishRB(&req, status);
}

/***************************************************************************
 *  Initiate a write from PROGMEM (flash) to an I2C device (blocking operation)
 ***************************************************************************/
uint8_t I2CManagerClass::write_P(I2CAddress i2cAddress, const uint8_t * data, uint8_t dataLen) {
  I2CRB req;
  uint8_t status = write_P(i2cAddress, data, dataLen, &req);
  return finishRB(&req, status);
}

/***************************************************************************
 *  Initiate a write (optional) followed by a read from the I2C device (blocking operation)
 ***************************************************************************/
uint8_t I2CManagerClass::read(I2CAddress i2cAddress, uint8_t *readBuffer, uint8_t readLen, 
    const uint8_t *writeBuffer, uint8_t writeLen)
{
  I2CRB req;
  uint8_t status = read(i2cAddress, readBuffer, readLen, writeBuffer, writeLen, &req);
  return finishRB(&req, status);
}

/***************************************************************************
 *  Overload of read() to allow command to be specified as a series of bytes (blocking operation)
 ***************************************************************************/
uint8_t I2CManagerClass::read(I2CAddress address, uint8_t readBuffer[], uint8_t readSize, 
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
    case I2C_STATUS_TIMEOUT: return F("I2C bus timeout");
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

// Buffer for conversion of I2CAddress to char*.
/* static */ char I2CAddress::addressBuffer[30];

/////////////////////////////////////////////////////////////////////////////
// Helper functions associated with I2C Request Block
/////////////////////////////////////////////////////////////////////////////

/***************************************************************************
 *  Block waiting for request to complete, and return completion status.
 *  Timeout monitoring is performed in the I2CManager.loop() function.
 ***************************************************************************/
uint8_t I2CRB::wait() {
  while (status==I2C_STATUS_PENDING) {
    I2CManager.loop();
  };
  return status;
}

/***************************************************************************
 *  Check whether request is still in progress.
 *  Timeout monitoring is performed in the I2CManager.loop() function.
 ***************************************************************************/
bool I2CRB::isBusy() {
  if (status==I2C_STATUS_PENDING) {
    I2CManager.loop();
    return true;
  } else
    return false;
}

/***************************************************************************
 *  Helper functions to fill the I2CRequest structure with parameters.
 ***************************************************************************/
void I2CRB::setReadParams(I2CAddress i2cAddress, uint8_t *readBuffer, uint8_t readLen) {
  this->i2cAddress = i2cAddress;
  this->writeLen = 0;
  this->readBuffer = readBuffer;
  this->readLen = readLen;
  this->operation = OPERATION_READ;
  this->status = I2C_STATUS_OK;
}

void I2CRB::setRequestParams(I2CAddress i2cAddress, uint8_t *readBuffer, uint8_t readLen, 
    const uint8_t *writeBuffer, uint8_t writeLen) {
  this->i2cAddress = i2cAddress;
  this->writeBuffer = writeBuffer;
  this->writeLen = writeLen;
  this->readBuffer = readBuffer;
  this->readLen = readLen;
  this->operation = OPERATION_REQUEST;
  this->status = I2C_STATUS_OK;
}

void I2CRB::setWriteParams(I2CAddress i2cAddress, const uint8_t *writeBuffer, uint8_t writeLen) {
  this->i2cAddress = i2cAddress;
  this->writeBuffer = writeBuffer;
  this->writeLen = writeLen;
  this->readLen = 0;
  this->operation = OPERATION_SEND;
  this->status = I2C_STATUS_OK;
}

void I2CRB::suppressRetries(bool suppress) {
  if (suppress)
    this->operation |= OPERATION_NORETRY;
  else
    this->operation &= ~OPERATION_NORETRY;
}


// Helper function for converting a uint8_t to four characters (e.g. 0x23).
void I2CAddress::toHex(const uint8_t value, char *buffer) {
  char *ptr = buffer;
  // Just display hex value, two digits.
  *ptr++ = '0';
  *ptr++ = 'x';
  uint8_t bits = (value >> 4) & 0xf;
  *ptr++ = bits > 9 ? bits-10+'a' : bits+'0';
  bits = value & 0xf;
  *ptr++ = bits > 9 ? bits-10+'a' : bits+'0';
}

#if !defined(I2C_EXTENDED_ADDRESS) 

/* static */ bool I2CAddress::_addressWarningDone = false;

#endif
