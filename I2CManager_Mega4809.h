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

#ifndef I2CMANAGER_MEGA4809_H
#define I2CMANAGER_MEGA4809_H

#include <Arduino.h>
#include "I2CManager.h"

/***************************************************************************
 *  Set I2C clock speed register.
 ***************************************************************************/
void I2CManagerClass::I2C_setClock(unsigned long i2cClockSpeed) {
  uint16_t t_rise;
  if (i2cClockSpeed < 200000) {
    i2cClockSpeed = 100000;
    t_rise = 1000;
  } else if (i2cClockSpeed < 800000) {
    i2cClockSpeed = 400000;
    t_rise = 300;
  } else if (i2cClockSpeed < 1200000) {
    i2cClockSpeed = 1000000;
    t_rise = 120;
  } else {
    i2cClockSpeed = 100000;
    t_rise = 1000;
  }
  uint32_t baud = (F_CPU_CORRECTED / i2cClockSpeed - F_CPU_CORRECTED / 1000 / 1000
    * t_rise / 1000 - 10) / 2;
  TWI0.MBAUD = (uint8_t)baud;
}

/***************************************************************************
 *  Initialise I2C registers.
 ***************************************************************************/
void I2CManagerClass::I2C_init()
{ 
  pinMode(PIN_WIRE_SDA, INPUT_PULLUP);
  pinMode(PIN_WIRE_SCL, INPUT_PULLUP);
  PORTMUX.TWISPIROUTEA |= TWI_MUX;

#if defined(I2C_USE_INTERRUPTS)
  TWI0.MCTRLA = TWI_RIEN_bm | TWI_WIEN_bm | TWI_ENABLE_bm;
#else
  TWI0.MCTRLA = TWI_ENABLE_bm;
#endif
  I2C_setClock(I2C_FREQ);
  TWI0.MSTATUS = TWI_BUSSTATE_IDLE_gc;
}

/***************************************************************************
 *  Initiate a start bit for transmission, followed by address and R/W
 ***************************************************************************/
void I2CManagerClass::I2C_sendStart() {
  bytesToSend = currentRequest->writeLen;
  bytesToReceive = currentRequest->readLen;

  // If anything to send, initiate write.  Otherwise initiate read.
  if (operation == OPERATION_READ || ((operation == OPERATION_REQUEST) & !bytesToSend))
    TWI0.MADDR = (currentRequest->i2cAddress << 1) | 1;
  else
    TWI0.MADDR = (currentRequest->i2cAddress << 1) | 0;
}

/***************************************************************************
 *  Initiate a stop bit for transmission.
 ***************************************************************************/
void I2CManagerClass::I2C_sendStop() {
  TWI0.MCTRLB = TWI_MCMD_STOP_gc;
}

/***************************************************************************
 *  Close I2C down
 ***************************************************************************/
void I2CManagerClass::I2C_close() {
  I2C_sendStop();
}

/***************************************************************************
 *  Main state machine for I2C, called from interrupt handler.
 ***************************************************************************/
void I2CManagerClass::I2C_handleInterrupt() {
  
  uint8_t currentStatus = TWI0.MSTATUS;

  if (currentStatus & TWI_ARBLOST_bm) {
    // Arbitration lost, restart
    TWI0.MSTATUS = currentStatus; // clear all flags
    I2C_sendStart();   // Reinitiate request
  } else if (currentStatus & TWI_BUSERR_bm) {
    // Bus error
    state = I2C_STATUS_BUS_ERROR;
    TWI0.MSTATUS = currentStatus; // clear all flags
  } else if (currentStatus & TWI_WIF_bm) {
    // Master write completed
    if (currentStatus & TWI_RXACK_bm) {
      // Nacked, send stop.
      TWI0.MCTRLB = TWI_MCMD_STOP_gc;
      state = I2C_STATUS_NEGATIVE_ACKNOWLEDGE;
    } else if (bytesToSend) {
      // Acked, so send next byte
      if (currentRequest->operation == OPERATION_SEND_P)
        TWI0.MDATA = GETFLASH(currentRequest->writeBuffer + (txCount++));
      else
        TWI0.MDATA = currentRequest->writeBuffer[txCount++];
      bytesToSend--;
    } else if (bytesToReceive) {
        // Last sent byte acked and no more to send.  Send repeated start, address and read bit.
        TWI0.MADDR = (currentRequest->i2cAddress << 1) | 1;
    } else {
      // No more data to send/receive. Initiate a STOP condition.
      TWI0.MCTRLB = TWI_MCMD_STOP_gc;
      state = I2C_STATUS_OK;  // Done
    }
  } else if (currentStatus & TWI_RIF_bm) {
    // Master read completed without errors
    if (bytesToReceive) {
      currentRequest->readBuffer[rxCount++] = TWI0.MDATA;  // Store received byte
      bytesToReceive--;
    } else { 
      // Buffer full, issue nack/stop
      TWI0.MCTRLB = TWI_ACKACT_bm | TWI_MCMD_STOP_gc;
      state = I2C_STATUS_OK;
    }
    if (bytesToReceive) {
      // More bytes to receive, issue ack and start another read
      TWI0.MCTRLB = TWI_MCMD_RECVTRANS_gc;
    } else {
      // Transaction finished, issue NACK and STOP.
      TWI0.MCTRLB = TWI_ACKACT_bm | TWI_MCMD_STOP_gc;
      state = I2C_STATUS_OK;
    }
  }
}


/***************************************************************************
 *  Interrupt handler.
 ***************************************************************************/
ISR(TWI0_TWIM_vect) {
  I2CManagerClass::handleInterrupt();
}

#endif
