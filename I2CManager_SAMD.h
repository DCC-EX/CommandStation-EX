/*
 *  © 2022 Paul M Antoine
 *  © 2021, Neil McKechnie
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

#ifndef I2CMANAGER_SAMD_H
#define I2CMANAGER_SAMD_H

#include <Arduino.h>
#include "I2CManager.h"

//#include <avr/io.h>
//#include <avr/interrupt.h>
#include <wiring_private.h>

/***************************************************************************
 *  Interrupt handler.
 *  IRQ handler for SERCOM3 which is the default I2C definition for Arduino Zero
 *  compatible variants such as the Sparkfun SAMD21 Dev Breakout etc.
 *  Later we may wish to allow use of an alternate I2C bus, or more than one I2C
 *  bus on the SAMD architecture 
 ***************************************************************************/
#if defined(I2C_USE_INTERRUPTS) && defined(ARDUINO_SAMD_ZERO)
void SERCOM3_Handler() {
  I2CManagerClass::handleInterrupt();
}
#endif

// Assume SERCOM3 for now - default I2C bus on Arduino Zero and variants of same
Sercom *s = SERCOM3;

/***************************************************************************
 *  Set I2C clock speed register.
 ***************************************************************************/
void I2CManagerClass::I2C_setClock(uint32_t i2cClockSpeed) {

  // Calculate a rise time appropriate to the requested bus speed
  int t_rise;
  if (i2cClockSpeed < 200000L) {
    i2cClockSpeed = 100000L;
    t_rise = 1000;
  } else if (i2cClockSpeed < 800000L) {
    i2cClockSpeed = 400000L;
    t_rise = 300;
  } else if (i2cClockSpeed < 1200000L) {
    i2cClockSpeed = 1000000L;
    t_rise = 120;
  } else {
    i2cClockSpeed = 100000L;
    t_rise = 1000;
  }

  // Disable the I2C master mode and wait for sync
  s->I2CM.CTRLA.bit.ENABLE = 0 ;
  while (s->I2CM.SYNCBUSY.bit.ENABLE != 0);

  // Calculate baudrate - using a rise time appropriate for the speed
  s->I2CM.BAUD.bit.BAUD = SystemCoreClock / (2 * i2cClockSpeed) - 5 - (((SystemCoreClock / 1000000) * t_rise) / (2 * 1000));

  // Enable the I2C master mode and wait for sync
  s->I2CM.CTRLA.bit.ENABLE = 1 ;
  while (s->I2CM.SYNCBUSY.bit.ENABLE != 0);

  // Setting bus idle mode and wait for sync
  s->I2CM.STATUS.bit.BUSSTATE = 1 ;
  while (s->I2CM.SYNCBUSY.bit.SYSOP != 0);

  return;
}

/***************************************************************************
 *  Initialise I2C registers.
 ***************************************************************************/
void I2CManagerClass::I2C_init()
{
  //Setting clock
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCM_SERCOM3_CORE) | // Generic Clock 0 (SERCOM3)
                      GCLK_CLKCTRL_GEN_GCLK0 | // Generic Clock Generator 0 is source
                      GCLK_CLKCTRL_CLKEN ;

  /* Wait for peripheral clock synchronization */
  while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY );

  // Software reset the SERCOM
  s->I2CM.CTRLA.bit.SWRST = 1;

  //Wait both bits Software Reset from CTRLA and SYNCBUSY are equal to 0
  while(s->I2CM.CTRLA.bit.SWRST || s->I2CM.SYNCBUSY.bit.SWRST);

  // Set master mode and enable SCL Clock Stretch mode (stretch after ACK bit)
  s->I2CM.CTRLA.reg =  SERCOM_I2CM_CTRLA_MODE( I2C_MASTER_OPERATION )/* |
                            SERCOM_I2CM_CTRLA_SCLSM*/ ;

  // Enable Smart mode and Quick Command
  s->I2CM.CTRLB.reg =  SERCOM_I2CM_CTRLB_SMEN | SERCOM_I2CM_CTRLB_QCEN;

#if defined(I2C_USE_INTERRUPTS)
  // Setting NVIC
  NVIC_EnableIRQ(SERCOM3_IRQn);
  NVIC_SetPriority (SERCOM3_IRQn, 0);  /* set Priority */

  // Enable all interrupts
  s->I2CM.INTENSET.reg = SERCOM_I2CM_INTENSET_MB | SERCOM_I2CM_INTENSET_SB | SERCOM_I2CM_INTENSET_ERROR;
#endif

  // Calculate baudrate and set default rate for now
  s->I2CM.BAUD.bit.BAUD = SystemCoreClock / ( 2 * I2C_FREQ) - 7 / (2 * 1000);

  // Enable the I2C master mode and wait for sync
  s->I2CM.CTRLA.bit.ENABLE = 1 ;
  while (s->I2CM.SYNCBUSY.bit.ENABLE != 0);

  // Setting bus idle mode and wait for sync
  s->I2CM.STATUS.bit.BUSSTATE = 1 ;
  while (s->I2CM.SYNCBUSY.bit.SYSOP != 0);

  // Set SDA/SCL pins as outputs and enable pullups, at present we assume these are
  // the default ones for SERCOM3 (see assumption above)
  pinPeripheral(PIN_WIRE_SDA, g_APinDescription[PIN_WIRE_SDA].ulPinType);
  pinPeripheral(PIN_WIRE_SCL, g_APinDescription[PIN_WIRE_SCL].ulPinType);

  // Enable the SCL and SDA pins on the sercom: includes increased driver strength,
  // pull-up resistors and pin multiplexer
	PORT->Group[g_APinDescription[PIN_WIRE_SCL].ulPort].PINCFG[g_APinDescription[PIN_WIRE_SCL].ulPin].reg =  
		PORT_PINCFG_DRVSTR | PORT_PINCFG_PULLEN | PORT_PINCFG_PMUXEN;  
  PORT->Group[g_APinDescription[PIN_WIRE_SDA].ulPort].PINCFG[g_APinDescription[PIN_WIRE_SDA].ulPin].reg = 
		PORT_PINCFG_DRVSTR | PORT_PINCFG_PULLEN | PORT_PINCFG_PMUXEN;
}

/***************************************************************************
 *  Initiate a start bit for transmission.
 ***************************************************************************/
void I2CManagerClass::I2C_sendStart() {
  bytesToSend = currentRequest->writeLen;
  bytesToReceive = currentRequest->readLen;

  // We may have initiated a stop bit before this without waiting for it.
  // Wait for stop bit to be sent before sending start.
  while (s->I2CM.STATUS.bit.BUSSTATE == 0x2);

  // If anything to send, initiate write.  Otherwise initiate read.
  if (operation == OPERATION_READ || ((operation == OPERATION_REQUEST) && !bytesToSend))
  {
    // Send start and address with read/write flag or'd in
    s->I2CM.ADDR.bit.ADDR = (currentRequest->i2cAddress << 1) | 1;
  }
  else {
    // Wait while the I2C bus is BUSY
    while (s->I2CM.STATUS.bit.BUSSTATE != 0x1);
    s->I2CM.ADDR.bit.ADDR = (currentRequest->i2cAddress << 1ul) | 0;
  }
}

/***************************************************************************
 *  Initiate a stop bit for transmission (does not interrupt)
 ***************************************************************************/
void I2CManagerClass::I2C_sendStop() {
  s->I2CM.CTRLB.bit.CMD = 3; // Stop condition
}

/***************************************************************************
 *  Close I2C down
 ***************************************************************************/
void I2CManagerClass::I2C_close() {
  I2C_sendStop();
}

/***************************************************************************
 *  Main state machine for I2C, called from interrupt handler or,
 *  if I2C_USE_INTERRUPTS isn't defined, from the I2CManagerClass::loop() function
 *  (and therefore, indirectly, from I2CRB::wait() and I2CRB::isBusy()).
 ***************************************************************************/
void I2CManagerClass::I2C_handleInterrupt() {

  if (s->I2CM.STATUS.bit.ARBLOST) {
    // Arbitration lost, restart
    I2C_sendStart();   // Reinitiate request
  } else if (s->I2CM.STATUS.bit.BUSERR) {
    // Bus error
    state = I2C_STATUS_BUS_ERROR;
  } else if (s->I2CM.INTFLAG.bit.MB) {
    // Master write completed
    if (s->I2CM.STATUS.bit.RXNACK) {
      // Nacked, send stop.
      I2C_sendStop();
      state = I2C_STATUS_NEGATIVE_ACKNOWLEDGE;
    } else if (bytesToSend) {
      // Acked, so send next byte
      if (currentRequest->operation == OPERATION_SEND_P)
        s->I2CM.DATA.bit.DATA = GETFLASH(currentRequest->writeBuffer + (txCount++));
      else
        s->I2CM.DATA.bit.DATA = currentRequest->writeBuffer[txCount++];
      bytesToSend--;
    } else if (bytesToReceive) {
      // Last sent byte acked and no more to send.  Send repeated start, address and read bit.
        s->I2CM.ADDR.bit.ADDR = (currentRequest->i2cAddress << 1) | 1;
    } else {
      // No more data to send/receive. Initiate a STOP condition.
      I2C_sendStop();
      state = I2C_STATUS_OK; // Done
    }
  } else if (s->I2CM.INTFLAG.bit.SB) {
    // Master read completed without errors
    if (bytesToReceive) {
      currentRequest->readBuffer[rxCount++] = s->I2CM.DATA.bit.DATA;  // Store received byte
      bytesToReceive--;
    } else { 
      // Buffer full, issue nack/stop
      s->I2CM.CTRLB.bit.ACKACT = 1;
      I2C_sendStop();
      state = I2C_STATUS_OK;
    }
    if (bytesToReceive) {
      // PMA - I think Smart Mode means we have nothing to do...
      // More bytes to receive, issue ack and start another read
    }
    else
    {
      // Transaction finished, issue NACK and STOP.
      s->I2CM.CTRLB.bit.ACKACT = 1;
      I2C_sendStop();
      state = I2C_STATUS_OK;
    }
  }
}

#endif /* I2CMANAGER_SAMD_H */
