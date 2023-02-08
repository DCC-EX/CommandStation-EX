/*
 *  © 2022-23 Paul M Antoine
 *  © 2023, Neil McKechnie
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

#ifndef I2CMANAGER_STM32_H
#define I2CMANAGER_STM32_H

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
#if defined(I2C_USE_INTERRUPTS) && defined(ARDUINO_ARCH_STM32)
void I2C1_IRQHandler() {
  I2CManagerClass::handleInterrupt();
}
#endif

// Assume I2C1 for now - default I2C bus on Nucleo-F411RE and likely Nucleo-64 variants
I2C_TypeDef *s = I2C1;

/***************************************************************************
 *  Set I2C clock speed register.  This should only be called outside of
 *  a transmission.  The I2CManagerClass::_setClock() function ensures 
 *  that it is only called at the beginning of an I2C transaction.
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
  // s->I2CM.CTRLA.bit.ENABLE = 0 ;
  // while (s->I2CM.SYNCBUSY.bit.ENABLE != 0);

  // Calculate baudrate - using a rise time appropriate for the speed
  // s->I2CM.BAUD.bit.BAUD = SystemCoreClock / (2 * i2cClockSpeed) - 5 - (((SystemCoreClock / 1000000) * t_rise) / (2 * 1000));

  // Enable the I2C master mode and wait for sync
  // s->I2CM.CTRLA.bit.ENABLE = 1 ;
  // while (s->I2CM.SYNCBUSY.bit.ENABLE != 0);

  // Setting bus idle mode and wait for sync
  // s->I2CM.STATUS.bit.BUSSTATE = 1 ;
  // while (s->I2CM.SYNCBUSY.bit.SYSOP != 0);
}

/***************************************************************************
 *  Initialise I2C registers.
 ***************************************************************************/
void I2CManagerClass::I2C_init()
{
  //Setting up the clocks
  RCC->APB1ENR |= (1<<21);  // Enable I2C CLOCK
  RCC->AHB1ENR |= (1<<1);   // Enable GPIOB CLOCK for PB8/PB9
  // Standard I2C pins are SCL on PB8 and SDA on PB9
  // Bits (17:16)= 1:0 --> Alternate Function for Pin PB8;
  // Bits (19:18)= 1:0 --> Alternate Function for Pin PB9
  GPIOB->MODER |= (2<<(8*2)) | (2<<(9*2));    // PB8 and PB9 set to ALT function
  GPIOB->OTYPER |= (1<<8) | (1<<9);           // PB8 and PB9 set to open drain output capability
  GPIOB->OSPEEDR |= (3<<(8*2)) | (3<<(9*2));  // PB8 and PB9 set to High Speed mode
  GPIOB->PUPDR |= (1<<(8*2)) | (1<<(9*2));    // PB8 and PB9 set to pull-up capability
  // Alt Function High register routing pins PB8 and PB9 for I2C1:
  // Bits (3:2:1:0) = 0:1:0:0 --> AF4 for pin PB8
  // Bits (7:6:5:4) = 0:1:0:0 --> AF4 for pin PB9
  GPIOB->AFR[1] |= (4<<0) | (4<<4);           // PB8 on low nibble, PB9 on next nibble up

  // Software reset the I2C peripheral
  s->CR1 |= (1<<15);  // reset the I2C
  s->CR1 &= ~(1<<15);  // Normal operation

  // Program the peripheral input clock in I2C_CR2 Register in order to generate correct timings
  s->CR2 |= (16<<0);  // PCLK1 FREQUENCY in MHz

  // Configure the Clock Control Register for 100KHz SCL frequency
  // Bit 15: I2C Master mode, 0=standard, 1=Fast Mode
  // Bit 14: Duty, fast mode duty cycle
  // Bit 11-0: FREQR = 16MHz => TPCLK1 = 62.5ns, so CCR divisor must be 0x50 (80 * 62.5ns = 5000ns)
  s->CCR = 0x0050;

  // Configure the rise time register - max allowed in 1000ns
  s->TRISE = 0x0011; // 1000 ns / 62.5 ns = 16 + 1

#if defined(I2C_USE_INTERRUPTS)
  // Setting NVIC
  NVIC_SetPriority(I2C1_EV_IRQn, 1);  // Match default priorities
  NVIC_EnableIRQ(I2C1_EV_IRQn);

  // CR2 Interrupt Settings
  // Bit 15-13: reserved
  // Bit 12: LAST - DMA last transfer
  // Bit 11: DMAEN - DMA enable
  // Bit 10: ITBUFEN - Buffer interrupt enable
  // Bit 9: ITEVTEN - Event interrupt enable
  // Bit 8: ITERREN - Error interrupt enable
  // Bit 7-6: reserved
  // Bit 5-0: FREQ - Peripheral clock frequency (max 50MHz)
  // Enable all interrupts
  s->CR2 |= 0x0700;
#endif

  // Calculate baudrate and set default rate for now

  // Enable the I2C master mode and wait for sync

  // Setting bus idle mode and wait for sync
}

/***************************************************************************
 *  Initiate a start bit for transmission.
 ***************************************************************************/
void I2CManagerClass::I2C_sendStart() {

  // Set counters here in case this is a retry.
  bytesToSend = currentRequest->writeLen;
  bytesToReceive = currentRequest->readLen;

  // On a single-master I2C bus, the start bit won't be sent until the bus 
  // state goes to IDLE so we can request it without waiting.  On a 
  // multi-master bus, the bus may be BUSY under control of another master, 
  // in which case we can avoid some arbitration failures by waiting until
  // the bus state is IDLE.  We don't do that here.

  // If anything to send, initiate write.  Otherwise initiate read.
  if (operation == OPERATION_READ || ((operation == OPERATION_REQUEST) && !bytesToSend))
  {
    // Send start and address with read flag (1) or'd in
    // s->I2CM.ADDR.bit.ADDR = (currentRequest->i2cAddress << 1) | 1;
  }
  else {
    // Send start and address with write flag (0) or'd in
    // s->I2CM.ADDR.bit.ADDR = (currentRequest->i2cAddress << 1ul) | 0;
  }
}

/***************************************************************************
 *  Initiate a stop bit for transmission (does not interrupt)
 ***************************************************************************/
void I2CManagerClass::I2C_sendStop() {
  // s->I2CM.CTRLB.bit.CMD = 3; // Stop condition
}

/***************************************************************************
 *  Close I2C down
 ***************************************************************************/
void I2CManagerClass::I2C_close() {
  I2C_sendStop();
  // Disable the I2C master mode and wait for sync
  // s->I2CM.CTRLA.bit.ENABLE = 0 ;
  // Wait for up to 500us only.
  unsigned long startTime = micros();
  // while (s->I2CM.SYNCBUSY.bit.ENABLE != 0) {
  //   if (micros() - startTime >= 500UL) break;
  // }
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
    if (bytesToReceive == 1) {
      s->I2CM.CTRLB.bit.ACKACT = 1;  // NAK final byte
      I2C_sendStop();  // send stop
      currentRequest->readBuffer[rxCount++] = s->I2CM.DATA.bit.DATA;  // Store received byte
      bytesToReceive = 0;
      state = I2C_STATUS_OK; // done
    } else if (bytesToReceive) {
      s->I2CM.CTRLB.bit.ACKACT = 0;  // ACK all but final byte
      currentRequest->readBuffer[rxCount++] = s->I2CM.DATA.bit.DATA;  // Store received byte
      bytesToReceive--;
    }
  }
}

#endif /* I2CMANAGER_STM32_H */
