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
#include "I2CManager_NonBlocking.h"   // to satisfy intellisense

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
  I2CManager.handleInterrupt();
}
#endif

// Assume I2C1 for now - default I2C bus on Nucleo-F411RE and likely Nucleo-64 variants
I2C_TypeDef *s = I2C1;
#define I2C_IRQn  I2C1_EV_IRQn
#define I2C_BUSFREQ 16

// I2C SR1 Status Register #1 bit definitions for convenience
// #define I2C_SR1_SMBALERT  (1<<15)   // SMBus alert
// #define I2C_SR1_TIMEOUT   (1<<14)   // Timeout of Tlow error
// #define I2C_SR1_PECERR    (1<<12)   // PEC error in reception
// #define I2C_SR1_OVR       (1<<11)   // Overrun/Underrun error
// #define I2C_SR1_AF        (1<<10)   // Acknowledge failure
// #define I2C_SR1_ARLO      (1<<9)    // Arbitration lost (master mode)
// #define I2C_SR1_BERR      (1<<8)    // Bus error (misplaced start or stop condition)
// #define I2C_SR1_TxE       (1<<7)    // Data register empty on transmit
// #define I2C_SR1_RxNE      (1<<6)    // Data register not empty on receive
// #define I2C_SR1_STOPF     (1<<4)    // Stop detection (slave mode)
// #define I2C_SR1_ADD10     (1<<3)    // 10 bit header sent
// #define I2C_SR1_BTF       (1<<2)    // Byte transfer finished - data transfer done
// #define I2C_SR1_ADDR      (1<<1)    // Address sent (master) or matched (slave)
// #define I2C_SR1_SB        (1<<0)    // Start bit (master mode) 1=start condition generated

// I2C CR1 Control Register #1 bit definitions for convenience
// #define I2C_CR1_SWRST     (1<<15)   // Software reset - places peripheral under reset
// #define I2C_CR1_ALERT     (1<<13)   // SMBus alert assertion
// #define I2C_CR1_PEC       (1<<12)   // Packet Error Checking transfer in progress
// #define I2C_CR1_POS       (1<<11)   // Acknowledge/PEC Postion (for data reception in PEC mode)
// #define I2C_CR1_ACK       (1<<10)   // Acknowledge enable - ACK returned after byte is received (address or data)
// #define I2C_CR1_STOP      (1<<9)    // STOP generated
// #define I2C_CR1_START     (1<<8)    // START generated
// #define I2C_CR1_NOSTRETCH (1<<7)    // Clock stretching disable (slave mode)
// #define I2C_CR1_ENGC      (1<<6)    // General call (broadcast) enable (address 00h is ACKed)
// #define I2C_CR1_ENPEC     (1<<5)    // PEC Enable
// #define I2C_CR1_ENARP     (1<<4)    // ARP enable (SMBus)
// #define I2C_CR1_SMBTYPE   (1<<3)    // SMBus type, 1=host, 0=device
// #define I2C_CR1_SMBUS     (1<<1)    // SMBus mode, 1=SMBus, 0=I2C
// #define I2C_CR1_PE        (1<<0)    // I2C Peripheral enable

/***************************************************************************
 *  Set I2C clock speed register.  This should only be called outside of
 *  a transmission.  The I2CManagerClass::_setClock() function ensures 
 *  that it is only called at the beginning of an I2C transaction.
 ***************************************************************************/
void I2CManagerClass::I2C_setClock(uint32_t i2cClockSpeed) {

  // Calculate a rise time appropriate to the requested bus speed
  // Use 10x the rise time spec to enable integer divide of 62.5ns clock period
  uint16_t t_rise;
  uint32_t ccr_freq;
  if (i2cClockSpeed < 200000L) {
    // i2cClockSpeed = 100000L;
    t_rise = 0x11;  // (1000ns /62.5ns) + 1;
  }
  else if (i2cClockSpeed < 800000L)
  {
    i2cClockSpeed = 400000L;
    t_rise = 0x06;  // (300ns / 62.5ns) + 1;
    // } else if (i2cClockSpeed < 1200000L) {
    //   i2cClockSpeed = 1000000L;
    //   t_rise = 120;
  }
  else
  {
    i2cClockSpeed = 100000L;
    t_rise = 0x11;  // (1000ns /62.5ns) + 1;
  }

  // Enable the I2C master mode
  s->CR1 &= ~(I2C_CR1_PE);  // Enable I2C
  // Software reset the I2C peripheral
  // s->CR1 |= I2C_CR1_SWRST;  // reset the I2C
  // Release reset
  // s->CR1 &= ~(I2C_CR1_SWRST);  // Normal operation

  // Calculate baudrate - using a rise time appropriate for the speed
  ccr_freq = I2C_BUSFREQ * 1000000 / i2cClockSpeed / 2;

  // Bit 15: I2C Master mode, 0=standard, 1=Fast Mode
  // Bit 14: Duty, fast mode duty cycle
  // Bit 11-0: FREQR = 16MHz => TPCLK1 = 62.5ns, so CCR divisor must be 0x50 (80 * 62.5ns = 5000ns)
  s->CCR = (uint16_t)ccr_freq;

  // Configure the rise time register
  s->TRISE = t_rise;  // 1000 ns / 62.5 ns = 16 + 1

  // Enable the I2C master mode
  s->CR1 |= I2C_CR1_PE;  // Enable I2C
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
  s->CR1 |= I2C_CR1_SWRST;  // reset the I2C
  s->CR1 &= ~(I2C_CR1_SWRST);  // Normal operation

  // Program the peripheral input clock in CR2 Register in order to generate correct timings
  s->CR2 |= I2C_BUSFREQ;  // PCLK1 FREQUENCY in MHz

#if defined(I2C_USE_INTERRUPTS)
  // Setting NVIC
  NVIC_SetPriority(I2C_IRQn, 1);  // Match default priorities
  NVIC_EnableIRQ(I2C_IRQn);

  // CR2 Interrupt Settings
  // Bit 15-13: reserved
  // Bit 12: LAST - DMA last transfer
  // Bit 11: DMAEN - DMA enable
  // Bit 10: ITBUFEN - Buffer interrupt enable
  // Bit 9: ITEVTEN - Event interrupt enable
  // Bit 8: ITERREN - Error interrupt enable
  // Bit 7-6: reserved
  // Bit 5-0: FREQ - Peripheral clock frequency (max 50MHz)
  // s->CR2 |= 0x0700;   // Enable Buffer, Event and Error interrupts
  s->CR2 |= 0x0300;   // Enable Event and Error interrupts
#endif

  // Calculate baudrate and set default rate for now
  // Configure the Clock Control Register for 100KHz SCL frequency
  // Bit 15: I2C Master mode, 0=standard, 1=Fast Mode
  // Bit 14: Duty, fast mode duty cycle
  // Bit 11-0: FREQR = 16MHz => TPCLK1 = 62.5ns, so CCR divisor must be 0x50 (80 * 62.5ns = 5000ns)
  s->CCR = 0x0050;

  // Configure the rise time register - max allowed in 1000ns
  s->TRISE = 0x0011; // 1000 ns / 62.5 ns = 16 + 1

  // Enable the I2C master mode
  s->CR1 |= I2C_CR1_PE;  // Enable I2C
  // Setting bus idle mode and wait for sync
}

/***************************************************************************
 *  Initiate a start bit for transmission.
 ***************************************************************************/
void I2CManagerClass::I2C_sendStart() {

  // Set counters here in case this is a retry.
  rxCount = txCount = 0;
  uint8_t temp;

  // On a single-master I2C bus, the start bit won't be sent until the bus 
  // state goes to IDLE so we can request it without waiting.  On a 
  // multi-master bus, the bus may be BUSY under control of another master, 
  // in which case we can avoid some arbitration failures by waiting until
  // the bus state is IDLE.  We don't do that here.

  // If anything to send, initiate write.  Otherwise initiate read.
  if (operation == OPERATION_READ || ((operation == OPERATION_REQUEST) && !bytesToSend))
  {
    // Send start for read operation
    s->CR1 |= I2C_CR1_ACK;  // Enable the ACK
    s->CR1 |= I2C_CR1_START;  // Generate START
    // Send address with read flag (1) or'd in
    s->DR = (deviceAddress << 1) | 1;  //  send the address
    while (!(s->SR1 && I2C_SR1_ADDR));  // wait for ADDR bit to set
    // Special case for 1 byte reads!
    if (bytesToReceive == 1)
    {
      s->CR1 &= ~I2C_CR1_ACK;            // clear the ACK bit 
		  temp = I2C1->SR1 | I2C1->SR2;  // read SR1 and SR2 to clear the ADDR bit.... EV6 condition
		  s->CR1 |= I2C_CR1_STOP;              // Stop I2C
    }
    else
      temp = s->SR1 | s->SR2;        // read SR1 and SR2 to clear the ADDR bit
  }
  else {
    // Send start for write operation
    s->CR1 |= I2C_CR1_ACK;  // Enable the ACK
    s->CR1 |= I2C_CR1_START;  // Generate START
    // Send address with write flag (0) or'd in
    s->DR = (deviceAddress << 1) | 0;  //  send the address
    while (!(s->SR1 && I2C_SR1_ADDR));  // wait for ADDR bit to set
    temp = s->SR1 | s->SR2;  // read SR1 and SR2 to clear the ADDR bit
  }
}

/***************************************************************************
 *  Initiate a stop bit for transmission (does not interrupt)
 ***************************************************************************/
void I2CManagerClass::I2C_sendStop() {
  s->CR1 |= I2C_CR1_STOP;              // Stop I2C
}

/***************************************************************************
 *  Close I2C down
 ***************************************************************************/
void I2CManagerClass::I2C_close() {
  I2C_sendStop();
  // Disable the I2C master mode and wait for sync
  s->CR1 &= ~I2C_CR1_PE;  // Disable I2C peripheral
  // Should never happen, but wait for up to 500us only.
  unsigned long startTime = micros();
  while ((s->CR1 && I2C_CR1_PE) != 0) {
    if (micros() - startTime >= 500UL) break;
  }
}

/***************************************************************************
 *  Main state machine for I2C, called from interrupt handler or,
 *  if I2C_USE_INTERRUPTS isn't defined, from the I2CManagerClass::loop() function
 *  (and therefore, indirectly, from I2CRB::wait() and I2CRB::isBusy()).
 ***************************************************************************/
void I2CManagerClass::I2C_handleInterrupt() {

  if (s->SR1 && I2C_SR1_ARLO) {
    // Arbitration lost, restart
    I2C_sendStart();   // Reinitiate request
  } else if (s->SR1 && I2C_SR1_BERR) {
    // Bus error
    completionStatus = I2C_STATUS_BUS_ERROR;
    state = I2C_STATE_COMPLETED;
  } else if (s->SR1 && I2C_SR1_TXE) {
    // Master write completed
    if (s->SR1 && (1<<10)) {
      // Nacked, send stop.
      I2C_sendStop();
      completionStatus = I2C_STATUS_NEGATIVE_ACKNOWLEDGE;
      state = I2C_STATE_COMPLETED;
    } else if (bytesToSend) {
      // Acked, so send next byte
      s->DR = sendBuffer[txCount++];
      bytesToSend--;
    } else if (bytesToReceive) {
      // Last sent byte acked and no more to send.  Send repeated start, address and read bit.
      // s->I2CM.ADDR.bit.ADDR = (deviceAddress << 1) | 1;
    } else {
      // Check both TxE/BTF == 1 before generating stop
      while (!(s->SR1 && I2C_SR1_TXE));    // Check TxE
      while (!(s->SR1 && I2C_SR1_BTF));    // Check BTF
      // No more data to send/receive. Initiate a STOP condition and finish
      I2C_sendStop();
      state = I2C_STATE_COMPLETED;
    }
  } else if (s->SR1 && I2C_SR1_RXNE) {
    // Master read completed without errors
    if (bytesToReceive == 1) {
//      s->I2CM.CTRLB.bit.ACKACT = 1;  // NAK final byte
      I2C_sendStop();  // send stop
      receiveBuffer[rxCount++] = s->DR;  // Store received byte
      bytesToReceive = 0;
      state = I2C_STATE_COMPLETED;
    } else if (bytesToReceive) {
//      s->I2CM.CTRLB.bit.ACKACT = 0;  // ACK all but final byte
      receiveBuffer[rxCount++] = s->DR;  // Store received byte
      bytesToReceive--;
    }
  }
}

#endif /* I2CMANAGER_STM32_H */
