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

#include <wiring_private.h>
#include "stm32f4xx_hal_rcc.h"

/*****************************************************************************
 *  STM32F4xx I2C native driver support
 * 
 *  Nucleo-64 and Nucleo-144 boards all use I2C1 as the default I2C peripheral
 *  Later we may wish to support other STM32 boards, allow use of an alternate
 *  I2C bus, or more than one I2C bus on the STM32 architecture 
 *****************************************************************************/
#if defined(I2C_USE_INTERRUPTS) && defined(ARDUINO_ARCH_STM32)
#if defined(ARDUINO_NUCLEO_F401RE) || defined(ARDUINO_NUCLEO_F411RE) || defined(ARDUINO_NUCLEO_F446RE) \
    || defined(ARDUINO_NUCLEO_F412ZG) || defined(ARDUINO_NUCLEO_F413ZH) \
    || defined(ARDUINO_NUCLEO_F429ZI) || defined(ARDUINO_NUCLEO_F446ZE)
// Assume I2C1 for now - default I2C bus on Nucleo-F411RE and likely all Nucleo-64
// and Nucleo-144 variants
I2C_TypeDef *s = I2C1;

// In init we will ask the STM32 HAL layer for the configured APB1 clock frequency in Hz
uint32_t APB1clk1; // Peripheral Input Clock speed in Hz.
uint32_t i2c_MHz;  // Peripheral Input Clock speed in MHz.

// IRQ handler for I2C1, replacing the weak definition in the STM32 HAL 
extern "C" void I2C1_EV_IRQHandler(void) {
  I2CManager.handleInterrupt();
}
extern "C" void I2C1_ER_IRQHandler(void) {
  I2CManager.handleInterrupt();
}
#else
#warning STM32 board selected is not yet supported - so I2C1 peripheral is not defined
#endif
#endif

// Peripheral Input Clock speed in MHz.
// For STM32F446RE, the speed is 45MHz.  Ideally, this should be determined
// at run-time from the APB1 clock, as it can vary from STM32 family to family.
// #define I2C_PERIPH_CLK 45

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

// States of the STM32 I2C driver state machine
enum {TS_IDLE,TS_START,TS_W_ADDR,TS_W_DATA,TS_W_STOP,TS_R_ADDR,TS_R_DATA,TS_R_STOP};


/***************************************************************************
 *  Set I2C clock speed register.  This should only be called outside of
 *  a transmission.  The I2CManagerClass::_setClock() function ensures 
 *  that it is only called at the beginning of an I2C transaction.
 ***************************************************************************/
void I2CManagerClass::I2C_setClock(uint32_t i2cClockSpeed) {
  // Calculate a rise time appropriate to the requested bus speed
  // Use 10x the rise time spec to enable integer divide of 50ns clock period
  uint16_t t_rise;

  while (s->CR1 & I2C_CR1_STOP);  // Prevents lockup by guarding further
                                  // writes to CR1 while STOP is being executed!

  // Disable the I2C device, as TRISE can only be programmed whilst disabled
  s->CR1 &= ~(I2C_CR1_PE);  // Disable I2C
  s->CR1 |= I2C_CR1_SWRST;  // reset the I2C
  asm("nop");    // wait a bit... suggestion from online!
  s->CR1 &= ~(I2C_CR1_SWRST); // Normal operation

  if (i2cClockSpeed > 100000UL)
  {
    // if (i2cClockSpeed > 400000L)
    //   i2cClockSpeed = 400000L;

    t_rise = 300;  // nanoseconds
  }
  else
  {
    // i2cClockSpeed = 100000L;
    t_rise = 1000;  // nanoseconds
  }
  // Configure the rise time register - max allowed tRISE is 1000ns,
  // so value = 1000ns * I2C_PERIPH_CLK MHz / 1000 + 1.
  s->TRISE = (t_rise * i2c_MHz / 1000) + 1;

  // Bit 15: I2C Master mode, 0=standard, 1=Fast Mode
  // Bit 14: Duty, fast mode duty cycle (use 2:1)
  // Bit 11-0: FREQR
  // if (i2cClockSpeed > 400000UL) {
  //   // In fast mode plus, I2C period is 3 * CCR * TPCLK1.
  //   // s->CCR &= ~(0x3000); // Clear all bits except 12 and 13 which must remain per reset value
  //   s->CCR = APB1clk1 / 3 / i2cClockSpeed; // Set I2C clockspeed to start!
  //   s->CCR |= 0xC000; // We need Fast Mode AND DUTY bits set
  // } else {
    // In standard and fast mode, I2C period is 2 * CCR * TPCLK1
    s->CCR &= ~(0x3000); // Clear all bits except 12 and 13 which must remain per reset value
    s->CCR |= (APB1clk1 / 2 / i2cClockSpeed); // Set I2C clockspeed to start!
    // s->CCR |= (i2c_MHz * 500 / (i2cClockSpeed / 1000)); // Set I2C clockspeed to start!
    // if (i2cClockSpeed > 100000UL)
    //     s->CCR |= 0xC000; // We need Fast Mode bits set as well
  // }

  // DIAG(F("I2C_init() peripheral clock is now: %d, full reg is %x"), (s->CR2 & 0xFF), s->CR2);
  // DIAG(F("I2C_init() peripheral CCR is now: %d"), s->CCR);
  // DIAG(F("I2C_init() peripheral TRISE is now: %d"), s->TRISE);

  // Enable the I2C master mode
  s->CR1 |= I2C_CR1_PE;  // Enable I2C
}

/***************************************************************************
 *  Initialise I2C registers.
 ***************************************************************************/
void I2CManagerClass::I2C_init()
{
  // Query the clockspeed from the STM32 HAL layer
  APB1clk1 = HAL_RCC_GetPCLK1Freq();
  i2c_MHz = APB1clk1 / 1000000UL;
  // DIAG(F("I2C_init() peripheral clock speed is: %d"), i2c_MHz);
  // Enable clocks
  RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;//(1 << 21); // Enable I2C CLOCK
  // Reset the I2C1 peripheral to initial state
  RCC->APB1RSTR |=  RCC_APB1RSTR_I2C1RST;
	RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;
  // Standard I2C pins are SCL on PB8 and SDA on PB9
  RCC->AHB1ENR |= (1<<1);   // Enable GPIOB CLOCK for PB8/PB9
  // Bits (17:16)= 1:0 --> Alternate Function for Pin PB8;
  // Bits (19:18)= 1:0 --> Alternate Function for Pin PB9
  GPIOB->MODER &= ~((3<<(8*2)) | (3<<(9*2)));    // Clear all MODER bits for PB8 and PB9
  GPIOB->MODER |= (2<<(8*2)) | (2<<(9*2));    // PB8 and PB9 set to ALT function
  GPIOB->OTYPER |= (1<<8) | (1<<9);           // PB8 and PB9 set to open drain output capability
  GPIOB->OSPEEDR |= (3<<(8*2)) | (3<<(9*2));  // PB8 and PB9 set to High Speed mode
  GPIOB->PUPDR &= ~((3<<(8*2)) | (3<<(9*2))); // Clear all PUPDR bits for PB8 and PB9
  GPIOB->PUPDR |= (1<<(8*2)) | (1<<(9*2));    // PB8 and PB9 set to pull-up capability
  // Alt Function High register routing pins PB8 and PB9 for I2C1:
  // Bits (3:2:1:0) = 0:1:0:0 --> AF4 for pin PB8
  // Bits (7:6:5:4) = 0:1:0:0 --> AF4 for pin PB9
  GPIOB->AFR[1] &= ~((15<<0) | (15<<4));      // Clear all AFR bits for PB8 on low nibble, PB9 on next nibble up
  GPIOB->AFR[1] |= (4<<0) | (4<<4);           // PB8 on low nibble, PB9 on next nibble up

  // Software reset the I2C peripheral
  I2C1->CR1 &= ~I2C_CR1_PE; // Disable I2C1 peripheral
  s->CR1 |= I2C_CR1_SWRST;  // reset the I2C
  asm("nop");    // wait a bit... suggestion from online!
  s->CR1 &= ~(I2C_CR1_SWRST); // Normal operation

  // Clear all bits in I2C CR2 register except reserved bits
  s->CR2 &= 0xE000;

  // Set I2C peripheral clock frequency
  // s->CR2 |= I2C_PERIPH_CLK;
  s->CR2 |= i2c_MHz;
  // DIAG(F("I2C_init() peripheral clock is now: %d"), s->CR2);

  // set own address to 00 - not used in master mode
  I2C1->OAR1 = (1 << 14); // bit 14 should be kept at 1 according to the datasheet

#if defined(I2C_USE_INTERRUPTS)
  // Setting NVIC
  NVIC_SetPriority(I2C1_EV_IRQn, 1);  // Match default priorities
  NVIC_EnableIRQ(I2C1_EV_IRQn);
  NVIC_SetPriority(I2C1_ER_IRQn, 1);  // Match default priorities
  NVIC_EnableIRQ(I2C1_ER_IRQn);

  // CR2 Interrupt Settings
  // Bit 15-13: reserved
  // Bit 12: LAST - DMA last transfer
  // Bit 11: DMAEN - DMA enable
  // Bit 10: ITBUFEN - Buffer interrupt enable
  // Bit 9: ITEVTEN - Event interrupt enable
  // Bit 8: ITERREN - Error interrupt enable
  // Bit 7-6: reserved
  // Bit 5-0: FREQ - Peripheral clock frequency (max 50MHz)
  s->CR2 |= (I2C_CR2_ITBUFEN | I2C_CR2_ITEVTEN | I2C_CR2_ITERREN);   // Enable Buffer, Event and Error interrupts
#endif

  // DIAG(F("I2C_init() setting initial I2C clock to 100KHz"));
  // Calculate baudrate and set default rate for now
  // Configure the Clock Control Register for 100KHz SCL frequency
  // Bit 15: I2C Master mode, 0=standard, 1=Fast Mode
  // Bit 14: Duty, fast mode duty cycle
  // Bit 11-0: so CCR divisor would be clk / 2 / 100000 (where clk is in Hz)
  // s->CCR = I2C_PERIPH_CLK * 5;
  s->CCR &= ~(0x3000); // Clear all bits except 12 and 13 which must remain per reset value
  s->CCR |= (APB1clk1 / 2 / 100000UL); // Set a default of 100KHz I2C clockspeed to start!

  // Configure the rise time register - max allowed is 1000ns, so value = 1000ns * I2C_PERIPH_CLK MHz / 1000 + 1.
  s->TRISE = (1000 * i2c_MHz / 1000) + 1;

  // DIAG(F("I2C_init() peripheral clock is now: %d, full reg is %x"), (s->CR2 & 0xFF), s->CR2);
  // DIAG(F("I2C_init() peripheral CCR is now: %d"), s->CCR);
  // DIAG(F("I2C_init() peripheral TRISE is now: %d"), s->TRISE);

  // Enable the I2C master mode
  s->CR1 |= I2C_CR1_PE;  // Enable I2C
}

/***************************************************************************
 *  Initiate a start bit for transmission.
 ***************************************************************************/
void I2CManagerClass::I2C_sendStart() {

  // Set counters here in case this is a retry.
  rxCount = txCount = 0;

  // On a single-master I2C bus, the start bit won't be sent until the bus
  // state goes to IDLE so we can request it without waiting.  On a
  // multi-master bus, the bus may be BUSY under control of another master,
  // in which case we can avoid some arbitration failures by waiting until
  // the bus state is IDLE.  We don't do that here.
  //while (s->SR2 & I2C_SR2_BUSY) {}

  // Check there's no STOP still in progress.  If we OR the START bit into CR1
  // and the STOP bit is already set, we could output multiple STOP conditions.
  while (s->CR1 & I2C_CR1_STOP) {}  // Wait for STOP bit to reset

  s->CR2 |= (I2C_CR2_ITEVTEN | I2C_CR2_ITERREN);  // Enable interrupts
  s->CR2 &= ~I2C_CR2_ITBUFEN;     // Don't enable buffer interupts yet.
  s->CR1 &= ~I2C_CR1_POS;   // Clear the POS bit
  s->CR1 |= (I2C_CR1_ACK | I2C_CR1_START);   // Enable the ACK and generate START
  transactionState = TS_START;
}

/***************************************************************************
 *  Initiate a stop bit for transmission (does not interrupt)
 ***************************************************************************/
void I2CManagerClass::I2C_sendStop() {
  s->CR1 |= I2C_CR1_STOP; // Stop I2C
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
  while ((s->CR1 & I2C_CR1_PE) != 0) {
    if ((int32_t)(micros() - startTime) >= 500) break;
  }
  NVIC_DisableIRQ(I2C1_EV_IRQn);
  NVIC_DisableIRQ(I2C1_ER_IRQn);
}

/***************************************************************************
 *  Main state machine for I2C, called from interrupt handler or,
 *  if I2C_USE_INTERRUPTS isn't defined, from the I2CManagerClass::loop() function
 *  (and therefore, indirectly, from I2CRB::wait() and I2CRB::isBusy()).
 ***************************************************************************/
void I2CManagerClass::I2C_handleInterrupt() {
  volatile uint16_t temp_sr1, temp_sr2;

  temp_sr1 = s->SR1;

  // Check for errors first
  if (temp_sr1 & (I2C_SR1_AF | I2C_SR1_ARLO | I2C_SR1_BERR)) {
    // Check which error flag is set
    if (temp_sr1 & I2C_SR1_AF)
    {
      s->SR1 &= ~(I2C_SR1_AF); // Clear AF
      I2C_sendStop(); // Clear the bus
      transactionState = TS_IDLE;
      completionStatus = I2C_STATUS_NEGATIVE_ACKNOWLEDGE;
      state = I2C_STATE_COMPLETED;
    }
    else if (temp_sr1 & I2C_SR1_ARLO)
    {
      // Arbitration lost, restart
      s->SR1 &= ~(I2C_SR1_ARLO); // Clear ARLO
      I2C_sendStart(); // Reinitiate request
      transactionState = TS_START;
    }
    else if (temp_sr1 & I2C_SR1_BERR)
    {
      // Bus error
      s->SR1 &= ~(I2C_SR1_BERR); // Clear BERR
      I2C_sendStop(); // Clear the bus
      transactionState = TS_IDLE;
      completionStatus = I2C_STATUS_BUS_ERROR;
      state = I2C_STATE_COMPLETED;
    }
  } 
  else {
    // No error flags, so process event according to current state.
    switch (transactionState) {
      case TS_START:
        if (temp_sr1 & I2C_SR1_SB) {
          // Event EV5
          // Start bit has been sent successfully and we have the bus.
          // If anything to send, initiate write.  Otherwise initiate read.
          if (operation == OPERATION_READ || ((operation == OPERATION_REQUEST) && !bytesToSend)) {
            // Send address with read flag (1) or'd in
            s->DR = (deviceAddress << 1) | 1;  //  send the address
            transactionState = TS_R_ADDR;
          } else {
            // Send address with write flag (0) or'd in
            s->DR = (deviceAddress << 1) | 0;  //  send the address
            transactionState = TS_W_ADDR;
          }
        }
        // SB bit is cleared by writing to DR (already done).
        break;

      case TS_W_ADDR:
        if (temp_sr1 & I2C_SR1_ADDR) {
          temp_sr2 = s->SR2; // read SR2 to complete clearing the ADDR bit
          // Event EV6
          // Address sent successfully, device has ack'd in response.
          if (!bytesToSend) {
            I2C_sendStop();
            transactionState = TS_IDLE;
            completionStatus = I2C_STATUS_OK;
            state = I2C_STATE_COMPLETED;
          } else {
            // Put one byte into DR to load shift register.
            s->DR = sendBuffer[txCount++];
            bytesToSend--;
            if (bytesToSend) {
              // Put another byte to load DR
              s->DR = sendBuffer[txCount++];
              bytesToSend--;
            }
            if (!bytesToSend) {
              // No more bytes to send.
              // The TXE interrupt occurs when the DR is empty, and the BTF interrupt 
              // occurs when the shift register is also empty (one character later).
              // To avoid repeated TXE interrupts during this time, we disable TXE interrupt.
              s->CR2 &= ~I2C_CR2_ITBUFEN;  // Wait for BTF interrupt, disable TXE interrupt
              transactionState = TS_W_STOP;
            } else {
              // More data remaining to send after this interrupt, enable TXE interrupt.
              s->CR2 |= I2C_CR2_ITBUFEN;
              transactionState = TS_W_DATA;
            }
          }
        }
        break;

      case TS_W_DATA:
        if (temp_sr1 & I2C_SR1_TXE) {
          // Event EV8_1/EV8
          // Transmitter empty, write a byte to it.
          if (bytesToSend) {
            s->DR = sendBuffer[txCount++];
            bytesToSend--;
            if (!bytesToSend) {
              s->CR2 &= ~I2C_CR2_ITBUFEN;  // Disable TXE interrupt
              transactionState = TS_W_STOP;
            }
          }
        } 
        break;

      case TS_W_STOP:
        if (temp_sr1 & I2C_SR1_BTF) {
          // Event EV8_2
          // Done, last character sent. Anything to receive?
          if (bytesToReceive) {
            I2C_sendStart();
            // NOTE: Three redundant BTF interrupts take place between the
            // first BTF interrupt and the START interrupt.  I've tried all sorts
            // of ways to eliminate them, and the only thing that worked for
            // me was to loop until the BTF bit becomes reset.  Either way,
            // it's a waste of processor time.  Anyone got a solution?
            //while (s->SR1 && I2C_SR1_BTF) {}
            transactionState = TS_START;
          } else {
            I2C_sendStop();
            transactionState = TS_IDLE;
            completionStatus = I2C_STATUS_OK;
            state = I2C_STATE_COMPLETED;
          }
          s->SR1 &= I2C_SR1_BTF;  // Clear BTF interrupt
        }
        break;

      case TS_R_ADDR:
        if (temp_sr1 & I2C_SR1_ADDR) {
          // Event EV6
          // Address sent for receive.
          // The next bit is different depending on whether there are 
          // 1 byte, 2 bytes or >2 bytes to be received, in accordance with the
          // Programmers Reference RM0390.
          if (bytesToReceive == 1) {
            // Receive 1 byte
            s->CR1 &= ~I2C_CR1_ACK;  // Disable ack
            temp_sr2 = s->SR2; // read SR2 to complete clearing the ADDR bit
            // Next step will occur after a RXNE interrupt, so enable it
            s->CR2 |= I2C_CR2_ITBUFEN;
            transactionState = TS_R_STOP;
          } else if (bytesToReceive == 2) {
            // Receive 2 bytes
            s->CR1 &= ~I2C_CR1_ACK;  // Disable ACK for final byte
            s->CR1 |= I2C_CR1_POS;  // set POS flag to delay effect of ACK flag
            // Next step will occur after a BTF interrupt, so disable RXNE interrupt
            s->CR2 &= ~I2C_CR2_ITBUFEN;
            temp_sr2 = s->SR2; // read SR2 to complete clearing the ADDR bit
            transactionState = TS_R_STOP;
          } else {
            // >2 bytes, just wait for bytes to come in and ack them for the time being
            // (ack flag has already been set).
            // Next step will occur after a BTF interrupt, so disable RXNE interrupt
            s->CR2 &= ~I2C_CR2_ITBUFEN;
            temp_sr2 = s->SR2; // read SR2 to complete clearing the ADDR bit
            transactionState = TS_R_DATA;
          }
        }
        break;
      
      case TS_R_DATA:
        // Event EV7/EV7_1
        if (temp_sr1 & I2C_SR1_BTF) {
          // Byte received in receiver - read next byte
          if (bytesToReceive == 3) {
            // Getting close to the last byte, so a specific sequence is recommended.
            s->CR1 &= ~I2C_CR1_ACK;  // Reset ack for next byte received.
            transactionState = TS_R_STOP;
          }
          receiveBuffer[rxCount++] = s->DR;  // Store received byte
          bytesToReceive--;
        } 
        break;
        
      case TS_R_STOP:
        if (temp_sr1 & I2C_SR1_BTF) {
          // Event EV7 (last one)
          // When we've got here, the receiver has got the last two bytes
          // (or one byte, if only one byte is being received),
          // and NAK has already been sent, so we need to read from the receiver.
          if (bytesToReceive) {
            if (bytesToReceive > 1) 
              I2C_sendStop();
            while(bytesToReceive) {
              receiveBuffer[rxCount++] = s->DR;  // Store received byte(s)
              bytesToReceive--;
            }
            // Finish.
            transactionState = TS_IDLE;
            completionStatus = I2C_STATUS_OK;
            state = I2C_STATE_COMPLETED;
          }
        } else if (temp_sr1 & I2C_SR1_RXNE) {
          if (bytesToReceive == 1) {
            // One byte on a single-byte transfer.  Ack has already been set.
            I2C_sendStop();
            receiveBuffer[rxCount++] = s->DR; // Store received byte
            bytesToReceive--;
            // Finish.
            transactionState = TS_IDLE;
            completionStatus = I2C_STATUS_OK;
            state = I2C_STATE_COMPLETED;
          } else
            s->SR1 &= I2C_SR1_RXNE;  // Acknowledge interrupt
        }
        break;
    }
    // If we've received an interrupt at any other time, we're not interested so clear it
    // to prevent it recurring ad infinitum.
    s->SR1 = 0;
  }
  
}

#endif /* I2CMANAGER_STM32_H */
