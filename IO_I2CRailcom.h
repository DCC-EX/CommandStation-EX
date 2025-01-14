   /*
 *  © 2024, Henk Kruisbrink & Chris Harlow. All rights reserved.
 *  © 2023, Neil McKechnie. All rights reserved.
 *  
 *  This file is part of DCC++EX API
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

/*
 * 
 * Dec 2023, Added NXP SC16IS752 I2C Dual UART
 * The SC16IS752 has 64 bytes TX & RX FIFO buffer
 * First version without interrupts from I2C UART and only RX/TX are used, interrupts may not be
 * needed as the RX Fifo holds the reply 
 * 
 * Jan 2024, Issue with using both UARTs simultaniously, the secod uart seems to work  but the first transmit 
 * corrupt data. This need more analysis and experimenatation. 
 * Will push this driver to the dev branch with the uart fixed to 0 
 * Both SC16IS750 (single uart) and SC16IS752 (dual uart, but only uart 0 is enable)
 * 
 * myHall.cpp configuration syntax:
 * 
 * I2CRailcom::create(1st vPin, vPins, I2C address);
 * 
 * myAutomation configuration
 *  HAL(I2CRailcom, 1st vPin, vPins, I2C address)
 * Parameters:
 * 1st vPin     : First virtual pin that EX-Rail can control to play a sound, use PLAYSOUND command (alias of ANOUT)
 * vPins        : Total number of virtual pins allocated (2 vPins are supported, one for each UART)
 *                1st vPin for UART 0, 2nd for UART 1
 * I2C Address  : I2C address of the serial controller, in 0x format
 */

#ifndef IO_I2CRailcom_h
#define IO_I2CRailcom_h
#include "Arduino.h"
#include "Railcom.h"

// Debug and diagnostic defines, enable too many will result in slowing the driver
#define DIAG_I2CRailcom

class I2CRailcom : public IODevice {
private: 
  // SC16IS752 defines
  uint8_t _UART_CH=0x00; // channel 0 or 1 flips each loop if npins>1
  byte _inbuf[12];
  byte _outbuf[2];
  byte cutoutCounter[2]; 
  Railcom * _channelMonitors[2]; 
public:
  // Constructor
  I2CRailcom(VPIN firstVpin, int nPins, I2CAddress i2cAddress);
  
  static void create(VPIN firstVpin, int nPins, I2CAddress i2cAddress) ;
  
  void _begin() ;
  void _loop(unsigned long currentMicros) override ;
  void _display() override ;
  
private: 


  // SC16IS752 functions
  // Initialise SC16IS752 only for this channel
  // First a software reset
  // Enable FIFO and clear TX & RX FIFO
  // Need to set the following registers
  // IOCONTROL set bit 1 and 2 to 0 indicating that they are GPIO
  // IODIR set all bit to 1 indicating al are output
  // IOSTATE set only bit 0 to 1 for UART 0, or only bit 1 for UART 1  // 
  // LCR bit 7=0 divisor latch (clock division registers DLH & DLL, they store 16 bit divisor), 
  //     WORD_LEN, STOP_BIT, PARITY_ENA and PARITY_TYPE
  // MCR bit 7=0 clock divisor devide-by-1 clock input
  // DLH most significant part of divisor
  // DLL least significant part of divisor
  //
  // BAUD_RATE, WORD_LEN, STOP_BIT, PARITY_ENA and PARITY_TYPE have been defined and initialized
  //
  // Communication parameters 8 bit, No parity, 1 stopbit
  static const uint8_t WORD_LEN = 0x03;    // Value LCR bit 0,1
  static const uint8_t STOP_BIT = 0x00;    // Value LCR bit 2 
  static const uint8_t PARITY_ENA = 0x00;  // Value LCR bit 3
  static const uint8_t PARITY_TYPE = 0x00; // Value LCR bit 4
  static const uint32_t BAUD_RATE = 250000;
  static const uint8_t PRESCALER = 0x01;   // Value MCR bit 7  
  static const unsigned long SC16IS752_XTAL_FREQ_RAILCOM = 16000000; // Baud rate for Railcom signal
  static const uint16_t _divisor = (SC16IS752_XTAL_FREQ_RAILCOM / PRESCALER) / (BAUD_RATE * 16);  
     
  void Init_SC16IS752(); 
  void UART_WriteRegister(uint8_t reg, uint8_t val, bool readback=true);
  uint8_t UART_ReadRegister(uint8_t reg);

// SC16IS752 General register set (from the datasheet)
enum : uint8_t {
    REG_RHR       = 0x00, // FIFO Read
    REG_THR       = 0x00, // FIFO Write
    REG_IER       = 0x01, // Interrupt Enable Register R/W
    REG_FCR       = 0x02, // FIFO Control Register Write
    REG_IIR       = 0x02, // Interrupt Identification Register Read
    REG_LCR       = 0x03, // Line Control Register R/W
    REG_MCR       = 0x04, // Modem Control Register R/W
    REG_LSR       = 0x05, // Line Status Register Read
    REG_MSR       = 0x06, // Modem Status Register Read
    REG_SPR       = 0x07, // Scratchpad Register R/W
    REG_TCR       = 0x06, // Transmission Control Register R/W
    REG_TLR       = 0x07, // Trigger Level Register R/W    
    REG_TXLV      = 0x08, // Transmitter FIFO Level register Read
    REG_RXLV      = 0x09, // Receiver FIFO Level register Read
    REG_IODIR     = 0x0A, // Programmable I/O pins Direction register R/W
    REG_IOSTATE   = 0x0B, // Programmable I/O pins State register R/W
    REG_IOINTENA  = 0x0C, // I/O Interrupt Enable register R/W
    REG_IOCONTROL = 0x0E, // I/O Control register R/W
    REG_EFCR      = 0x0F, // Extra Features Control Register R/W
  };

// SC16IS752 Special register set
enum : uint8_t{
    REG_DLL       = 0x00, // Division registers R/W
    REG_DLH       = 0x01, // Division registers R/W
  };

// SC16IS752 Enhanced regiter set
enum : uint8_t{
    REG_EFR       = 0X02, // Enhanced Features Register R/W
    REG_XON1      = 0x04, // R/W
    REG_XON2      = 0x05, // R/W
    REG_XOFF1     = 0x06, // R/W
    REG_XOFF2     = 0x07, // R/W
  };

};

#endif // IO_I2CRailcom_h
