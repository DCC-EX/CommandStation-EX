   /*
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
 * DFPlayer is an MP3 player module with an SD card holder.  It also has an integrated
 * amplifier, so it only needs a power supply and a speaker.
 * This driver is a modified version of the IO_DFPlayer.h file
 * *********************************************************************************************
 * 
 * Dec 2023, Added NXP SC16IS752 I2C Dual UART to enable the DFPlayer connection over the I2C bus
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
 * DFPlayer::create(1st vPin, vPins, I2C address, xtal);
 * 
 * Parameters:
 * 1st vPin     : First virtual pin that EX-Rail can control to play a sound, use PLAYSOUND command (alias of ANOUT)
 * vPins        : Total number of virtual pins allocated (2 vPins are supported, one for each UART)
 *                1st vPin for UART 0, 2nd for UART 1
 * I2C Address  : I2C address of the serial controller, in 0x format
 * xtal         : 0 for 1,8432Mhz, 1 for 14,7456Mhz
 * 
 * The vPin is also a pin that can be read, it indicate if the DFPlayer has finished playing a track
 *
 */

#ifndef IO_DFPlayerI2C_h
#define IO_DFPlayerI2C_h

#include "IODevice.h"
#include "I2CManager.h"
#include "IO_DFPlayerBase.h"
#include "DIAG.h"

class DFPlayerI2C : public DFPlayerBase  {
private: 
  // SC16IS752 defines
  uint8_t _UART_CH=0x00;  // Fix uart ch to 0 for now
  // Communication parameters for the DFPlayer are fixed at 8 bit, No parity, 1 stopbit
  const uint8_t WORD_LEN = 0x03;    // Value LCR bit 0,1
  const uint8_t STOP_BIT = 0x00;    // Value LCR bit 2 
  const uint8_t PARITY_ENA = 0x00;  // Value LCR bit 3
  const uint8_t PARITY_TYPE = 0x00; // Value LCR bit 4
  const uint32_t BAUD_RATE = 9600;
  const uint8_t PRESCALER = 0x01;   // Value MCR bit 7
   
  unsigned long _sc16is752_xtal_freq;
  const unsigned long SC16IS752_XTAL_FREQ_LOW = 1843200; // To support cheap eBay/AliExpress SC16IS752 boards
  const unsigned long SC16IS752_XTAL_FREQ_HIGH = 14745600; // Support for higher baud rates, standard for modular EX-IO system
   
public:
  // Constructor
   DFPlayerI2C(VPIN firstVpin, int nPins, I2CAddress i2cAddress, uint8_t xtal) : DFPlayerBase(firstVpin, nPins)  {
    _I2CAddress = i2cAddress;
    _sc16is752_xtal_freq = (xtal) ? SC16IS752_XTAL_FREQ_HIGH: SC16IS752_XTAL_FREQ_LOW;
    addDevice(this);
   } 
  
public:
 
  void _begin() override {
    // check if SC16IS752 exist first, initialize and then resume DFPlayer init via SC16IS752
    I2CManager.begin();
    I2CManager.setClock(1000000);
    if (I2CManager.exists(_I2CAddress)){
      DIAG(F("SC16IS752 I2C:%s UART detected"), _I2CAddress.toString());
      Init_SC16IS752(); // Initialize UART
      if (_deviceState == DEVSTATE_FAILED){
        DIAG(F("SC16IS752 I2C:%s UART initialization failed"), _I2CAddress.toString());
        }
      } else {
         DIAG(F("SC16IS752 I2C:%s UART not detected"), _I2CAddress.toString());
        }
      _display();
    DFPlayerBase::_begin(); // Resume DFPlayer init 
     }
  
   void transmitCommandBuffer(const uint8_t buffer[],size_t bytes) override {
      // Transmit the command buffer via the uart  
      if (_deviceState == DEVSTATE_FAILED) return;
      if(UART_ReadRegister(REG_TXLV)< bytes) { //FIFO is full
        DIAG(F("DFPlayer at: %s, TX FIFO not empty on UART: 0x%x"), _I2CAddress.toString(), _UART_CH);
        _deviceState = DEVSTATE_FAILED; // This should not happen 
        return;
      }

      I2CManager.write(_I2CAddress, buffer, bytes);
    }

  bool processIncoming() override  {
    // poll uart for any input, and pass it to processIncomingByte 
    if (_deviceState == DEVSTATE_FAILED) return false;
    uint8_t available=UART_ReadRegister(REG_RXLV);
    if (available==0) return false; // No data available

    uint8_t outbuf[]={combine(REG_RHR)};
    uint8_t inbuf[available];
    I2CManager.read(_I2CAddress, inbuf, available, outbuf, 1);
    for (uint8_t i=0;i<available;i++){
      processIncomingByte(inbuf[i]);
    }
    return true;
  }
      
  
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
  uint8_t _status;   // latest i2c status 

  uint8_t combine(uint8_t reg) {
    return (reg << 3) | (_UART_CH << 1);
  }

  void Init_SC16IS752(){ // Return value is in _deviceState
    #ifdef DIAG_I2CDFplayer
      DIAG(F("SC16IS752: Initialize I2C: %s , UART Ch: 0x%x"), _I2CAddress.toString(),  _UART_CH);      
    #endif
    //uint16_t _divisor = (SC16IS752_XTAL_FREQ / PRESCALER) / (BAUD_RATE * 16);
    uint16_t _divisor = (_sc16is752_xtal_freq/PRESCALER)/(BAUD_RATE * 16);  // Calculate _divisor for baudrate
    UART_WriteRegister(REG_IOCONTROL, 0x08);  // uart sopftware reset 

    // Extra delay when using low frequency xtal after soft reset
    // Test when using 1.8432 Mhz xtal.

    if(_sc16is752_xtal_freq == SC16IS752_XTAL_FREQ_LOW){
      delay(10);  // 10mS timeout (safe because this is only during setup)      
    }

     
    UART_WriteRegister(REG_IOCONTROL, 0x00); // Set pins to GPIO mode
    UART_WriteRegister(REG_IODIR, 0xFF); //Set all pins as output
    UART_WriteRegister(REG_IOSTATE, 0x00); //Set all pins low

    UART_WriteRegister(REG_FCR, 0x07); // Enable FIFO and reset RX & TX FIFO
    UART_WriteRegister(REG_MCR, 0x00); // Set MCR to all 0, includes Clock divisor
    UART_WriteRegister(REG_LCR, 0x80 | WORD_LEN | STOP_BIT | PARITY_ENA | PARITY_TYPE); // Divisor latch enabled
    UART_WriteRegister(REG_DLL, (uint8_t)_divisor);  // Write DLL
    UART_WriteRegister(REG_DLH, (uint8_t)(_divisor >> 8)); // Write DLH
    UART_ReadRegister(REG_LCR);
    UART_WriteRegister(REG_LCR, UART_ReadRegister(REG_LCR) & 0x7F); // Divisor latch disabled  

    DIAG(F("SC16IS752: I2C: %s init %S"), _I2CAddress.toString(), I2CManager.getErrorMessage(_status));
      
     _deviceState =  (_status == I2C_STATUS_OK) ? DEVSTATE_NORMAL : DEVSTATE_FAILED;
  }


  void UART_WriteRegister(uint8_t UART_REG, uint8_t Val) {
    uint8_t outbuffer[] = {combine(UART_REG), Val};
    _status=I2CManager.write(_I2CAddress, outbuffer, 2);
  }

 
  uint8_t UART_ReadRegister(uint8_t UART_REG){
     uint8_t outbuffer[] =  {combine(UART_REG)}; // _outbuffer has now UART_REG and UART_CH
     uint8_t inbuffer[1];
     _status=I2CManager.read(_I2CAddress, inbuffer, 1, outbuffer, 1);    
     return inbuffer[0];// _inbuffer has the REG data
  }

// SC16IS752 General register set (from the datasheet)
enum : uint8_t{
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

// SC16IS752 Enhanced register set
enum : uint8_t{
    REG_EFR       = 0X02, // Enhanced Features Register R/W
    REG_XON1      = 0x04, // R/W
    REG_XON2      = 0x05, // R/W
    REG_XOFF1     = 0x06, // R/W
    REG_XOFF2     = 0x07, // R/W
  };

};

#endif // IO_DFPlayerI2C_h
