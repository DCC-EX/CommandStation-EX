/*
 *  © 2021, Neil McKechnie. All rights reserved.
 *  Based on the work by DFRobot, Frank de Brabander and Marco Schwartz.
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

#ifndef LiquidCrystal_I2C_h
#define LiquidCrystal_I2C_h

#include <Arduino.h>
#include "Display.h"
#include "I2CManager.h"

// commands
#define LCD_CLEARDISPLAY 0x01
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// flags for display entry mode
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_CURSOROFF 0x00
#define LCD_BLINKOFF 0x00

// flags for function set
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x8DOTS 0x00

// Bit mapping onto PCF8574 port
#define BACKPACK_Rs_BIT 0
#define BACKPACK_Rw_BIT 1
#define BACKPACK_En_BIT 2
#define BACKPACK_BACKLIGHT_BIT 3
#define BACKPACK_DATA_BITS 4 // Bits 4-7
// Equivalent mask bits
#define LCD_BACKLIGHT (1 << BACKPACK_BACKLIGHT_BIT)  // Backlight enable
#define En (1 << BACKPACK_En_BIT)  // Enable bit
#define Rw (1 << BACKPACK_Rw_BIT)  // Read/Write bit
#define Rs (1 << BACKPACK_Rs_BIT)  // Register select bit

class LiquidCrystal_I2C : public DisplayDevice {
public:
  LiquidCrystal_I2C(I2CAddress lcd_Addr,uint8_t lcd_cols,uint8_t lcd_rows);
  bool begin() override;
  void clearNative() override;
  void setRowNative(byte line) override;
  size_t writeNative(uint8_t c) override;
  // I/O is synchronous, so if this is called we're not busy!
  bool isBusy() override; 
  
  void display();
  void noBacklight();
  void backlight();
  
  void command(uint8_t);
  uint16_t getNumCols() { return lcdCols; }
  uint16_t getNumRows() { return lcdRows; }


private:
  void send(uint8_t, uint8_t);
  void write4bits(uint8_t);
  void expanderWrite(uint8_t);
  uint8_t lcdCols=0, lcdRows=0;
  I2CAddress _Addr;
  uint8_t _displayfunction;
  uint8_t _displaycontrol;
  uint8_t _displaymode;
  uint8_t _backlightval = 0;

  uint8_t outputBuffer[4];
  I2CRB rb;
};

#endif
