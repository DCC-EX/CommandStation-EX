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

#include "LiquidCrystal_I2C.h"
#include "I2CManager.h"

#include <inttypes.h>
#if defined(ARDUINO) && ARDUINO >= 100

#include "Arduino.h"

#define printIIC(args) Wire.write(args)
inline size_t LiquidCrystal_I2C::write(uint8_t value) {
  send(value, Rs);
  return 1;
}

#else
#include "WProgram.h"

#define printIIC(args) Wire.send(args)
inline void LiquidCrystal_I2C::write(uint8_t value) { send(value, Rs); }

#endif
#include "Wire.h"

// When the display powers up, it is configured as follows:
//
// 1. Display clear
// 2. Function set:
//    DL = 1; 8-bit interface data
//    N = 0; 1-line display
//    F = 0; 5x8 dot character font
// 3. Display on/off control:
//    D = 0; Display off
//    C = 0; Cursor off
//    B = 0; Blinking off
// 4. Entry mode set:
//    I/D = 1; Increment by 1
//    S = 0; No shift
//
// Note, however, that resetting the Arduino doesn't reset the LCD, so we
// can't assume that its in that state when a sketch starts (and the
// LiquidCrystal constructor is called).

LiquidCrystal_I2C::LiquidCrystal_I2C(uint8_t lcd_Addr, uint8_t lcd_cols,
                                     uint8_t lcd_rows) {
  _Addr = lcd_Addr;
  _cols = lcd_cols;
  _rows = lcd_rows;
  _backlightval = LCD_NOBACKLIGHT;
}

void LiquidCrystal_I2C::init() { init_priv(); }

void LiquidCrystal_I2C::init_priv() {
  I2CManager.begin();
  I2CManager.setClock(100000L);    // PCF8574 is limited to 100kHz.

  _displayfunction = LCD_4BITMODE | LCD_1LINE | LCD_5x8DOTS;
  begin(_cols, _rows);
}

void LiquidCrystal_I2C::begin(uint8_t cols, uint8_t lines) {
  if (lines > 1) {
    _displayfunction |= LCD_2LINE;
  }
  _numlines = lines;
  (void)_cols;

  // SEE PAGE 45/46 FOR INITIALIZATION SPECIFICATION!
  // according to datasheet, we need at least 40ms after power rises above 2.7V
  // before sending commands. Arduino can turn on way befer 4.5V so we'll allow
  // 100 milliseconds after pulling  both RS and R/W and backlight pin low
  expanderWrite(
      _backlightval);  // reset expander and turn backlight off (Bit 8 =1)
  delay(100);

  // put the LCD into 4 bit mode
  // this is according to the hitachi HD44780 datasheet
  // figure 24, pg 46

  // we start in 8bit mode, try to set 4 bit mode
  write4bits(0x03 << 4);
  delayMicroseconds(4500);  // wait min 4.1ms

  // second try
  write4bits(0x03 << 4);
  delayMicroseconds(4500);  // wait min 4.1ms

  // third go!
  write4bits(0x03 << 4);
  delayMicroseconds(150);

  // finally, set to 4-bit interface
  write4bits(0x02 << 4);

  // set # lines, font size, etc.
  command(LCD_FUNCTIONSET | _displayfunction);

  // turn the display on with no cursor or blinking default
  _displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
  display();

  // clear it off
  clear();

  // Initialize to default text direction (for roman languages)
  _displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;

  // set the entry mode
  command(LCD_ENTRYMODESET | _displaymode);

  setCursor(0, 0);
}

/********** high level commands, for the user! */
void LiquidCrystal_I2C::clear() {
  command(LCD_CLEARDISPLAY);  // clear display, set cursor position to zero
  delayMicroseconds(2000);    // this command takes 1.52ms
}

void LiquidCrystal_I2C::setCursor(uint8_t col, uint8_t row) {
  int row_offsets[] = {0x00, 0x40, 0x14, 0x54};
  if (row > _numlines) {
    row = _numlines - 1;  // we count rows starting w/0
  }
  command(LCD_SETDDRAMADDR | (col + row_offsets[row]));
}

// Turn the display on/off (quickly)
void LiquidCrystal_I2C::noDisplay() {
  _displaycontrol &= ~LCD_DISPLAYON;
  command(LCD_DISPLAYCONTROL | _displaycontrol);
}

void LiquidCrystal_I2C::display() {
  _displaycontrol |= LCD_DISPLAYON;
  command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// Turn the (optional) backlight off/on
void LiquidCrystal_I2C::noBacklight(void) {
  _backlightval = LCD_NOBACKLIGHT;
  expanderWrite(0);
}

void LiquidCrystal_I2C::backlight(void) {
  _backlightval = LCD_BACKLIGHT;
  expanderWrite(0);
}

void LiquidCrystal_I2C::setBacklight(uint8_t new_val) {
  if (new_val) {
    backlight();  // turn backlight on
  } else {
    noBacklight();  // turn backlight off
  }
}

void LiquidCrystal_I2C::printstr(const char c[]) {
  // This function is not identical to the function used for "real" I2C displays
  // it's here so the user sketch doesn't have to be changed
  print(c);
}

/*********** mid level commands, for sending data/cmds */

inline void LiquidCrystal_I2C::command(uint8_t value) { send(value, 0); }

/************ low level data pushing commands **********/

/* According to the NXP Datasheet for the PCF8574 section 8.2:
 *   "The master (microcontroller) sends the START condition and slave address
 *    setting the last bit of the address byte to logic 0 for the write mode.
 *    The PCF8574/74A acknowledges and the master then sends the data byte for
 *    P7 to P0 to the port register. As the clock line goes HIGH, the 8-bit
 *    data is presented on the port lines after it has been acknowledged by the
 *    PCF8574/74A. [...] The master can then send a STOP or ReSTART condition
 *    or continue sending data. The number of data bytes that can be sent 
 *    successively is not limited and the previous data is overwritten every
 *    time a data byte has been sent and acknowledged."
 * 
 * This driver takes advantage of this by sending multiple data bytes in succession
 * within a single I2C transmission.  With a fast clock rate of 400kHz, the time
 * between successive updates of the PCF8574 outputs will be at least 2.5us.  With
 * the default clock rate of 100kHz the time between updates will be at least 10us.
 * 
 * The LCD controller HD44780, according to its datasheet, needs nominally 37us
 * (up to 50us) to execute a command (i.e. write to gdram, reposition, etc.). Each
 * command is sent in a separate I2C transmission here.  The time taken to end a 
 * transmission and start another one is a stop bit, a start bit, 8 address bits,
 * an ack, 8 data bits and another ack; this is at least 20 bits, i.e. >50us
 * at 400kHz and >200us at 100kHz. Therefore, we don't need additional delay.
 */

// write either command or data (8 bits) to the HD44780 as
//  a single I2C transmission.
void LiquidCrystal_I2C::send(uint8_t value, uint8_t mode) {
  uint8_t highnib = value & 0xf0;
  uint8_t lownib = (value << 4) & 0xf0;
  // Send both nibbles
  Wire.beginTransmission(_Addr);
  write4bits(highnib | mode, true);
  write4bits(lownib | mode, true);
  Wire.endTransmission();
}

// write 4 bits to the HD44780 interface.  If inTransmission is false
//   then the nibble will be sent in its own I2C transmission.
void LiquidCrystal_I2C::write4bits(uint8_t value, bool inTransmission) {
  int _data = (int)value | _backlightval;
  if (!inTransmission) Wire.beginTransmission(_Addr);
  // Enable must be set/reset for at least 450ns.  This is well within the
  // I2C clock cycle time of 2.5us at 400kHz. Data is clocked in to the
  // HD44780 on the trailing edge of the Enable pin.
  printIIC(_data | En);
  printIIC(_data);
  if (!inTransmission) Wire.endTransmission();
}

// write a byte to the PCF8574 I2C interface
void LiquidCrystal_I2C::expanderWrite(uint8_t value) {
  int _data = (int)value | _backlightval;
  Wire.beginTransmission(_Addr);
  printIIC(_data);
  Wire.endTransmission();
}