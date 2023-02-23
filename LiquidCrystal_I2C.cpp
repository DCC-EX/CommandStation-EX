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
 *  along with CommandStation-EX.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <Arduino.h>
#include "LiquidCrystal_I2C.h"
#include "DIAG.h"

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

LiquidCrystal_I2C::LiquidCrystal_I2C(I2CAddress lcd_Addr, uint8_t lcd_cols,
                                     uint8_t lcd_rows) {
  _Addr = lcd_Addr;
  lcdRows = lcd_rows;  // Number of character rows (typically 2 or 4).
  lcdCols = lcd_cols;  // Number of character columns (typically 16 or 20)
  _backlightval = 0;
 }

bool LiquidCrystal_I2C::begin() {

  I2CManager.begin();
  I2CManager.setClock(100000L);    // PCF8574 is spec'd to 100kHz.

  if (I2CManager.exists(_Addr)) {
    DIAG(F("%dx%d LCD configured on I2C:%s"), (int)lcdCols, (int)lcdRows, _Addr.toString());
    _displayfunction = LCD_4BITMODE | LCD_1LINE | LCD_5x8DOTS;
    backlight();
  } else {
    DIAG(F("LCD not found on I2C:%s"), _Addr.toString());
    return false;
  }

  if (lcdRows > 1) {
    _displayfunction |= LCD_2LINE;
  }

  // according to datasheet, we need at least 40ms after power rises above 2.7V
  // before sending commands. Arduino can turn on way before 4.5V so we'll allow
  // 100 milliseconds after pulling both RS and R/W and backlight pin low
  expanderWrite(
      _backlightval);  // reset expander and turn backlight off (Bit 8 =1)
  delay(100);

  // put the LCD into 4 bit mode
  // this is according to the hitachi HD44780 datasheet
  // figure 24, pg 46

  // we start in 8bit mode, try to set 4 bit mode
  write4bits(0x03);
  delayMicroseconds(5000);  // wait min 4.1ms

  // second try
  write4bits(0x03);
  delayMicroseconds(5000);  // wait min 4.1ms

  // third go!
  write4bits(0x03);
  delayMicroseconds(5000);

  // finally, set to 4-bit interface
  write4bits(0x02);

  // set # lines, font size, etc.
  command(LCD_FUNCTIONSET | _displayfunction);

  // turn the display on with no cursor or blinking default
  _displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
  display();

  // Initialize to default text direction (for roman languages)
  _displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;

  // set the entry mode
  command(LCD_ENTRYMODESET | _displaymode);

  return true;
}

/********** high level commands, for the user! */
void LiquidCrystal_I2C::clearNative() {
  command(LCD_CLEARDISPLAY);  // clear display, set cursor position to zero
  delayMicroseconds(2000);    // this command takes 1.52ms but allow plenty
}

void LiquidCrystal_I2C::setRowNative(byte row) {
  uint8_t row_offsets[] = {0x00, 0x40, 0x14, 0x54};
  if (row >= lcdRows) {
    row = lcdRows - 1;  // we count rows starting w/0
  }
  command(LCD_SETDDRAMADDR | (row_offsets[row]));
}

void LiquidCrystal_I2C::display() {
  _displaycontrol |= LCD_DISPLAYON;
  command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// Turn the (optional) backlight off/on
void LiquidCrystal_I2C::noBacklight(void) {
  _backlightval &= ~LCD_BACKLIGHT;
  expanderWrite(0);
}

void LiquidCrystal_I2C::backlight(void) {
  _backlightval = LCD_BACKLIGHT;
  expanderWrite(0);
}

size_t LiquidCrystal_I2C::writeNative(uint8_t value) {
  send(value, Rs);
  return 1;
}

bool LiquidCrystal_I2C::isBusy() { 
  return rb.isBusy();
}

/*********** mid level commands, for sending data/cmds */

inline void LiquidCrystal_I2C::command(uint8_t value) { 
  send(value, 0); 
}

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
 * 
 * Similarly, the Enable must be set/reset for at least 450ns.  This is 
 * well within the I2C clock cycle time of 2.5us at 400kHz.  Data is clocked in
 * to the HD44780 on the trailing edge of the Enable pin, so we set the Enable
 * as we present the data, then in the next byte we reset Enable without changing
 * the data.
 */

// write either command or data (8 bits) to the HD44780 LCD controller as
//  a single I2C transmission. 
void LiquidCrystal_I2C::send(uint8_t value, uint8_t mode) {
  mode |= _backlightval;
  uint8_t highnib = (((value >> 4) & 0x0f) << BACKPACK_DATA_BITS) | mode;
  uint8_t lownib = ((value & 0x0f) << BACKPACK_DATA_BITS) | mode;
  // Send both nibbles
  uint8_t len = 0;
  rb.wait();
  outputBuffer[len++] = highnib|En;
  outputBuffer[len++] = highnib;
  outputBuffer[len++] = lownib|En;
  outputBuffer[len++] = lownib;
  I2CManager.write(_Addr, outputBuffer, len, &rb);  // Write command asynchronously
}

// write 4 data bits to the HD44780 LCD controller.
void LiquidCrystal_I2C::write4bits(uint8_t value) {
  uint8_t _data = ((value & 0x0f) << BACKPACK_DATA_BITS) | _backlightval;
  // Enable must be set/reset for at least 450ns.  This is well within the
  // I2C clock cycle time of 2.5us at 400kHz. Data is clocked in to the
  // HD44780 on the trailing edge of the Enable pin.
  uint8_t len = 0;
  rb.wait();
  outputBuffer[len++] = _data|En;
  outputBuffer[len++] = _data;
  I2CManager.write(_Addr, outputBuffer, len, &rb);  // Write command asynchronously
}

// write a byte to the PCF8574 I2C interface.  We don't need to set
// the enable pin for this.
void LiquidCrystal_I2C::expanderWrite(uint8_t value) {
  rb.wait();
  outputBuffer[0] = value | _backlightval;
  I2CManager.write(_Addr, outputBuffer, 1, &rb);  // Write command asynchronously
}