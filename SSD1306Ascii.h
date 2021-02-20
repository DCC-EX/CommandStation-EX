/* Based on Arduino SSD1306Ascii Library, Copyright (C) 2015 by William Greiman
 * Modifications (C) 2021 Neil McKechnie
 *
 * This Library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This Library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the Arduino SSD1306Ascii Library.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file SSD1306AsciiWire.h
 * @brief Class for I2C displays using Wire.
 */
#ifndef SSD1306AsciiWire_h
#define SSD1306AsciiWire_h

#include <Wire.h>
#include "Arduino.h"
#include "SSD1306init.h"
#include "SSD1306font.h"

//------------------------------------------------------------------------------
/** SSD1306Ascii version basis */
#define SDD1306_ASCII_VERSION 1.3.0
//------------------------------------------------------------------------------
// Configuration options.

/** Use larger faster I2C code. */
#define OPTIMIZE_I2C 1

//------------------------------------------------------------------------------
// Values for writeDisplay() mode parameter.
/** Write to Command register. */
#define SSD1306_MODE_CMD     0
/** Write one byte to display RAM. */
#define SSD1306_MODE_RAM     1
/** Write to display RAM with possible buffering. */
#define SSD1306_MODE_RAM_BUF 2
//------------------------------------------------------------------------------

/**
 * @class SSD1306Ascii
 * @brief SSD1306 base class
 */
class SSD1306Ascii : public Print {
 public:
  using Print::write;
  SSD1306Ascii() {}
  /**
   * @brief Clear the display and set the cursor to (0, 0).
   */
  void clear();
  /**
   * @brief Clear a region of the display.
   *
   * @param[in] c0 Starting column.
   * @param[in] c1 Ending column.
   * @param[in] r0 Starting row;
   * @param[in] r1 Ending row;
   * @note The final cursor position will be (c0, r0).
   */
  void clear(uint8_t c0, uint8_t c1, uint8_t r0, uint8_t r1);
  /**
   * @brief Clear a field of n fieldWidth() characters.
   *
   * @param[in] col Field start column.
   *
   * @param[in] row Field start row.
   *
   * @param[in] n Number of characters in the field.
   *
   */
  void clearField(uint8_t col, uint8_t row, uint8_t n);
  /**
   * @brief Clear the display to the end of the current line.
   * @note The number of rows cleared will be determined by the height
   *       of the current font.
   * @note The cursor will be returned to the original position.
   */
  void clearToEOL();
  /**
   * @return The current column in pixels.
   */
  inline uint8_t col() const {return m_col;}
  /**
   * @return The display hight in pixels.
   */
  inline uint8_t displayHeight() const {return m_displayHeight;}
  /**
   * @brief Set display to normal or 180 degree remap mode.
   *
   * @param[in] mode true for normal mode, false for remap mode.
   *
   * @note Adafruit and many ebay displays use remap mode.
   *       Use normal mode to rotate these displays 180 degrees.
   */
  void displayRemap(bool mode);
  /**
   * @return The display height in rows with eight pixels to a row.
   */
  inline uint8_t displayRows() const {return m_displayHeight/8;}
  /**
   * @return The display width in pixels.
   */
  inline uint8_t displayWidth() const {return m_displayWidth;}
  /**
   * @brief Set the cursor position to (0, 0).
   */
  inline void home() {setCursor(0, 0);}
  /**
   * @brief Initialize the display controller.
   *
   * @param[in] dev A display initialization structure.
   */
  void init(const DevType* dev);
  /**
   * @return the current row number with eight pixels to a row.
   */
  inline uint8_t row() const {return m_row;}
  /**
   * @brief Set the current column number.
   *
   * @param[in] col The desired column number in pixels.
   */
  void setCol(uint8_t col);
  /**
   * @brief Set the display contrast.
   *
   * @param[in] value The contrast level in th range 0 to 255.
   */
  void setContrast(uint8_t value);
  /**
   * @brief Set the cursor position.
   *
   * @param[in] col The column number in pixels.
   * @param[in] row the row number in eight pixel rows.
   */
  void setCursor(uint8_t col, uint8_t row);
  /**
   * @brief Set the current font.
   *
   * @param[in] font Pointer to a font table.
   */
  void setFont(const uint8_t* font);
  /**
   * @brief Set the current row number.
   *
   * @param[in] row the row number in eight pixel rows.
   */
  void setRow(uint8_t row);
  /**
   * @brief Write a command byte to the display controller.
   *
   * @param[in] c The command byte.
   * @note The byte will immediately be sent to the controller.
   */
  inline void ssd1306WriteCmd(uint8_t c) {writeDisplay(c, SSD1306_MODE_CMD);}
  /**
   * @brief Write a byte to RAM in the display controller.
   *
   * @param[in] c The data byte.
   * @note The byte will immediately be sent to the controller.
   */
  void ssd1306WriteRam(uint8_t c);
  /**
   * @brief Write a byte to RAM in the display controller.
   *
   * @param[in] c The data byte.
   * @note The byte may be buffered until a call to ssd1306WriteCmd
   *       or ssd1306WriteRam or endWrite.
   */
  void ssd1306WriteRamBuf(uint8_t c);
   /**
   * @brief Display a character.
   *
   * @param[in] c The character to display.
   * @return one for success else zero.
   */
  size_t write(uint8_t c);

 protected:
  virtual void writeDisplay(uint8_t b, uint8_t mode) = 0;
  virtual void flushDisplay() = 0;
  uint8_t m_col;            // Cursor column.
  uint8_t m_row;            // Cursor RAM row.
  uint8_t m_displayWidth;   // Display width.
  uint8_t m_displayHeight;  // Display height.
  uint8_t m_colOffset;      // Column offset RAM to SEG.
  const uint8_t* m_font = nullptr;  // Current font.
};


/**
 * @class SSD1306AsciiWire
 * @brief Class for I2C displays using Wire.
 */
class SSD1306AsciiWire : public SSD1306Ascii {
 public:
#define m_oledWire Wire
  /**
   * @brief Initialize the display controller.
   *
   * @param[in] dev A device initialization structure.
   * @param[in] i2cAddr The I2C address of the display controller.
   */
  void begin(const DevType* dev, uint8_t i2cAddr) {
#if OPTIMIZE_I2C
    m_nData = 0;
#endif  // OPTIMIZE_I2C
    m_i2cAddr = i2cAddr;
    init(dev);
  }
  
 protected:
  void writeDisplay(uint8_t b, uint8_t mode) {
#if OPTIMIZE_I2C
    if (m_nData > 16 || (m_nData && mode == SSD1306_MODE_CMD)) {
      m_oledWire.endTransmission();
      m_nData = 0;
    }
    if (m_nData == 0) {
      m_oledWire.beginTransmission(m_i2cAddr);
      m_oledWire.write(mode == SSD1306_MODE_CMD ? 0X00 : 0X40);
    }
    m_oledWire.write(b);
    if (mode == SSD1306_MODE_RAM_BUF) {
      m_nData++;
    } else {
      m_oledWire.endTransmission();
      m_nData = 0;
    }
#else  // OPTIMIZE_I2C
    m_oledWire.beginTransmission(m_i2cAddr);
    m_oledWire.write(mode == SSD1306_MODE_CMD ? 0X00: 0X40);
    m_oledWire.write(b);
    m_oledWire.endTransmission();
#endif    // OPTIMIZE_I2C
  }

  void flushDisplay() {
#if OPTIMIZE_I2C
    if (m_nData) {
      m_oledWire.endTransmission();
      m_nData = 0;
    }
#endif // OPTIMIZE_I2C
  }

 protected:
  uint8_t m_i2cAddr;
#if OPTIMIZE_I2C
  uint8_t m_nData;
#endif  // OPTIMIZE_I2C
};
#endif  // SSD1306AsciiWire_h
