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
#include "SSD1306Ascii.h"
#include "I2CManager.h"
#include "FSH.h"


// Maximum number of bytes we can send per transmission is 32.
const uint8_t FLASH SSD1306AsciiWire::blankPixels[32] = 
  {0x40,        // First byte specifies data mode
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};  

//==============================================================================
// SSD1306AsciiWire Method Definitions
//------------------------------------------------------------------------------
void SSD1306AsciiWire::clear() {
  clear(0, displayWidth() - 1, 0, displayRows() - 1);
}
//------------------------------------------------------------------------------
void SSD1306AsciiWire::clear(uint8_t columnStart, uint8_t columnEnd, 
                             uint8_t rowStart, uint8_t rowEnd) {
  const int maxBytes = sizeof(blankPixels);  // max number of bytes sendable over Wire
  // Ensure only rows on display will be cleared.
  if (rowEnd >= displayRows()) rowEnd = displayRows() - 1;
  for (uint8_t r = rowStart; r <= rowEnd; r++) {
    setCursor(columnStart, r);   // Position at start of row to be erased
    for (uint8_t c = columnStart; c <= columnEnd; c += maxBytes-1) {
      uint8_t len = min((uint8_t)(columnEnd-c+1), maxBytes-1) + 1;
      I2CManager.write_P(m_i2cAddr, blankPixels, len);  // Write up to 31 blank columns
    }
  }
}
//------------------------------------------------------------------------------
void SSD1306AsciiWire::begin(const DevType* dev, uint8_t i2cAddr) {
  m_i2cAddr = i2cAddr;
  m_col = 0;
  m_row = 0;
#ifdef __AVR__
  const uint8_t* table = (const uint8_t*)pgm_read_word(&dev->initcmds);
#else   // __AVR__
  const uint8_t* table = dev->initcmds;
#endif  // __AVR
  uint8_t size = readFontByte(&dev->initSize);
  m_displayWidth = readFontByte(&dev->lcdWidth);
  m_displayHeight = readFontByte(&dev->lcdHeight);
  m_colOffset = readFontByte(&dev->colOffset);
  I2CManager.write_P(m_i2cAddr, table, size);
}
//------------------------------------------------------------------------------
void SSD1306AsciiWire::setContrast(uint8_t value) {
  I2CManager.write(m_i2cAddr, 2, 
    0x00,     // Set to command mode
    SSD1306_SETCONTRAST, value);
}
//------------------------------------------------------------------------------
void SSD1306AsciiWire::setCursor(uint8_t col, uint8_t row) {
  if (row < displayRows() && col < m_displayWidth) {
    m_row = row;
    m_col = col + m_colOffset;
    I2CManager.write(m_i2cAddr, 4,
      0x00,    // Set to command mode
      SSD1306_SETLOWCOLUMN | (col & 0XF), 
      SSD1306_SETHIGHCOLUMN | (col >> 4),
      SSD1306_SETSTARTPAGE | m_row);
  }
}
//------------------------------------------------------------------------------
void SSD1306AsciiWire::setFont(const uint8_t* font) {
  m_font = font;
  m_fontFirstChar = readFontByte(m_font + FONT_FIRST_CHAR);
  m_fontCharCount = readFontByte(m_font + FONT_CHAR_COUNT);
}
//------------------------------------------------------------------------------
size_t SSD1306AsciiWire::write(uint8_t ch) {
  const uint8_t* base = m_font + FONT_WIDTH_TABLE;

  if (ch < m_fontFirstChar || ch >= (m_fontFirstChar + m_fontCharCount))
    return 0;
  ch -= m_fontFirstChar;
  base += fontWidth * ch;
  uint8_t buffer[1+fontWidth+letterSpacing];
  buffer[0] = 0x40;     // set SSD1306 controller to data mode
  uint8_t bufferPos = 1;
  // Copy character pixel columns
  for (uint8_t i = 0; i < fontWidth; i++) 
    buffer[bufferPos++] = readFontByte(base++);
  // Add blank pixels between letters
  for (uint8_t i = 0; i < letterSpacing; i++) 
    buffer[bufferPos++] = 0;
  // Write the data to I2C display
  I2CManager.write(m_i2cAddr, buffer, bufferPos);
  return 1;
}
