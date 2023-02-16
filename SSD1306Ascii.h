/* Based on Arduino SSD1306Ascii Library, Copyright (C) 2015 by William Greiman
 * Modifications (C) 2021 Neil McKechnie
 *
 *  This file is part of CommandStation-EX
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
 * along with this software.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

#ifndef SSD1306Ascii_h
#define SSD1306Ascii_h

#include "Arduino.h"
#include "FSH.h"
#include "Display.h"

#include "I2CManager.h"
#include "DIAG.h"
#include "DisplayInterface.h"

// Uncomment to remove lower-case letters to save 108 bytes of flash
//#define NOLOWERCASE



//------------------------------------------------------------------------------
// Constructor
class SSD1306AsciiWire : public DisplayDevice {
 public:

  // Constructors
  SSD1306AsciiWire(int width, int height); // Auto-detects I2C address
  SSD1306AsciiWire(I2CAddress address, int width, int height);  

  // Initialize the display controller.
  bool begin();

  // Clear the display and set the cursor to (0, 0).
  void clearNative() override;

  // Set cursor to start of specified text line
  void setRowNative(byte line) override;
  
  // Write one character to OLED
  size_t writeNative(uint8_t c) override;

  bool isBusy() override { return requestBlock.isBusy(); }
  uint16_t getNumCols() { return m_charsPerRow; }
  uint16_t getNumRows() { return m_charsPerColumn; }

 private:
  // Cursor column.
  uint8_t m_col;
  // Cursor RAM row.
  uint8_t m_row;
  // Display width.
  uint8_t m_displayWidth;
  // Display height.
  uint8_t m_displayHeight;
  // Display width in characters
  uint8_t m_charsPerRow;
  // Display height in characters
  uint8_t m_charsPerColumn;
  // Column offset RAM to SEG.
  uint8_t m_colOffset = 0;
  // Current font.
  const uint8_t* const m_font = System6x8;
  // Flag to prevent calling begin() twice
  uint8_t m_initialised = false;

  // Only fixed size 6x8 fonts in a 6x8 cell are supported.
  static const uint8_t fontWidth = 6;
  static const uint8_t fontHeight = 8;
  static const uint8_t m_fontFirstChar = 0x20;
  static const uint8_t m_fontCharCount;

  I2CAddress m_i2cAddr = 0;

  I2CRB requestBlock;
  uint8_t outputBuffer[fontWidth+1];

  static const uint8_t blankPixels[];

  static const uint8_t System6x8[];
  static const uint8_t FLASH Adafruit128xXXinit[];
  static const uint8_t FLASH SH1106_132x64init[];
};

#endif  // SSD1306Ascii_h
