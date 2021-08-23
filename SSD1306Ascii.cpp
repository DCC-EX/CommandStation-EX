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
 * along with the Arduino SSD1306Ascii Library.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
#include "SSD1306Ascii.h"
#include "I2CManager.h"
#include "FSH.h"

//==============================================================================
// SSD1306/SSD1106 I2C command bytes
//------------------------------------------------------------------------------
/** Set Lower Column Start Address for Page Addressing Mode. */
static const uint8_t SSD1306_SETLOWCOLUMN = 0x00;
/** Set Higher Column Start Address for Page Addressing Mode. */
static const uint8_t SSD1306_SETHIGHCOLUMN = 0x10;
/** Set Memory Addressing Mode. */
static const uint8_t SSD1306_MEMORYMODE = 0x20;
/** Set display RAM display start line register from 0 - 63. */
static const uint8_t SSD1306_SETSTARTLINE = 0x40;
/** Set Display Contrast to one of 256 steps. */
static const uint8_t SSD1306_SETCONTRAST = 0x81;
/** Enable or disable charge pump.  Follow with 0X14 enable, 0X10 disable. */
static const uint8_t SSD1306_CHARGEPUMP = 0x8D;
/** Set Segment Re-map between data column and the segment driver. */
static const uint8_t SSD1306_SEGREMAP = 0xA0;
/** Resume display from GRAM content. */
static const uint8_t SSD1306_DISPLAYALLON_RESUME = 0xA4;
/** Force display on regardless of GRAM content. */
static const uint8_t SSD1306_DISPLAYALLON = 0xA5;
/** Set Normal Display. */
static const uint8_t SSD1306_NORMALDISPLAY = 0xA6;
/** Set Inverse Display. */
static const uint8_t SSD1306_INVERTDISPLAY = 0xA7;
/** Set Multiplex Ratio from 16 to 63. */
static const uint8_t SSD1306_SETMULTIPLEX = 0xA8;
/** Set Display off. */
static const uint8_t SSD1306_DISPLAYOFF = 0xAE;
/** Set Display on. */
static const uint8_t SSD1306_DISPLAYON = 0xAF;
/**Set GDDRAM Page Start Address. */
static const uint8_t SSD1306_SETSTARTPAGE = 0xB0;
/** Set COM output scan direction normal. */
static const uint8_t SSD1306_COMSCANINC = 0xC0;
/** Set COM output scan direction reversed. */
static const uint8_t SSD1306_COMSCANDEC = 0xC8;
/** Set Display Offset. */
static const uint8_t SSD1306_SETDISPLAYOFFSET = 0xD3;
/** Sets COM signals pin configuration to match the OLED panel layout. */
static const uint8_t SSD1306_SETCOMPINS = 0xDA;
/** This command adjusts the VCOMH regulator output. */
static const uint8_t SSD1306_SETVCOMDETECT = 0xDB;
/** Set Display Clock Divide Ratio/ Oscillator Frequency. */
static const uint8_t SSD1306_SETDISPLAYCLOCKDIV = 0xD5;
/** Set Pre-charge Period */
static const uint8_t SSD1306_SETPRECHARGE = 0xD9;
/** Deactivate scroll */
static const uint8_t SSD1306_DEACTIVATE_SCROLL = 0x2E;
/** No Operation Command. */
static const uint8_t SSD1306_NOP = 0xE3;
//------------------------------------------------------------------------------
/** Set Pump voltage value: (30H~33H) 6.4, 7.4, 8.0 (POR), 9.0. */
static const uint8_t SH1106_SET_PUMP_VOLTAGE = 0x30;
/** First byte of set charge pump mode */
static const uint8_t SH1106_SET_PUMP_MODE = 0xAD;
/** Second byte charge pump on. */
static const uint8_t SH1106_PUMP_ON = 0x8B;
/** Second byte charge pump off. */
static const uint8_t SH1106_PUMP_OFF = 0x8A;
//------------------------------------------------------------------------------

// Sequence of blank pixels, to optimise clearing screen.
// Send a maximum of 30 pixels per transmission.
const uint8_t FLASH SSD1306AsciiWire::blankPixels[30] = 
  {0x40,        // First byte specifies data mode
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};  

//==============================================================================
// SSD1306AsciiWire Method Definitions
//------------------------------------------------------------------------------
 
// Constructor
SSD1306AsciiWire::SSD1306AsciiWire(int width, int height) {
  // Set size in characters in base class
  lcdRows = height / 8;
  lcdCols = width / 6;

  // Initialise request block for I2C
  requestBlock.init();

  I2CManager.begin();
  I2CManager.setClock(400000L);  // Set max supported I2C speed
  for (byte address = 0x3c; address <= 0x3d; address++) {
    if (I2CManager.exists(address)) {
      // Device found
      DIAG(F("%dx%d OLED display configured on I2C:x%x"), width, height, address);
      if (width == 132)
        begin(&SH1106_132x64, address);
      else if (height == 32)
        begin(&Adafruit128x32, address);
      else
        begin(&Adafruit128x64, address);
      // Set singleton address
      lcdDisplay = this;
      clear();
      return;
    }
  }
  DIAG(F("OLED display not found"));
}

/* Clear screen by writing blank pixels. */
void SSD1306AsciiWire::clearNative() {
  const int maxBytes = sizeof(blankPixels);  // max number of bytes sendable over Wire
  for (uint8_t r = 0; r <= m_displayHeight/8 - 1; r++) {
    setRowNative(r);   // Position at start of row to be erased
    for (uint8_t c = 0; c <= m_displayWidth - 1; c += maxBytes-1) {
      uint8_t len = min(m_displayWidth-c, maxBytes-1) + 1;
      I2CManager.write_P(m_i2cAddr, blankPixels, len);  // Write a number of blank columns
    }
  }
}

// Initialise device
void SSD1306AsciiWire::begin(const DevType* dev, uint8_t i2cAddr) {
  m_i2cAddr = i2cAddr;
  m_col = 0;
  m_row = 0;
  const uint8_t* table = (const uint8_t*)GETFLASHW(&dev->initcmds);
  uint8_t size = GETFLASH(&dev->initSize);
  m_displayWidth = GETFLASH(&dev->lcdWidth);
  m_displayHeight = GETFLASH(&dev->lcdHeight);
  m_colOffset = GETFLASH(&dev->colOffset);
  I2CManager.write_P(m_i2cAddr, table, size);
  if (m_displayHeight == 32) 
    I2CManager.write(m_i2cAddr, 5, 0, // Set command mode
      SSD1306_SETMULTIPLEX, 0x1F,     // ratio 32
      SSD1306_SETCOMPINS, 0x02);      // sequential COM pins, disable remap
}

//------------------------------------------------------------------------------

// Set cursor position (by text line)
void SSD1306AsciiWire::setRowNative(uint8_t line) {
  // Calculate pixel position from line number
  uint8_t row = line*8;
  if (row < m_displayHeight) {
    m_row = row;
    m_col = m_colOffset;
    // Before using buffer, wait for last request to complete
    requestBlock.wait();
    // Build output buffer for I2C
    uint8_t len = 0;
    outputBuffer[len++] = 0x00;  // Set to command mode
    outputBuffer[len++] = SSD1306_SETLOWCOLUMN | (m_col & 0XF);
    outputBuffer[len++] = SSD1306_SETHIGHCOLUMN | (m_col >> 4);
    outputBuffer[len++] = SSD1306_SETSTARTPAGE | (m_row/8);
    I2CManager.write(m_i2cAddr, outputBuffer, len, &requestBlock);
  }
}
//------------------------------------------------------------------------------

// Write a character to the OLED
size_t SSD1306AsciiWire::writeNative(uint8_t ch) {
  const uint8_t* base = m_font;

  if (ch < m_fontFirstChar || ch >= (m_fontFirstChar + m_fontCharCount))
    return 0;
  // Check if character would be partly or wholly off the display
  if (m_col + fontWidth > m_displayWidth)
    return 0;
#if defined(NOLOWERCASE)
  // Adjust if lowercase is missing
  if (ch >= 'a') {
    if (ch <= 'z')
      ch = ch - 'a' + 'A';  // Capitalise
    else
      ch -= 26; // Allow for missing lowercase letters
  }
#endif
  ch -= m_fontFirstChar;
  base += fontWidth * ch;
  // Before using buffer, wait for last request to complete
  requestBlock.wait();
  // Build output buffer for I2C
  outputBuffer[0] = 0x40;     // set SSD1306 controller to data mode
  uint8_t bufferPos = 1;
  // Copy character pixel columns
  for (uint8_t i = 0; i < fontWidth; i++) 
    outputBuffer[bufferPos++] = GETFLASH(base++);
  // Add blank pixels between letters
  for (uint8_t i = 0; i < letterSpacing; i++) 
    outputBuffer[bufferPos++] = 0;

  // Write the data to I2C display
  I2CManager.write(m_i2cAddr, outputBuffer, bufferPos, &requestBlock);
  m_col += fontWidth + letterSpacing;
  return 1;
}

//==============================================================================
// this section is based on https://github.com/adafruit/Adafruit_SSD1306

/** Initialization commands for a 128x32 or 128x64 SSD1306 oled display. */
const uint8_t FLASH SSD1306AsciiWire::Adafruit128xXXinit[] = {
    // Init sequence for Adafruit 128x32/64 OLED module
    0x00,                              // Set to command mode
    SSD1306_DISPLAYOFF,
    SSD1306_SETDISPLAYCLOCKDIV, 0x80,  // the suggested ratio 0x80 
    SSD1306_SETMULTIPLEX, 0x3F,        // ratio 64 (initially)
    SSD1306_SETDISPLAYOFFSET, 0x0,     // no offset
    SSD1306_SETSTARTLINE | 0x0,        // line #0
    SSD1306_CHARGEPUMP, 0x14,          // internal vcc
    SSD1306_MEMORYMODE, 0x02,          // page mode
    SSD1306_SEGREMAP | 0x1,            // column 127 mapped to SEG0
    SSD1306_COMSCANDEC,                // column scan direction reversed
    SSD1306_SETCOMPINS, 0X12,          // set COM pins
    SSD1306_SETCONTRAST, 0x7F,         // contrast level 127
    SSD1306_SETPRECHARGE, 0xF1,        // pre-charge period (1, 15)
    SSD1306_SETVCOMDETECT, 0x40,       // vcomh regulator level
    SSD1306_DISPLAYALLON_RESUME,
    SSD1306_NORMALDISPLAY,
    SSD1306_DISPLAYON
};

/** Initialize a 128x32 SSD1306 oled display. */
const DevType FLASH SSD1306AsciiWire::Adafruit128x32 = {
  Adafruit128xXXinit,
  sizeof(Adafruit128xXXinit),
  128,
  32,
  0
};

/** Initialize a 128x64 oled display. */
const DevType FLASH SSD1306AsciiWire::Adafruit128x64 = {
  Adafruit128xXXinit,
  sizeof(Adafruit128xXXinit),
  128,
  64,
  0
};
//------------------------------------------------------------------------------
// This section is based on https://github.com/stanleyhuangyc/MultiLCD

/** Initialization commands for a 128x64 SH1106 oled display. */
const uint8_t FLASH SSD1306AsciiWire::SH1106_132x64init[] = {
  0x00,                                  // Set to command mode
  SSD1306_DISPLAYOFF,
  SSD1306_SETDISPLAYCLOCKDIV, 0X80,      // set osc division
  SSD1306_SETMULTIPLEX, 0x3F,            // ratio 64
  SSD1306_SETDISPLAYOFFSET, 0X00,        // set display offset
  SSD1306_SETSTARTPAGE | 0X0,            // set page address
  SSD1306_SETSTARTLINE | 0x0,            // set start line
  SH1106_SET_PUMP_MODE, SH1106_PUMP_ON,  // set charge pump enable
  SSD1306_SEGREMAP | 0X1,                // set segment remap
  SSD1306_COMSCANDEC,                    // Com scan direction
  SSD1306_SETCOMPINS, 0X12,              // set COM pins
  SSD1306_SETCONTRAST, 0x80,             // 128
  SSD1306_SETPRECHARGE, 0X1F,            // set pre-charge period
  SSD1306_SETVCOMDETECT,  0x40,          // set vcomh
  SH1106_SET_PUMP_VOLTAGE | 0X2,         // 8.0 volts
  SSD1306_NORMALDISPLAY,                 // normal / reverse
  SSD1306_DISPLAYON
};

/** Initialize a 132x64 oled SH1106 display. */
const DevType FLASH SSD1306AsciiWire::SH1106_132x64 =  {
  SH1106_132x64init,
  sizeof(SH1106_132x64init),
  128,
  64,
  2    // SH1106 is a 132x64 controller but most OLEDs are only attached
       // to columns 2-129.
};


//------------------------------------------------------------------------------

// Font characters, 5x7 pixels, 0x61 characters starting at 0x20.
// Lower case characters optionally omitted.
const uint8_t FLASH SSD1306AsciiWire::System5x7[] = {

    // Fixed width; char width table not used !!!!
    // or with lowercase character omitted.

    // font data
    0x00, 0x00, 0x00, 0x00, 0x00,  // (space)
    0x00, 0x00, 0x5F, 0x00, 0x00,  // !
    0x00, 0x07, 0x00, 0x07, 0x00,  // "
    0x14, 0x7F, 0x14, 0x7F, 0x14,  // #
    0x24, 0x2A, 0x7F, 0x2A, 0x12,  // $
    0x23, 0x13, 0x08, 0x64, 0x62,  // %
    0x36, 0x49, 0x55, 0x22, 0x50,  // &
    0x00, 0x05, 0x03, 0x00, 0x00,  // '
    0x00, 0x1C, 0x22, 0x41, 0x00,  // (
    0x00, 0x41, 0x22, 0x1C, 0x00,  // )
    0x08, 0x2A, 0x1C, 0x2A, 0x08,  // *
    0x08, 0x08, 0x3E, 0x08, 0x08,  // +
    0x00, 0x50, 0x30, 0x00, 0x00,  // ,
    0x08, 0x08, 0x08, 0x08, 0x08,  // -
    0x00, 0x60, 0x60, 0x00, 0x00,  // .
    0x20, 0x10, 0x08, 0x04, 0x02,  // /
    0x3E, 0x51, 0x49, 0x45, 0x3E,  // 0
    0x00, 0x42, 0x7F, 0x40, 0x00,  // 1
    0x42, 0x61, 0x51, 0x49, 0x46,  // 2
    0x21, 0x41, 0x45, 0x4B, 0x31,  // 3
    0x18, 0x14, 0x12, 0x7F, 0x10,  // 4
    0x27, 0x45, 0x45, 0x45, 0x39,  // 5
    0x3C, 0x4A, 0x49, 0x49, 0x30,  // 6
    0x01, 0x71, 0x09, 0x05, 0x03,  // 7
    0x36, 0x49, 0x49, 0x49, 0x36,  // 8
    0x06, 0x49, 0x49, 0x29, 0x1E,  // 9
    0x00, 0x36, 0x36, 0x00, 0x00,  // :
    0x00, 0x56, 0x36, 0x00, 0x00,  // ;
    0x00, 0x08, 0x14, 0x22, 0x41,  // <
    0x14, 0x14, 0x14, 0x14, 0x14,  // =
    0x41, 0x22, 0x14, 0x08, 0x00,  // >
    0x02, 0x01, 0x51, 0x09, 0x06,  // ?
    0x32, 0x49, 0x79, 0x41, 0x3E,  // @
    0x7E, 0x11, 0x11, 0x11, 0x7E,  // A
    0x7F, 0x49, 0x49, 0x49, 0x36,  // B
    0x3E, 0x41, 0x41, 0x41, 0x22,  // C
    0x7F, 0x41, 0x41, 0x22, 0x1C,  // D
    0x7F, 0x49, 0x49, 0x49, 0x41,  // E
    0x7F, 0x09, 0x09, 0x01, 0x01,  // F
    0x3E, 0x41, 0x41, 0x51, 0x32,  // G
    0x7F, 0x08, 0x08, 0x08, 0x7F,  // H
    0x00, 0x41, 0x7F, 0x41, 0x00,  // I
    0x20, 0x40, 0x41, 0x3F, 0x01,  // J
    0x7F, 0x08, 0x14, 0x22, 0x41,  // K
    0x7F, 0x40, 0x40, 0x40, 0x40,  // L
    0x7F, 0x02, 0x04, 0x02, 0x7F,  // M
    0x7F, 0x04, 0x08, 0x10, 0x7F,  // N
    0x3E, 0x41, 0x41, 0x41, 0x3E,  // O
    0x7F, 0x09, 0x09, 0x09, 0x06,  // P
    0x3E, 0x41, 0x51, 0x21, 0x5E,  // Q
    0x7F, 0x09, 0x19, 0x29, 0x46,  // R
    0x46, 0x49, 0x49, 0x49, 0x31,  // S
    0x01, 0x01, 0x7F, 0x01, 0x01,  // T
    0x3F, 0x40, 0x40, 0x40, 0x3F,  // U
    0x1F, 0x20, 0x40, 0x20, 0x1F,  // V
    0x7F, 0x20, 0x18, 0x20, 0x7F,  // W
    0x63, 0x14, 0x08, 0x14, 0x63,  // X
    0x03, 0x04, 0x78, 0x04, 0x03,  // Y
    0x61, 0x51, 0x49, 0x45, 0x43,  // Z
    0x00, 0x00, 0x7F, 0x41, 0x41,  // [
    0x02, 0x04, 0x08, 0x10, 0x20,  // "\"
    0x41, 0x41, 0x7F, 0x00, 0x00,  // ]
    0x04, 0x02, 0x01, 0x02, 0x04,  // ^
    0x40, 0x40, 0x40, 0x40, 0x40,  // _
    0x00, 0x01, 0x02, 0x04, 0x00,  // `
#ifndef NOLOWERCASE
    0x20, 0x54, 0x54, 0x54, 0x78,  // a
    0x7F, 0x48, 0x44, 0x44, 0x38,  // b
    0x38, 0x44, 0x44, 0x44, 0x20,  // c
    0x38, 0x44, 0x44, 0x48, 0x7F,  // d
    0x38, 0x54, 0x54, 0x54, 0x18,  // e
    0x08, 0x7E, 0x09, 0x01, 0x02,  // f
    0x08, 0x14, 0x54, 0x54, 0x3C,  // g
    0x7F, 0x08, 0x04, 0x04, 0x78,  // h
    0x00, 0x44, 0x7D, 0x40, 0x00,  // i
    0x20, 0x40, 0x44, 0x3D, 0x00,  // j
    0x00, 0x7F, 0x10, 0x28, 0x44,  // k
    0x00, 0x41, 0x7F, 0x40, 0x00,  // l
    0x7C, 0x04, 0x18, 0x04, 0x78,  // m
    0x7C, 0x08, 0x04, 0x04, 0x78,  // n
    0x38, 0x44, 0x44, 0x44, 0x38,  // o
    0x7C, 0x14, 0x14, 0x14, 0x08,  // p
    0x08, 0x14, 0x14, 0x18, 0x7C,  // q
    0x7C, 0x08, 0x04, 0x04, 0x08,  // r
    0x48, 0x54, 0x54, 0x54, 0x20,  // s
    0x04, 0x3F, 0x44, 0x40, 0x20,  // t
    0x3C, 0x40, 0x40, 0x20, 0x7C,  // u
    0x1C, 0x20, 0x40, 0x20, 0x1C,  // v
    0x3C, 0x40, 0x30, 0x40, 0x3C,  // w
    0x44, 0x28, 0x10, 0x28, 0x44,  // x
    0x0C, 0x50, 0x50, 0x50, 0x3C,  // y
    0x44, 0x64, 0x54, 0x4C, 0x44,  // z
#endif
    0x00, 0x08, 0x36, 0x41, 0x00,  // {
    0x00, 0x00, 0x7F, 0x00, 0x00,  // |
    0x00, 0x41, 0x36, 0x08, 0x00,  // }
    0x08, 0x08, 0x2A, 0x1C, 0x08,  // ->
    0x08, 0x1C, 0x2A, 0x08, 0x08,  // <-
    0x00, 0x06, 0x09, 0x09, 0x06   // degree symbol

};
