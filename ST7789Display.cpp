/*
 *  © 2026, Paul Antoine
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

#include "defines.h"

#if defined(ST7789_DRIVER) || (__has_include(<Adafruit_GFX.h>) && __has_include(<Adafruit_ST7789.h>))

#include "ST7789Display.h"
#include "DIAG.h"
#include <stdlib.h>
#include <string.h>

ST7789Display::ST7789Display(int8_t cs, int8_t dc, int8_t rst,
                             uint16_t width, uint16_t height,
                             uint8_t rotation, uint8_t textSize,
                             int8_t backlightPin,
                             uint8_t backlightActiveLevel,
                             uint16_t textColour,
                             uint16_t backgroundColour)
  : _display(cs, dc, rst),
    _width(width),
    _height(height),
    _textColour(textColour),
    _backgroundColour(backgroundColour),
    _rotation(rotation),
    _textSize(textSize),
    _backlightPin(backlightPin),
    _backlightActiveLevel(backlightActiveLevel) {
  _textSize = calculateTextSize(textSize);
}

bool ST7789Display::begin() {
  if (_backlightPin >= 0) {
    pinMode(_backlightPin, OUTPUT);
    digitalWrite(_backlightPin, _backlightActiveLevel);
  }
  SPI.begin();
  _display.init(_width, _height);
#if defined(ST7789_SPI_SPEED)
  _display.setSPISpeed(ST7789_SPI_SPEED);
#endif
  _display.setRotation(_rotation);
  _display.setTextWrap(false);
  _display.setTextSize(_textSize);
  _display.setTextColor(_textColour, _backgroundColour);
  _display.fillScreen(_backgroundColour);
  _display.setCursor(0, 0);
  resetCellCache();
  _row = 0;
  _col = 0;
  _begun = true;
  DIAG(F("%dx%d ST7789 TFT configured on SPI"), displayWidth(), displayHeight());
  return true;
}

void ST7789Display::clearNative() {
  if (!_begun) return;
  _display.fillScreen(_backgroundColour);
  _display.setCursor(0, 0);
  resetCellCache();
  _row = 0;
  _col = 0;
}

void ST7789Display::setRowNative(uint8_t line) {
  if (!_begun) return;
  uint16_t rows = getNumRows();
  if (rows == 0) return;
  if (line >= rows) line = rows - 1;

  _row = line;
  _col = 0;
  uint16_t y = _row * cellHeight();
  _display.setCursor(0, y);
}

size_t ST7789Display::writeNative(uint8_t c) {
  uint16_t cols = getNumCols();
  if (!_begun || _col >= cols || _row >= getNumRows()) return 0;

  uint16_t cacheIndex = _row * cols + _col;
  if (_cellCache && cacheIndex < _cacheSize && _cellCache[cacheIndex] == (char)c) {
    _col++;
    return 1;
  }

  _display.setCursor(_col * cellWidth(), _row * cellHeight());
  _display.write(c);
  if (_cellCache && cacheIndex < _cacheSize)
    _cellCache[cacheIndex] = (char)c;
  _col++;
  return 1;
}

bool ST7789Display::isBusy() {
  return false;
}

uint8_t ST7789Display::calculateTextSize(uint8_t requestedTextSize) {
  if (requestedTextSize) return requestedTextSize;

  uint16_t selectedSize = 0xFFFF;
  bool constrained = false;

#if defined(ST7789_TEXT_COLS)
  if (ST7789_TEXT_COLS > 0) {
    uint16_t sizeForCols = displayWidth() / (ST7789_TEXT_COLS * 6);
    if (sizeForCols < selectedSize) selectedSize = sizeForCols;
    constrained = true;
  }
#endif

#if defined(ST7789_TEXT_ROWS)
  if (ST7789_TEXT_ROWS > 0) {
    uint16_t rowSpacing = 0;
#if defined(ST7789_TEXT_ROW_SPACING)
    rowSpacing = ST7789_TEXT_ROW_SPACING;
#endif
    uint16_t spacingHeight = ST7789_TEXT_ROWS * rowSpacing;
    uint16_t availableHeight = spacingHeight < displayHeight()
      ? displayHeight() - spacingHeight
      : 0;
    uint16_t sizeForRows = availableHeight / (ST7789_TEXT_ROWS * 8);
    if (sizeForRows < selectedSize) selectedSize = sizeForRows;
    constrained = true;
  }
#endif

  if (!constrained) selectedSize = 2;
  if (selectedSize < 1) selectedSize = 1;
  if (selectedSize > 255) selectedSize = 255;
  return selectedSize;
}

uint16_t ST7789Display::getNumCols() {
  return displayWidth() / cellWidth();
}

uint16_t ST7789Display::getNumRows() {
  return displayHeight() / cellHeight();
}

uint16_t ST7789Display::cellWidth() {
  return 6 * _textSize;
}

uint16_t ST7789Display::cellHeight() {
  uint16_t height = 8 * _textSize;
#if defined(ST7789_TEXT_ROW_SPACING)
  height += ST7789_TEXT_ROW_SPACING;
#endif
  return height;
}

uint16_t ST7789Display::displayWidth() {
  return (_rotation & 1) ? _height : _width;
}

uint16_t ST7789Display::displayHeight() {
  return (_rotation & 1) ? _width : _height;
}

void ST7789Display::resetCellCache() {
  uint16_t rows = getNumRows();
  uint16_t cols = getNumCols();
  uint16_t cacheSize = rows * cols;

  if (cacheSize == 0) return;
  if (!_cellCache || _cacheSize != cacheSize) {
    char *newCache = (char *)realloc(_cellCache, cacheSize);
    if (!newCache) {
      return;
    }
    _cellCache = newCache;
    _cacheSize = cacheSize;
  }
  memset(_cellCache, ' ', _cacheSize);
}

#endif // ST7789_DRIVER or Adafruit ST7789 library available
