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

#ifndef ST7789Display_h
#define ST7789Display_h

#include "defines.h"

#if defined(ST7789_DRIVER) || (__has_include(<Adafruit_GFX.h>) && __has_include(<Adafruit_ST7789.h>))

#include <Arduino.h>
#if !__has_include(<Adafruit_GFX.h>) || !__has_include(<Adafruit_ST7789.h>)
#error ST7789_DRIVER requires the Adafruit GFX and Adafruit ST7735/ST7789 libraries.
#endif
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>
#include "DisplayInterface.h"

class ST7789Display : public DisplayDevice {
public:
  ST7789Display(int8_t cs, int8_t dc, int8_t rst,
                uint16_t width, uint16_t height,
                uint8_t rotation = 0, uint8_t textSize = 2,
                int8_t backlightPin = -1,
                uint8_t backlightActiveLevel = HIGH,
                uint16_t textColour = ST77XX_WHITE,
                uint16_t backgroundColour = ST77XX_BLACK);

  bool begin() override;
  void clearNative() override;
  void setRowNative(uint8_t line) override;
  size_t writeNative(uint8_t c) override;
  bool isBusy() override;
  uint16_t getNumCols() override;
  uint16_t getNumRows() override;

private:
  uint8_t calculateTextSize(uint8_t requestedTextSize);
  uint16_t cellWidth();
  uint16_t cellHeight();
  uint16_t displayWidth();
  uint16_t displayHeight();
  void resetCellCache();

  Adafruit_ST7789 _display;
  uint16_t _width;
  uint16_t _height;
  uint16_t _textColour;
  uint16_t _backgroundColour;
  uint8_t _rotation;
  uint8_t _textSize;
  int8_t _backlightPin;
  uint8_t _backlightActiveLevel;
  uint16_t _row = 0;
  uint16_t _col = 0;
  char *_cellCache = NULL;
  uint16_t _cacheSize = 0;
  bool _begun = false;
};

#endif // ST7789_DRIVER or Adafruit ST7789 library available
#endif // ST7789Display_h
