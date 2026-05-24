/*
 *  © 2024, Oskar Senft. All rights reserved.
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

#ifndef LiquidCrystal_Parallel_h
#define LiquidCrystal_Parallel_h

#include <Arduino.h>
#include "Display.h"

#ifdef PARALLEL_LCD_DRIVER
// Only use the Arduino library if the driver is actually enabled.
#include <LiquidCrystal.h>
#else
// If the driver is not enabled, use a dummy version instead.
class LiquidCrystal
{
public:
    LiquidCrystal(uint8_t rs, uint8_t rw, uint8_t enable,
                  uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7) {};
    void begin(uint16_t cols, uint16_t rows) {};
    void noCursor() {};
    void setCursor(uint16_t col, uint16_t row) {};
    void clear() {};
    size_t write(uint8_t val) { return 0; };
};
#endif

// Support for an LCD based on the Hitachi HD44780 (or a compatible) chipset
// as supported by Arduino's LiquidCrystal library.
class LiquidCrystal_Parallel : public DisplayDevice
{
public:
    // Specify the display's number of columns and rows as well
    // as Arduino pins numbers for the display's pins (4-bit mode)
    LiquidCrystal_Parallel(uint16_t cols, uint16_t rows,
                           uint8_t rs, uint8_t rw, uint8_t enable,
                           uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7);
    bool begin() override;
    void clearNative() override;
    void setRowNative(byte line) override;
    size_t writeNative(uint8_t c) override;
    bool isBusy() override;

    uint16_t getNumCols() override { return _cols; }
    uint16_t getNumRows() override { return _rows; }

private:
    LiquidCrystal lcd;
    const uint16_t _cols;
    const uint16_t _rows;
};

#endif
