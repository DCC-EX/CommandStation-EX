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
 *  along with CommandStation-EX.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <Arduino.h>
#include "LiquidCrystal_Parallel.h"
#include "DIAG.h"

LiquidCrystal_Parallel::LiquidCrystal_Parallel(
    uint16_t cols, uint16_t rows,
    uint8_t rs, uint8_t rw, uint8_t enable,
    uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7)
    : _cols(cols), _rows(rows), lcd(rs, rw, enable, d4, d5, d6, d7)
{
}

bool LiquidCrystal_Parallel::begin()
{
    lcd.begin(getNumCols(), getNumRows());
    lcd.noCursor();
    return true;
}

void LiquidCrystal_Parallel::clearNative()
{
    lcd.clear();
}

void LiquidCrystal_Parallel::setRowNative(byte row)
{
    if (row >= getNumRows())
    {
        row = getNumRows() - 1; // we count rows starting w/0
    }
    lcd.setCursor(0, row);
}

size_t LiquidCrystal_Parallel::writeNative(uint8_t value)
{
    return lcd.write(value);
}

bool LiquidCrystal_Parallel::isBusy()
{
    return false;
}
