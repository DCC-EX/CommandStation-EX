/*
 *  © 2021, Chris Harlow, Neil McKechnie. All rights reserved.
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
  LiquidCrystal_I2C LCDDriver(LCD_DRIVER);  // set the LCD address, cols, rows 
  // DEVICE SPECIFIC LCDDisplay Implementation for LCD_DRIVER
  LCDDisplay::LCDDisplay() { 
    lcdDisplay=this;
    LCDDriver.init();
    LCDDriver.backlight();
    interfake(LCD_DRIVER);
    clear();
  }
  void LCDDisplay::interfake(int p1, int p2, int p3) {(void)p1; (void)p2; lcdRows=p3; }   
  void LCDDisplay::clearNative() {LCDDriver.clear();}
  void LCDDisplay::setRowNative(byte row) { LCDDriver.setCursor(0, row); }
  void LCDDisplay::writeNative(char b){ LCDDriver.write(b); }    
  void LCDDisplay::displayNative() { LCDDriver.display(); }
