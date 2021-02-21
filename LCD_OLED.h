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

// OLED Implementation of LCDDisplay class 
// Note: this file is optionally included by LCD_Implementation.h
// It is NOT a .cpp file to prevent it being compiled and demanding libraries even when not needed.
  
#include "Wire.h"
#include "SSD1306Ascii.h"
SSD1306AsciiWire LCDDriver;
  
// DEVICE SPECIFIC LCDDisplay Implementation for OLED

  LCDDisplay::LCDDisplay() {
    // Scan for device on 0x3c and 0x3d.
    Wire.begin();
    for (byte address=0x3c; address<=0x3d; address++) {
      Wire.beginTransmission(address);
      byte error = Wire.endTransmission(true);
      if (!error) {
        // Device found
        DIAG(F("\nOLED display found at 0x%x"), address); 
        interfake(OLED_DRIVER,0);
        const DevType *devType;
        if (lcdCols == 132)
          devType = &SH1106_128x64; // Actually 132x64 but treated as 128x64
        else if (lcdCols == 128 && lcdRows == 4) 
          devType = &Adafruit128x32;
        else 
          devType = &Adafruit128x64;
        LCDDriver.begin(devType, address);
        lcdDisplay=this;
        LCDDriver.setFont(System5x7);  // Normal 1:1 pixel scale, 8 bits high
        clear();
        return;  
      }
    }
    DIAG(F("\nOLED display not found\n"));
  }

  void LCDDisplay::interfake(int p1, int p2, int p3) {lcdCols=p1; lcdRows=p2/8; (void)p3;}   

  void LCDDisplay::clearNative() {LCDDriver.clear();}

  void LCDDisplay::setRowNative(byte row) {
    // Positions text write to start of row 1..n and clears previous text 
    int y=row;
    LCDDriver.setCursor(0, y); 
  }
  
  void LCDDisplay::writeNative(char b) {
    LCDDriver.write(b);
  }
  
  void LCDDisplay::displayNative() {  }  
  
