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

////////////////////////////////////////////////////////////////////////////////////
// This implementation is designed to be #included ONLY ONCE in the .ino 
//
// It will create a driver implemntation and a shim class implementation.
// This means that other classes can reference the shim without knowing
// which library is involved.
////////////////////////////////////////////////////////////////////////////////////

#ifndef LCD_Implementation_h
#define LCD_Implementation_h
#include "DisplayInterface.h"
#include "SSD1306Ascii.h"
#include "LiquidCrystal_I2C.h"
  

// Implement the Display shim class as a singleton.
// The DisplayInterface class implements a display handler with no code (null device);
// The Display class sub-classes DisplayInterface to provide the common display code;
// Then Display class talks to the specific device type classes:
//    SSD1306AsciiWire for I2C OLED driver with SSD1306 or SH1106 controllers;
//    LiquidCrystal_I2C for I2C LCD driver for HD44780 with PCF8574 'backpack'.

#if defined(OLED_DRIVER)
  #define DISPLAY_START(xxx) { \
    DisplayInterface *t = new Display(new SSD1306AsciiWire(OLED_DRIVER)); \
    t->begin(); \
    xxx; \
    t->refresh(); \
  } 
  
#elif defined(LCD_DRIVER)
  #define DISPLAY_START(xxx) { \
    DisplayInterface *t = new Display(new LiquidCrystal_I2C(LCD_DRIVER)); \
    t->begin(); \
    xxx;  \
    t->refresh();}
#else
  #define DISPLAY_START(xxx) { \
  xxx; \
  }

#endif
#endif // LCD_Implementation_h
