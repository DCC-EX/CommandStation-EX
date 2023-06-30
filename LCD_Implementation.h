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
#if !defined(OLED_DRIVER) && !defined(LCD_DRIVER)
  #include "DisplayInterface.h"
  #define NO_LCD_USED
#else
  #include "LCDDisplay.h"
  #include "SSD1306Ascii.h"
  #include "LiquidCrystal_I2C.h"
#endif

// Implement the LCDDisplay shim class as a singleton.
// The DisplayInterface class implements a displayy handler with no code (null device);
// The LCDDisplay class sub-classes DisplayInterface to provide the common display code;
// Then LCDDisplay class is subclassed to the specific device type classes:
//    SSD1306AsciiWire for I2C OLED driver with SSD1306 or SH1106 controllers;
//    LiquidCrystal_I2C for I2C LCD driver for HD44780 with PCF8574 'backpack'.

#if defined(OLED_DRIVER)
  #define CONDITIONAL_LCD_START for (DisplayInterface * dummy=new SSD1306AsciiWire(OLED_DRIVER);dummy!=NULL; dummy=dummy->loop2(true))
  
#elif defined(LCD_DRIVER)
  #define CONDITIONAL_LCD_START for (DisplayInterface * dummy=new LiquidCrystal_I2C(LCD_DRIVER);dummy!=NULL; dummy=dummy->loop2(true))

#else
  // Create null display handler just in case someone calls lcdDisplay->something without checking if lcdDisplay is NULL!
  #define CONDITIONAL_LCD_START { new DisplayInterface(); }
#endif

#endif // LCD_Implementation_h
