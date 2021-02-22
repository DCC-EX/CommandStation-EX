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
#include <Wire.h>
#include "LCDDisplay.h"
  
LCDDisplay * LCDDisplay::lcdDisplay=0;

// Implement the LCDDisplay shim class as a singleton.
// Notice that the LCDDisplay class declaration (LCDDisplay.h) is independent of the library
// but the implementation is compiled here with dependencies on LCDDriver which is 
// specific to the library in use.
// Thats the workaround to the drivers not all implementing a common interface. 
 
#if defined(OLED_DRIVER) 
  #include "LCD_OLED.h"
  #define CONDITIONAL_LCD_START for (LCDDisplay * dummy=new LCDDisplay();dummy!=NULL; dummy=dummy->loop2(true)) 
  

#elif defined(LCD_DRIVER)  
  #include "LCD_LCD.h"      
  #define CONDITIONAL_LCD_START for (LCDDisplay * dummy=new LCDDisplay();dummy!=NULL; dummy=dummy->loop2(true))  

#else 
  #include "LCD_NONE.h"
  #define CONDITIONAL_LCD_START if (true) /* NO LCD CONFIG, but do the LCD macros to get DIAGS */      
#endif

#endif // LCD_Implementation_h
