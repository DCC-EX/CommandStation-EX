
////////////////////////////////////////////////////////////////////////////////////
//  © 2020, Chris Harlow. All rights reserved.
//
// This implementation is designed to be #included ONLY ONCE in the .ino after config.h
//
// It will create a driver implemntation and a shim class implementation.
// This means that other classes can reference the shim without knowing
// which libraray is involved.
////////////////////////////////////////////////////////////////////////////////////

#include "config.h"
#include <Wire.h>
#include "LCDDisplay.h"
#define CONDITIONAL_LCD_START new LCDDisplay();    
LCDDisplay * LCDDisplay::lcdDisplay=0;

// Implement the LCDDisplay shim class as a singleton.
// Notice that the LCDDisplay class declaration (LCDDisplay.h) is independent of the library
// but the implementation is compiled here with dependencies on LCDDriver which is 
// specific to the library in use.
// Thats the workaround to the drivers not all implementing a common interface. 
 
#if defined(OLED_DRIVER) 
  #include "LCD_OLED.h"

#elif defined(LCD_DRIVER)  
  #include "LCD_LCD.h"      

#else 
  #include "LCD_NONE.h"
  #define CONDITIONAL_LCD_START // NO LCD CONFIGURED      
#endif
 

  
