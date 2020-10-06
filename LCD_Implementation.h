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


// Part 1: creates a LCDDriver to provide a single hook to the LCD Library
#ifdef LCD_DRIVER
  
  // Refer https://github.com/duinoWitchery/hd44780 for details
  // All the IO drivers are included here, it makes no difference at all to gthe memory
  // as only the one you use will be included in the executable. 
  // This makes it possible to defgine the driver in use with a single word.
   
  #include <hd44780.h>
  #include <hd44780ioClass/hd44780_HC1627_I2C.h>           // control LCD with native i2c interface (Tsingtek Display HC1627)
  #include <hd44780ioClass/hd44780_I2Cexp.h>               // control LCD using i2c i/o exapander backpack (PCF8574 or MCP23008)
  #include <hd44780ioClass/hd44780_I2Clcd.h>               // control LCD with native i2c interface (PCF2116, PCF2119x, etc...)
  #include <hd44780ioClass/hd44780_NTCU165ECPB.h>          // control Noritake CU165ECBP-T2J LCD display over SPI
  #include <hd44780ioClass/hd44780_NTCU20025ECPB_pinIO.h>  // control Noritake CU20025ECPB using direct Arduino pin connections
  #include <hd44780ioClass/hd44780_NTCUUserial.h>          // control Noritake CU-U Series VFD display in serial mode
  #include <hd44780ioClass/hd44780_pinIO.h>                // control LCD using direct Arduino Pin connections
  LCD_DRIVER LCDDriver;
  void LCDDisplay::clear() {LCDDriver.clear();}
#endif   

#ifdef OLED_DRIVER 
  #include <Adafruit_SSD1306.h>
  Adafruit_SSD1306 LCDDriver(128, 32);
  void LCDDisplay::clear() {LCDDriver.clearDisplay();}
#endif 

#if defined(LCD_DRIVER) || defined(OLED_DRIVER)  
// Part 2: Implement the LCDDisplay shim class as a singleton.
// Notice that the LCDDisplay class declaration (LCDDisplay.h) is independent of the library
// but the implementation is compiled here with dependencies on LCDDriver which is 
// specific to the library in use.
// Thats the workaround to the drivers not all implementing a common interface. 
  
LCDDisplay * LCDDisplay::lcdDisplay=new LCDDisplay();
    
LCDDisplay::LCDDisplay() {}

void LCDDisplay::setRow(byte row) { 
    LCDDriver.setCursor(0, row);
}

size_t LCDDisplay::write(uint8_t b){ 
  return LCDDriver.write(b);    
}
void LCDDisplay::display() {
  LCDDriver.display();    
}  
#else 
// Build dummy shim to keep linker happy
LCDDisplay * LCDDisplay::lcdDisplay=0;
LCDDisplay::LCDDisplay() {}  
void LCDDisplay::setRow(byte row) { (void)row;} 
void LCDDisplay::clear() {}
size_t LCDDisplay::write(uint8_t b){ (void)b; return -1;} //  
void LCDDisplay::display(){} 
#endif 
