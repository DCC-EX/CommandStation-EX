
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

// Implement the LCDDisplay shim class as a singleton.
// Notice that the LCDDisplay class declaration (LCDDisplay.h) is independent of the library
// but the implementation is compiled here with dependencies on LCDDriver which is 
// specific to the library in use.
// Thats the workaround to the drivers not all implementing a common interface. 

#if  defined(LCD_DRIVER_ADVANCED)
  
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
  LCDDriver LCD_DRIVER_ADVANCED;
  
  // DEVICE SPECIFIC LCDDisplay Implementation for LCD_DRIVER_ADVANCED
  LCDDisplay * LCDDisplay::lcdDisplay=0;
  LCDDisplay::LCDDisplay() { lcdDisplay=this;  }
  void LCDDisplay::clear() {LCDDriver.clear();}
  void LCDDisplay::setRow(byte row) { LCDDriver.setCursor(0, row); }
  size_t LCDDisplay::write(uint8_t b){ return LCDDriver.write(b); }    
  void LCDDisplay::display() { LCDDriver.display(); }
  #define CONDITIONAL_LCD_START new LCDDisplay();    

#elif defined(OLED_DRIVER) 
  #include <Adafruit_SSD1306.h>
  Adafruit_SSD1306 LCDDriver(128, 32);
  
  // DEVICE SPECIFIC LCDDisplay Implementation for OLED
    LCDDisplay * LCDDisplay::lcdDisplay=0;
    LCDDisplay::LCDDisplay() {
    if(!LCDDriver.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
      // OLED not found/working
      DIAG(F("\nOLED display not found\n"));
      return;  
      }
     DIAG(F("\nOLED display found")); 
     delay(2000); // painful Adafruit splash pants! 
    lcdDisplay=this;
    LCDDriver.clearDisplay();
    LCDDriver.setTextSize(1);             // Normal 1:1 pixel scale
    LCDDriver.setTextColor(SSD1306_WHITE);            // Draw white text
    }
  void LCDDisplay::clear() {LCDDriver.clearDisplay();}
  void LCDDisplay::setRow(byte row) { 
    int y=8*(row-1);
    LCDDriver.fillRect(0, y, LCDDriver.width(), 8, SSD1306_BLACK);
    LCDDriver.setCursor(0, y); 
    }
  size_t LCDDisplay::write(uint8_t b){ return LCDDriver.write(b); }    
  void LCDDisplay::display() { LCDDriver.display(); }  
  #define CONDITIONAL_LCD_START new LCDDisplay();    

#elif defined(LCD_DRIVER)  
  #include <LiquidCrystal_I2C.h>  
  LiquidCrystal_I2C LCDDriver(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display
  // DEVICE SPECIFIC LCDDisplay Implementation for LCD_DRIVER
  LCDDisplay * LCDDisplay::lcdDisplay=0;
  void LCDDisplay::LCDDisplay() { lcdDisplay=this; }
  void LCDDisplay::clear() {LCDDriver.clear();}
  void LCDDisplay::setRow(byte row) { LCDDriver.setCursor(0, row); }
  size_t LCDDisplay::write(uint8_t b){ return LCDDriver.write(b); }    
  void LCDDisplay::display() { LCDDriver.display(); }
  #define CONDITIONAL_LCD_START new LCDDisplay();    
      
#else 
  // Build dummy shim to keep linker happy
  LCDDisplay * LCDDisplay::lcdDisplay=0;
  LCDDisplay::LCDDisplay() {}  
  void LCDDisplay::setRow(byte row) { (void)row;} 
  void LCDDisplay::clear() {}
  size_t LCDDisplay::write(uint8_t b){ (void)b; return -1;} //  
  void LCDDisplay::display(){}
  #define CONDITIONAL_LCD_START // NO LCD CONFIGURED     
 
#endif 
