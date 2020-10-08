  #include <LiquidCrystal_I2C.h> 
  LiquidCrystal_I2C LCDDriver(0x3F,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display
  // DEVICE SPECIFIC LCDDisplay Implementation for LCD_DRIVER
  LCDDisplay::LCDDisplay() { 
    lcdDisplay=this;
    LCDDriver.init();
    LCDDriver.backlight();
    LCDDriver.clear();
    }
  void LCDDisplay::clear() {LCDDriver.clear();}
  void LCDDisplay::setRow(byte row) { 
    LCDDriver.setCursor(0, row-1);
    LCDDriver.print(F("                    "));
    LCDDriver.setCursor(0, row-1);
    }
  size_t LCDDisplay::write(uint8_t b){ return LCDDriver.write(b); }    
  void LCDDisplay::display() { LCDDriver.display(); }
