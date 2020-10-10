  #include <LiquidCrystal_I2C.h> 
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
  void LCDDisplay::setRowNative(byte row) { 
    LCDDriver.setCursor(0, row);
    LCDDriver.print(F("                "));
    LCDDriver.setCursor(0, row);
    }
  void LCDDisplay::writeNative(char * b){ LCDDriver.print(b); }    
  void LCDDisplay::displayNative() { LCDDriver.display(); }
