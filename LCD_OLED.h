// OLED Implementation of LCDDisplay class 
// Note: this file is optionally included by LCD_Implenentation.h
// It is NOT a .cpp file to prevent it being compiled and demanding libraraies even when not needed.
  
#include <Adafruit_SSD1306.h>
Adafruit_SSD1306 LCDDriver(OLED_DRIVER);
  
// DEVICE SPECIFIC LCDDisplay Implementation for OLED

LCDDisplay::LCDDisplay() {
    if(LCDDriver.begin(SSD1306_SWITCHCAPVCC, 0x3C) || LCDDriver.begin(SSD1306_SWITCHCAPVCC, 0x3D)) {
      DIAG(F("\nOLED display found")); 
      delay(2000); // painful Adafruit splash pants! 
      lcdDisplay=this;
      LCDDriver.clearDisplay();
      LCDDriver.setTextSize(1);             // Normal 1:1 pixel scale
      LCDDriver.setTextColor(SSD1306_WHITE);            // Draw white text
      return;  
      }
    DIAG(F("\nOLED display not found\n"));
    }
    
  void LCDDisplay::clear() {LCDDriver.clearDisplay();}

  void LCDDisplay::setRow(byte row) {
    // Positions text write to start of row 1..n and clears previous text 
    int y=8*(row-1);
    LCDDriver.fillRect(0, y, LCDDriver.width(), 8, SSD1306_BLACK);
    LCDDriver.setCursor(0, y); 
    }
  
  size_t LCDDisplay::write(uint8_t b){ return LCDDriver.write(b); }    
  
  void LCDDisplay::display() { LCDDriver.display(); }  
  
