 
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
  LCDDisplay::LCDDisplay() { lcdDisplay=this;  }
  void LCDDisplay::clear() {LCDDriver.clear();}
  void LCDDisplay::setRow(byte row) { LCDDriver.setCursor(0, row); }
  size_t LCDDisplay::write(uint8_t b){ return LCDDriver.write(b); }    
  void LCDDisplay::display() { LCDDriver.display(); }
 
