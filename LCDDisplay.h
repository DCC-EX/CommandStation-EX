#ifndef LCDDisplay_h
#define LCDDisplay_h
#include <Arduino.h>

// This class is created in LCDisplay_Implementation.h

class LCDDisplay : public Print {

  public:
    static const int MAX_LCD_ROWS=8;
    static const int MAX_LCD_COLS=16;
    static const long LCD_SCROLL_TIME=3000; // 3 seconds  
    
    static LCDDisplay* lcdDisplay; 
    LCDDisplay();
    void interfake(int p1, int p2, int p3);    

    // Internally handled functions
    static void loop();
    void loop2();
    void setRow(byte line); 
    void display();
    void clear();
    virtual size_t write(uint8_t b);
    using Print::write;
    
  private:
   // Relay functions to the live driver
    void clearNative();
    void displayNative();
    void setRowNative(byte line); 
    void writeNative(char * b);
   unsigned long lastScrollTime=0;
   int hotRow=0;
   int hotCol=0; 
   int topRow=0;
   int lcdRows;
   void renderRow(byte row);
   char rowBuffer[MAX_LCD_ROWS][MAX_LCD_COLS+1];     
};

#endif
