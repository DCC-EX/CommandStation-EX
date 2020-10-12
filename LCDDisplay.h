/*
 *  © 2020, Chris Harlow. All rights reserved.
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
    LCDDisplay*  loop2(bool force);
    void setRow(byte line); 
    void clear();
   
    virtual size_t write(uint8_t b);
    using Print::write;
    
  private:
    int nextFilledRow();
  
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
