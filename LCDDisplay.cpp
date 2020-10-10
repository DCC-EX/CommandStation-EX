
// CAUTION: the device dependent parts of this class are created in the .ini using LCD_Implementation.h
#include "LCDDisplay.h"

 void LCDDisplay::clear() {
      clearNative();
      for (byte row=0;row<MAX_LCD_ROWS; row++)  rowBuffer[row][0]='\0';
 }

 void LCDDisplay::setRow(byte line) {
      hotRow=line;
      hotCol=0;
      if (hotRow<MAX_LCD_ROWS) memset(rowBuffer[hotRow],0,MAX_LCD_COLS+1);       
 }

size_t LCDDisplay::write(uint8_t b) {
     if (hotRow>=MAX_LCD_ROWS || hotCol>=MAX_LCD_COLS) return -1;
     rowBuffer[hotRow][hotCol]=b;
     hotCol++;
     return 1;
 }

 void LCDDisplay::display() {
     if (hotRow>=topRow && hotRow<topRow+lcdRows) {
        renderRow(hotRow);
        displayNative(); 
     }
 }
 
 void LCDDisplay::renderRow(byte row) {
      setRowNative(row-topRow);
      writeNative(rowBuffer[row]);
 }
 
 void LCDDisplay::loop() {
    if (!lcdDisplay) return;
    lcdDisplay->loop2();
 }
 
 void LCDDisplay::loop2() { 
    if ((millis() - lastScrollTime)< LCD_SCROLL_TIME) return;
    lastScrollTime=millis();
    clearNative();
    topRow+=lcdRows;
    if (topRow>MAX_LCD_ROWS-lcdRows) topRow=0;
    for (int r=0; r<lcdRows; r++) {
       renderRow(topRow+r); 
    } 
    displayNative();   
 }
 
  
   
