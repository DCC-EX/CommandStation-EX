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

// CAUTION: the device dependent parts of this class are created in the .ini using LCD_Implementation.h
#include "LCDDisplay.h"

 void LCDDisplay::clear() {
      clearNative();
      for (byte row=0;row<MAX_LCD_ROWS; row++)  rowBuffer[row][0]='\0';
      topRow=-1; // loop2 will fill from row 0
 }

 void LCDDisplay::setRow(byte line) {
      hotRow=line;
      hotCol=0;
 }

size_t LCDDisplay::write(uint8_t b) {
     if (hotRow>=MAX_LCD_ROWS || hotCol>=MAX_LCD_COLS) return -1;
     rowBuffer[hotRow][hotCol]=b;
     hotCol++;
     rowBuffer[hotRow][hotCol]=0;
     return 1;
 }
 
 void LCDDisplay::loop() {
    if (!lcdDisplay) return;
    lcdDisplay->loop2(false);
 }
 
 LCDDisplay*  LCDDisplay::loop2(bool force) { 
    if ((!force) && (millis() - lastScrollTime)< LCD_SCROLL_TIME) return NULL;
    lastScrollTime=millis();
    clearNative();
    int rowFirst=nextFilledRow();
    if (rowFirst<0)return NULL; // No filled rows
    setRowNative(0);
    writeNative(rowBuffer[rowFirst]);
    for (int slot=1;slot<lcdRows;slot++) {
      int rowNext=nextFilledRow();
      if (rowNext==rowFirst){
         // we have wrapped around and not filled the screen 
         topRow=-1; // start again at first row next time.  
         break; 
      }
      setRowNative(slot);
      writeNative(rowBuffer[rowNext]);
    } 
    displayNative();   
    return NULL; 
 }

 int LCDDisplay::nextFilledRow() {
      for (int rx=1;rx<=MAX_LCD_ROWS;rx++) {
          topRow++;
          topRow %= MAX_LCD_ROWS;
          if (rowBuffer[topRow][0]) return topRow; 
          }
      return -1; // No slots filled
 }

  
   
