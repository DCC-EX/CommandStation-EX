/*
 *  © 2021, Chris Harlow, Neil McKechnie. All rights reserved.
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

LCDDisplay *LCDDisplay::loop2(bool force)
{
     unsigned long currentMillis = millis();

     if ((!force) && (currentMillis - lastScrollTime) < LCD_SCROLL_TIME)
          return NULL;

     do {
          if (bufferPointer == 0) { 
               // Find a line of data to write to the screen.
               if (!done) {
                    if (rowFirst < 0) rowFirst = rowNext;
                    // Skip blank rows
                    while (rowBuffer[rowNext][0] == 0) {
                         rowNext = (rowNext + 1) % MAX_LCD_ROWS;
                         if (rowNext == rowFirst) {
                              done = true;  // All lines done
                              break;
                         }
                    }
               }
               if (!done) {
                    // Non-blank line found, so copy it.
                    for (uint8_t i=0; i<sizeof(buffer); i++)
                         buffer[i] = rowBuffer[rowNext][i];
               } else
                    buffer[0] = '\0'; // Empty line
               setRowNative(slot);  // Set position for display
               charIndex = 0; 
               bufferPointer = &buffer[0];
          } else {
               // Write next character, or a space to erase current position.
               char ch = *bufferPointer;
               if (ch) {
                    writeNative(ch);
                    bufferPointer++;
               } else
                    writeNative(' ');

               if (++charIndex >= MAX_LCD_COLS) {
                    // Screen slot completed, move to next one
                    slot++;
                    bufferPointer = 0;
                    if (!done) {
                         rowNext = (rowNext + 1) % MAX_LCD_ROWS;
                         if (rowNext == rowFirst) done = true;
                    }
               }

               if (slot >= lcdRows) {
                    // Last slot finished, reset ready for next update.
                    slot = 0; 
                    done = false;
                    rowFirst = -1;
                    lastScrollTime = currentMillis;
                    return NULL;
               }
          }
     } while (force);

     return NULL;
}

  
   
