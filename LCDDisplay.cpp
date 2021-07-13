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

// CAUTION: the device dependent parts of this class are created in the .ini
// using LCD_Implementation.h

/* The strategy for drawing the screen is as follows.
 *  1) There are up to eight rows of text to be displayed.
 *  2) Blank rows of text are ignored.
 *  3) If there are more non-blank rows than screen lines,
 *     then all of the rows are displayed, with the rest of the
 *     screen being blank.
 *  4) If there are fewer non-blank rows than screen lines,
 *     then a scrolling strategy is adopted so that, on each screen
 *     refresh, a different subset of the rows is presented.
 *  5) On each entry into loop2(), a single operation is sent to the 
 *     screen; this may be a position command or a character for
 *     display.  This spreads the onerous work of updating the screen
 *     and ensures that other loop() functions in the application are
 *     not held up significantly.  The exception to this is when 
 *     the loop2() function is called with force=true, where 
 *     a screen update is executed to completion.  This is normally
 *     only done during start-up.
 *  The scroll mode is selected by defining SCROLLMODE as 0, 1 or 2
 *  in the config.h.
 *  #define SCROLLMODE 0 is scroll continuous (fill screen if poss),
 *  #define SCROLLMODE 1 is by page (alternate between pages),
 *  #define SCROLLMODE 2 is by row (move up 1 row at a time).

 */

#include "LCDDisplay.h"

void LCDDisplay::clear() {
  clearNative();
  for (byte row = 0; row < MAX_LCD_ROWS; row++) rowBuffer[row][0] = '\0';
  topRow = -1;  // loop2 will fill from row 0
}

void LCDDisplay::setRow(byte line) {
  hotRow = line;
  hotCol = 0;
}

size_t LCDDisplay::write(uint8_t b) {
  if (hotRow >= MAX_LCD_ROWS || hotCol >= MAX_LCD_COLS) return -1;
  rowBuffer[hotRow][hotCol] = b;
  hotCol++;
  rowBuffer[hotRow][hotCol] = 0;
  return 1;
}

void LCDDisplay::loop() {
  if (!lcdDisplay) return;
  lcdDisplay->loop2(false);
}

LCDDisplay *LCDDisplay::loop2(bool force) {
  if (!lcdDisplay) return NULL;

  unsigned long currentMillis = millis();

  if (!force) {
    // See if we're in the time between updates
    if ((currentMillis - lastScrollTime) < LCD_SCROLL_TIME)
      return NULL;
  } else {
    // force full screen update from the beginning.
    rowFirst = -1;
    rowNext = 0;
    bufferPointer = 0;
    done = false;
    slot = 0;
  }

  do {
    if (bufferPointer == 0) {
      // Find a line of data to write to the screen.
      if (rowFirst < 0) rowFirst = rowNext;
      skipBlankRows();
      if (!done) {
        // Non-blank line found, so copy it.
        for (uint8_t i = 0; i < sizeof(buffer); i++)
          buffer[i] = rowBuffer[rowNext][i];
      } else
        buffer[0] = '\0';  // Empty line
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
        // Screen slot completed, move to next slot on screen
        slot++;
        bufferPointer = 0;
        if (!done) {
          moveToNextRow();
          skipBlankRows();
        }
      }

      if (slot >= lcdRows) {
        // Last slot finished, reset ready for next screen update.
#if SCROLLMODE==2
        if (!done) {
          // On next refresh, restart one row on from previous start.
          rowNext = rowFirst;
          moveToNextRow();
          skipBlankRows();
        }
#endif
        done = false;
        slot = 0;
        rowFirst = -1;
        lastScrollTime = currentMillis;
        return NULL;
      }
    }
  } while (force);

  return NULL;
}

void LCDDisplay::moveToNextRow() {
  rowNext = (rowNext + 1) % MAX_LCD_ROWS;
#if SCROLLMODE == 1
  // Finished if we've looped back to row 0
  if (rowNext == 0) done = true;
#else
  // Finished if we're back to the first one shown
  if (rowNext == rowFirst) done = true;
#endif
}

void LCDDisplay::skipBlankRows() {
  while (!done && rowBuffer[rowNext][0] == 0)
    moveToNextRow();
}
