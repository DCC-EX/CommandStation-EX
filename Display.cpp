/*
 *  © 2021, Chris Harlow, Neil McKechnie. All rights reserved.
 *  © 2023, Harald Barth.
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
 *     only noMoreRowsToDisplay during start-up.
 *  The scroll mode is selected by defining SCROLLMODE as 0, 1 or 2
 *  in the config.h.
 *  #define SCROLLMODE 0 is scroll continuous (fill screen if poss),
 *  #define SCROLLMODE 1 is by page (alternate between pages),
 *  #define SCROLLMODE 2 is by row (move up 1 row at a time).

 */

#include "Display.h"

// Constructor - allocates device driver.
Display::Display(DisplayDevice *deviceDriver) {
  _deviceDriver = deviceDriver;
  // Get device dimensions in characters (e.g. 16x2).
  numCharacterColumns = _deviceDriver->getNumCols();
  numCharacterRows = _deviceDriver->getNumRows();
  for (uint8_t row = 0; row < MAX_CHARACTER_ROWS; row++) 
    rowBuffer[row][0] = '\0';
  topRow = ROW_INITIAL;  // loop2 will fill from row 0
  
  addDisplay(0);  // Add this display as display number 0
};

void Display::begin() {
  _deviceDriver->begin();
  _deviceDriver->clearNative();
}

void Display::_clear() {
  _deviceDriver->clearNative();
  for (uint8_t row = 0; row < MAX_CHARACTER_ROWS; row++) 
    rowBuffer[row][0] = '\0';
  topRow = ROW_INITIAL;  // loop2 will fill from row 0
}

void Display::_setRow(uint8_t line) {
  hotRow = line;
  hotCol = 0;
  rowBuffer[hotRow][0] = 0;  // Clear existing text
}

size_t Display::_write(uint8_t b) {
  if (hotRow >= MAX_CHARACTER_ROWS || hotCol >= MAX_CHARACTER_COLS) return -1;
  rowBuffer[hotRow][hotCol] = b;
  hotCol++;
  rowBuffer[hotRow][hotCol] = 0;
  return 1;
}

// Refresh screen completely (will block until complete). Used
// during start-up.
void Display::_refresh() {
  loop2(true);
}

// On normal loop entries, loop will only make one output request on each
// entry, to avoid blocking while waiting for the I2C.
void Display::_displayLoop() {
  // If output device is busy, don't do anything on this loop
  // This avoids blocking while waiting for the device to complete.
  if (!_deviceDriver->isBusy()) loop2(false);
}

Display *Display::loop2(bool force) {
  unsigned long currentMillis = millis();

  if (!force) {
    // See if we're in the time between updates
    if ((currentMillis - lastScrollTime) < DISPLAY_SCROLL_TIME)
      return NULL;
  } else {
    // force full screen update from the beginning.
    rowFirst = ROW_INITIAL;
    rowNext = ROW_INITIAL;
    bufferPointer = 0;
    noMoreRowsToDisplay = false;
    slot = 0;
  }

  do {
    if (bufferPointer == 0) {
      // Find a line of data to write to the screen.
      if (rowFirst == ROW_INITIAL) rowFirst = rowNext;
      if (findNextNonBlankRow()) {
        // Non-blank line found, so copy it (including terminator)
        for (uint8_t i = 0; i <= MAX_CHARACTER_COLS; i++)
          buffer[i] = rowBuffer[rowNext][i];
      } else {
        // No non-blank lines left, so draw a blank line
        buffer[0] = 0;
      }
#if SCROLLMODE==2
      if (buffer[0] == 0 && needScroll){     // surpresses empty line
#else
      if (false){
#endif
	charIndex = MAX_CHARACTER_COLS;
	slot--;
      } else {
	_deviceDriver->setRowNative(slot);  // Set position for display
	charIndex = 0;
	bufferPointer = &buffer[0];
      }
      rowNext++;
    } else {
      // Write next character, or a space to erase current position.
      char ch = *bufferPointer;
      if (ch) {
	_deviceDriver->writeNative(ch);
        bufferPointer++;
      } else {
        _deviceDriver->writeNative(' ');
      }
    }

    if (++charIndex >= MAX_CHARACTER_COLS) {
      // Screen slot completed, move to next slot on screen
      bufferPointer = 0;
      slot++;
      if (slot >= numCharacterRows) {
	// Last slot on screen written, reset ready for next screen update.
#if SCROLLMODE==2 || SCROLLMODE==1
	if (!noMoreRowsToDisplay) {
	  needScroll = true;
	}
	if (needScroll) {
#if SCROLLMODE==2
	  // SCROLLMODE 2 rotates through rowFirst and we
	  // (ab)use findNextBlankRow() to figure out
	  // next valid row which can be start row.
	  rowNext = rowFirst + 1;
	  noMoreRowsToDisplay = false;
	  findNextNonBlankRow();
	  if (rowNext == ROW_INITIAL)
	    rowNext = 0;
	  rowFirst = ROW_INITIAL;
#else
	  // SCROLLMODE 1 just alternates when the
	  // flag indicates that we have come to the end
	  if (noMoreRowsToDisplay)
	    rowNext = 0;
#endif
	} else {
	  // SCROLLMODE 1 or 2 but not scroll active
	  rowNext = 0;
	}
#else
	// this is for SCROLLMODE 0 but what should it do?
	rowNext = 0;
#endif
	rowFirst = ROW_INITIAL;

	noMoreRowsToDisplay = false;
	slot = 0;
	lastScrollTime = currentMillis;
	return NULL;
      }
#if SCROLLMODE==2
      if (needScroll)
	noMoreRowsToDisplay = false;
#endif
    }
  } while (force);

  return NULL;
}

bool Display::findNextNonBlankRow() {
  while (!noMoreRowsToDisplay) {
    if (rowNext == ROW_INITIAL)
      rowNext = 0;
    if (rowNext >= MAX_CHARACTER_ROWS) {
      // Finished if we've looped back to start
      rowNext = ROW_INITIAL;
      noMoreRowsToDisplay = true;
      return false;
    }
    if (rowBuffer[rowNext][0] != 0) {
      //rowBuffer[rowNext][0] = '0' + rowNext; // usefull for debug
      //rowBuffer[rowNext][1] = '0' + rowFirst; // usefull for debug
      // Found non-blank row
      return true;
    }
    rowNext = rowNext + 1;
  }
  return false;
}
