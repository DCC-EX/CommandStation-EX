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
#ifndef LCDDisplay_h
#define LCDDisplay_h
#include <Arduino.h>
#include "DisplayInterface.h"

#if __has_include ( "config.h")
  #include "config.h"
#endif

// Allow maximum message length to be overridden from config.h
#if !defined(MAX_MSG_SIZE) 
#define MAX_MSG_SIZE 16
#endif

// Set default scroll mode (overridable in config.h)
#if !defined(SCROLLMODE) 
#define SCROLLMODE 1
#endif

// This class is created in LCDisplay_Implementation.h

class LCDDisplay : public DisplayInterface {
 public:
  static const int MAX_LCD_ROWS = 8;
  static const int MAX_LCD_COLS = MAX_MSG_SIZE;
  static const long LCD_SCROLL_TIME = 3000;  // 3 seconds

  // Internally handled functions
  static void loop();
  LCDDisplay* loop2(bool force);
  void setRow(byte line);
  void clear();

  size_t write(uint8_t b);

protected:
  uint8_t lcdRows;
  uint8_t lcdCols;

 private:
  void moveToNextRow();
  void skipBlankRows();

  // Relay functions to the live driver in the subclass
  virtual void clearNative() = 0;
  virtual void setRowNative(byte line) = 0;
  virtual size_t writeNative(uint8_t b) = 0;

  unsigned long lastScrollTime = 0;
  int8_t hotRow = 0;
  int8_t hotCol = 0;
  int8_t topRow = 0;
  int8_t slot = 0;
  int8_t rowFirst = -1;
  int8_t rowNext = 0;
  int8_t charIndex = 0;
  char buffer[MAX_LCD_COLS + 1];
  char* bufferPointer = 0;
  bool done = false;

  char rowBuffer[MAX_LCD_ROWS][MAX_LCD_COLS + 1];
};

#endif
