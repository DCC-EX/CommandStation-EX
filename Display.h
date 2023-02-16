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
#ifndef Display_h
#define Display_h
#include <Arduino.h>
#include "defines.h"
#include "DisplayInterface.h"

// Allow maximum message length to be overridden from config.h
#if !defined(MAX_MSG_SIZE) 
// On a screen that's 128 pixels wide, character 22 overlaps end of screen
// However, by making the line longer than the screen, we don't have to 
// clear the screen, we just overwrite what was there.
#define MAX_MSG_SIZE 22  
#endif

// Set default scroll mode (overridable in config.h)
#if !defined(SCROLLMODE) 
#define SCROLLMODE 1
#endif

// This class is created in Display_Implementation.h

class Display : public DisplayInterface {
public:
  Display(DisplayDevice *deviceDriver);
  static const int MAX_CHARACTER_ROWS = 8;
  static const int MAX_CHARACTER_COLS = MAX_MSG_SIZE;
  static const long DISPLAY_SCROLL_TIME = 3000;  // 3 seconds

private:
  DisplayDevice *_deviceDriver;

  unsigned long lastScrollTime = 0;
  int8_t hotRow = 0;
  int8_t hotCol = 0;
  int8_t topRow = 0;
  int8_t slot = 0;
  int8_t rowFirst = -1;
  int8_t rowNext = 0;
  int8_t charIndex = 0;
  char buffer[MAX_CHARACTER_COLS + 1];
  char* bufferPointer = 0;
  bool done = false;
  uint16_t numCharacterRows;
  uint16_t numCharacterColumns = MAX_CHARACTER_COLS;

  char *rowBuffer[MAX_CHARACTER_ROWS];

public:
  void begin() override;  
  void _clear() override;
  void _setRow(uint8_t line) override;
  size_t _write(uint8_t b) override;
  void _refresh() override;
  void _displayLoop() override;
  Display *loop2(bool force);
  void moveToNextRow();
  void skipBlankRows();

};

#endif
