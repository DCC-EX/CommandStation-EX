/*
 *  © 2021 Neil McKechnie
 *  © 2021 Chris Harlow
 *  All rights reserved.
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
#ifndef DisplayInterface_h
#define DisplayInterface_h

#include <Arduino.h>

// Definition of base class for displays.  The base class does nothing.
class DisplayInterface : public Print {
public:
  virtual DisplayInterface* loop2(bool force) { (void)force; return NULL; };
  virtual void setRow(byte line) { (void)line; };
  virtual void clear() { };
  virtual size_t write(uint8_t c) { (void)c; return 0; };
  // Additional functions to support multiple displays.
  // Display number zero is the default one and the original display
  // drivers overloaded the above calls only.  Newer display drivers
  // (e.g. HAL IO_OledDisplay) should override all functions.
  virtual void setRow(uint8_t displayNo, byte line) { 
    if (!displayNo) setRow(line);
  }
  virtual void clear(uint8_t displayNo) { 
    if (!displayNo) clear();
   }

  static DisplayInterface *lcdDisplay;
};

#endif
