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
protected:
  static DisplayInterface *_displayHandler;
  static uint8_t _selectedDisplayNo;  // Nothing selected.
  DisplayInterface *_nextHandler = NULL;
  uint8_t _displayNo = 0;

public:
  // Add display object to list of displays
  void addDisplay(uint8_t displayNo) {
    _nextHandler = _displayHandler;
    _displayHandler = this;
    _displayNo = displayNo;
  }
  static DisplayInterface *getDisplayHandler() {
    return _displayHandler;
  }
  uint8_t getDisplayNo() {
    return _displayNo;
  }

  // The next functions are to provide compatibility with calls to the LCD function
  // which does not specify a display number.  These always apply to display '0'.
  static void refresh() { refresh(0); };
  static void setRow(uint8_t line) { setRow(0, line); };
  static void clear() { clear(0); };

  // Additional functions to support multiple displays.  These perform a
  // multicast to all displays that match the selected displayNo.
  // Display number zero is the default one.
  static void setRow(uint8_t displayNo, uint8_t line) { 
    _selectedDisplayNo = displayNo;
    for (DisplayInterface *p = _displayHandler; p!=0; p=p->_nextHandler) { 
      if (displayNo == p->_displayNo) p->_setRow(line);
    }
  }
  size_t write (uint8_t c) override {
    for (DisplayInterface *p = _displayHandler; p!=0; p=p->_nextHandler) 
      if (_selectedDisplayNo == p->_displayNo) p->_write(c);
    return _displayHandler ? 1 : 0;
  }
  static void clear(uint8_t displayNo) { 
    for (DisplayInterface *p = _displayHandler; p!=0; p=p->_nextHandler) 
      if (displayNo == p->_displayNo) p->_clear();
  }
  static void refresh(uint8_t displayNo) {
    for (DisplayInterface *p = _displayHandler; p!=0; p=p->_nextHandler)
      if (displayNo == p->_displayNo) p->_refresh();
  }
  static void loop() {
    for (DisplayInterface *p = _displayHandler; p!=0; p=p->_nextHandler) 
      p->_displayLoop();
  };
  // The following are overridden within the specific device class
  virtual void begin() {};
  virtual size_t _write(uint8_t c) { (void)c; return 0; };
  virtual void _setRow(uint8_t line) { (void)line; }
  virtual void _clear() {}
  virtual void _refresh() {}
  virtual void _displayLoop() {}
};

class DisplayDevice {
public:
  virtual bool begin() { return true; }
  virtual void clearNative() = 0;
  virtual void setRowNative(uint8_t line) = 0;
  virtual size_t writeNative(uint8_t c) = 0;
  virtual bool isBusy() = 0;
  virtual uint16_t getNumRows() = 0;
  virtual uint16_t getNumCols() = 0;
};
#endif
