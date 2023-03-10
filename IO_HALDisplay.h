/*
 *  © 2023, Neil McKechnie. All rights reserved.
 *  
 *  This file is part of DCC++EX API
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

/* 
 * This driver provides a more immediate interface into the OLED display
 * than the one installed through the config.h file.  When an LCD(...) call
 * is made, the text is output immediately to the specified display line,
 * without waiting for the next 2.5 second refresh.  However, if the line 
 * specified is off the screen then the text in the bottom line will be 
 * overwritten.  There is however a special case that if line 255 is specified, 
 * the existing text will scroll up and the new line added to the bottom
 * line of the screen.
 * 
 * To install, use the following command in myHal.cpp:
 *
 *    HALDisplay<OLED>::create(address, width, height);
 * 
 * where address is the I2C address of the OLED display (0x3c or 0x3d),
 * width is the width in pixels, and height is the height in pixels.
 * 
 * Valid width and height are 128x32 (SSD1306 controller), 
 * 128x64 (SSD1306) and 132x64 (SH1106).  The driver uses
 * a 5x7 character set in a 6x8 pixel cell.
 * 
 * OR
 * 
 *    HALDisplay<LiquidCrystal>::create(address, width, height);
 * 
 * where address is the I2C address of the LCD display (0x27 typically),
 * width is the width in characters (16 or 20 typically),
 * and height is the height in characters (2 or 4 typically).
 */


#ifndef IO_HALDisplay_H
#define IO_HALDisplay_H

#include "IODevice.h"
#include "DisplayInterface.h"
#include "SSD1306Ascii.h"
#include "LiquidCrystal_I2C.h"
#include "version.h"

typedef SSD1306AsciiWire OLED;
typedef LiquidCrystal_I2C LiquidCrystal; 

template <class T> 
class HALDisplay : public IODevice, public DisplayInterface {
private:
  // Here we define the device-specific variables.  
  uint8_t _height; // in pixels
  uint8_t _width;  // in pixels
  T *_displayDriver;
  uint8_t _rowNo = 0;   // Row number being written by caller
  uint8_t _colNo = 0;  // Position in line being written by caller
  uint8_t _numRows;
  uint8_t _numCols;
  char *_buffer = NULL;
  uint8_t *_rowGeneration = NULL;
  uint8_t *_lastRowGeneration = NULL;
  uint8_t _rowNoToScreen = 0; 
  uint8_t _charPosToScreen = 0;
  bool _startAgain = false;
  DisplayInterface *_nextDisplay = NULL;

public:
  //  Static function to handle "HALDisplay::create(...)" calls.
  static void create(I2CAddress i2cAddress, int width, int height) {
    if (checkNoOverlap(0, 0, i2cAddress)) new HALDisplay(0, i2cAddress, width, height);
  } 
  static void create(uint8_t displayNo, I2CAddress i2cAddress, int width, int height) {
    if (checkNoOverlap(0, 0, i2cAddress)) new HALDisplay(displayNo, i2cAddress, width, height);
  } 

protected:
  // Constructor
  HALDisplay(uint8_t displayNo, I2CAddress i2cAddress, int width, int height) {
    _displayDriver = new T(i2cAddress, width, height);
    if (!_displayDriver) return;  // Check for memory allocation failure
    _I2CAddress = i2cAddress;
    _width = width;
    _height = height;
    _numCols = _displayDriver->getNumCols();
    _numRows = _displayDriver->getNumRows();

    _charPosToScreen = _numCols;

    // Allocate arrays
    _buffer = (char *)calloc(_numRows*_numCols, sizeof(char));
    if (!_buffer) return;  // Check for memory allocation failure
    _rowGeneration = (uint8_t *)calloc(_numRows, sizeof(uint8_t));
    if (!_rowGeneration) return;  // Check for memory allocation failure
    _lastRowGeneration = (uint8_t *)calloc(_numRows, sizeof(uint8_t));
    if (!_lastRowGeneration) return;  // Check for memory allocation failure

    // Fill buffer with spaces
    memset(_buffer, ' ', _numCols*_numRows);

    _displayDriver->clearNative();

    // Add device to list of HAL devices (not necessary but allows
    // status to be displayed using <D HAL SHOW> and device to be
    // reinitialised using <D HAL RESET>).
    IODevice::addDevice(this);

    // Also add this display to list of display handlers
    DisplayInterface::addDisplay(displayNo);

    // Is this the system display (0)?
    if (displayNo == 0) {
      // Set first two lines on screen
      this->setRow(displayNo, 0);
      print(F("DCC-EX v"));
      print(F(VERSION));
      setRow(displayNo, 1);
      print(F("Lic GPLv3"));
    }
  }
  
  
  void screenUpdate() {
    // Loop through the buffer and if a row has changed
    // (rowGeneration[row] is changed) then start writing the
    // characters from the buffer, one character per entry, 
    // to the screen until that row has been refreshed.

    // First check if the OLED driver is still busy from a previous 
    // call.  If so, don't do anything until the next entry.
    if (!_displayDriver->isBusy()) {
      // Check if we've just done the end of a row
      if (_charPosToScreen >= _numCols) {
        // Move to next line
        if (++_rowNoToScreen >= _numRows || _startAgain) {
          _rowNoToScreen = 0; // Wrap to first row
          _startAgain = false;
        }

        if (_rowGeneration[_rowNoToScreen] != _lastRowGeneration[_rowNoToScreen]) {
          // Row content has changed, so start outputting it
          _lastRowGeneration[_rowNoToScreen] = _rowGeneration[_rowNoToScreen];
          _displayDriver->setRowNative(_rowNoToScreen);
          _charPosToScreen = 0;  // Prepare to output first character on next entry
        } else {
          // Row not changed, don't bother writing it.
        }
      } else {
        // output character at current position
        _displayDriver->writeNative(_buffer[_rowNoToScreen*_numCols+_charPosToScreen++]);
      }  
    }
    return;
  }

  /////////////////////////////////////////////////
  // IODevice Class Member Overrides
  /////////////////////////////////////////////////

  // Device-specific initialisation
  void _begin() override {
    // Initialise device
    if (_displayDriver->begin()) {

      _display();

      // Force all rows to be redrawn
      for (uint8_t row=0; row<_numRows; row++)
        _rowGeneration[row]++;
      
      // Start with top line (looks better).  
      // The numbers will wrap round on the first loop2 entry.
      _rowNoToScreen = _numRows;
      _charPosToScreen = _numCols;
    }
  }

  void _loop(unsigned long) override {
    screenUpdate();
  }
  
  // Display information about the device.
  void _display() {
    DIAG(F("HALDisplay %d configured on addr %s"), _displayNo, _I2CAddress.toString());
  }
  
  /////////////////////////////////////////////////
  // DisplayInterface functions
  // 
  /////////////////////////////////////////////////
  
public:
  void _displayLoop() override {
    screenUpdate();
  }

  // Position on nominated line number (0 to number of lines -1)
  // Clear the line in the buffer ready for updating
  // The displayNo referenced here is remembered and any following
  // calls to write() will be directed to that display.
  void _setRow(byte line) override {
    if (line == 255) {
      // LCD(255,"xxx") or SCREEN(displayNo,255, "xxx") - 
      // scroll the contents of the buffer and put the new line
      // at the bottom of the screen
      for (int row=1; row<_numRows; row++) {
        strncpy(&_buffer[(row-1)*_numCols], &_buffer[row*_numCols], _numCols);
        _rowGeneration[row-1]++;
      }
      line = _numRows-1;
    } else if (line >= _numRows) 
      line = _numRows - 1;  // Overwrite bottom line.

    _rowNo = line;
    // Fill line with blanks
    for (_colNo = 0; _colNo < _numCols; _colNo++)
      _buffer[_rowNo*_numCols+_colNo] = ' ';
    _colNo = 0;
    // Mark that the buffer has been touched.  It will start being 
    // sent to the screen on the next loop entry, by which time
    // the line should have been written to the buffer.
    _rowGeneration[_rowNo]++;
    // Indicate that the output loop is to start updating the screen again from
    // row 0.  Otherwise, on a full screen rewrite the bottom part may be drawn
    // before the top part!
    _startAgain = true;
  }

  // Write one character to the screen referenced in the last setRow() call.
  virtual size_t _write(uint8_t c) override {
    // Write character to buffer (if there's space)
    if (_colNo < _numCols) {
      _buffer[_rowNo*_numCols+_colNo++] = c;
    }
    return 1;
  }

  // Write blanks to all of the screen buffer
  void _clear() {
    // Clear buffer
    memset(_buffer, ' ', _numCols*_numRows);
    _colNo = 0;
    _rowNo = 0;
  }

};

#endif // IO_HALDisplay_H