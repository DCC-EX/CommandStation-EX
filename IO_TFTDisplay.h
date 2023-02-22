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
 * This driver provides a way of driving a ST7735 TFT display through SCREEN(disp,line,"text").
 * If the line specified is off the screen then the text in the bottom line will be 
 * overwritten.  There is however a special case that if line 255 is specified, 
 * the existing text will scroll up and the new line added to the bottom
 * line of the screen.
 * 
 * To install, use the following command in myHal.cpp:

 *    TFTDisplay::create(address, width, height);
 * 
 * where address is the I2C address (0x3c or 0x3d),
 * width is the width in pixels of the display, and
 * height is the height in pixels of the display.
 * 
 */


#ifndef IO_TFTDISPLAY_H
#define IO_TFTDDISPLAY_H

#include "IODevice.h"
#include "DisplayInterface.h"
#include "version.h"


template <class T> 
class TFTDisplay : public IODevice, public DisplayInterface {
private:
  uint8_t _displayNo = 0;
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
  DisplayInterface *_nextDisplay = NULL;
  uint8_t _selectedDisplayNo = 0;

public:
  //  Static function to handle "TFTDisplay::create(...)" calls.
  static void create(I2CAddress i2cAddress, int width = 128, int height=64) {
    /* if (checkNoOverlap(i2cAddress)) */ new TFTDisplay(0, i2cAddress, width, height);
  } 
  static void create(uint8_t displayNo, I2CAddress i2cAddress, int width = 128, int height=64) {
    /* if (checkNoOverlap(i2cAddress)) */ new TFTDisplay(displayNo, i2cAddress, width, height);
  } 

protected:
  // Constructor
  TFTDisplay(uint8_t displayNo, I2CAddress i2cAddress, int width, int height) {
    _displayDriver = new T(i2cAddress, width, height);
    _displayNo = displayNo;
    _I2CAddress = i2cAddress;
    _width = width;
    _height = height;
    _numCols = (_width+5) / 6;    // character block 6 x 8, round up 
    _numRows = _height / 8;       // Round down

    _charPosToScreen = _numCols;

    // Allocate arrays
    _buffer = (char *)calloc(_numRows*_numCols, sizeof(char));
    _rowGeneration = (uint8_t *)calloc(_numRows, sizeof(uint8_t));
    _lastRowGeneration = (uint8_t *)calloc(_numRows, sizeof(uint8_t));
    // Fill buffer with spaces
    memset(_buffer, ' ', _numCols*_numRows);

    _displayDriver->clearNative();

    // Is this the main display?
    if (_displayNo == 0) {
      // Set first two lines on screen
      this->setRow(0, 0);
      print(F("DCC-EX v"));
      print(F(VERSION));
      setRow(0, 1);
      print(F("Lic GPLv3"));
    }
   
    // Store pointer to this object into CS display hook, so that we
    // will intercept any subsequent calls to displayHandler methods.
    // Make a note of the existing display reference, to that we can
    // pass on anything we're not interested in.
    _nextDisplay = DisplayInterface::displayHandler;
    DisplayInterface::displayHandler = this;

    addDevice(this);
  }
  
  
  void screenUpdate() {
    // Loop through the buffer and if a row has changed
    // (rowGeneration[row] is changed) then start writing the
    // characters from the buffer, one character per entry, 
    // to the screen until that row has been refreshed.

    // First check if the OLED driver is still busy from a previous 
    // call.  If so, don't to anything until the next entry.
    if (!_displayDriver->isBusy()) {
      // Check if we've just done the end of a row or just started
      if (_charPosToScreen >= _numCols) {
        // Move to next line
        if (++_rowNoToScreen >= _numRows)
          _rowNoToScreen = 0; // Wrap to first row

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

      DIAG(F("TFTDisplay installed on address %s as screen %d"), 
        _I2CAddress.toString(), _displayNo);

      // Force all rows to be redrawn
      for (uint8_t row=0; row<_numRows; row++)
        _rowGeneration[row]++;
      
      // Start with top line (looks better)
      _rowNoToScreen = _numRows;
      _charPosToScreen = _numCols;
    }
  }

  void _loop(unsigned long) override {
    screenUpdate();
  }
  
  /////////////////////////////////////////////////
  // DisplayInterface functions
  // 
  /////////////////////////////////////////////////
  
public:
  void loop() override {
    screenUpdate();
    if (_nextDisplay) 
      _nextDisplay->loop();  // continue to next display
    return;
  }

  // Position on nominated line number (0 to number of lines -1)
  // Clear the line in the buffer ready for updating
  // The displayNo referenced here is remembered and any following
  // calls to write() will be directed to that display.
  void setRow(uint8_t displayNo, byte line) override {
    _selectedDisplayNo = displayNo;
    if (displayNo == _displayNo) {
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
      // Mark that the buffer has been touched.  It will be 
      // sent to the screen on the next loop entry, by which time
      // the line should have been written to the buffer.
      _rowGeneration[_rowNo]++;

    }
    if (_nextDisplay) 
      _nextDisplay->setRow(displayNo, line); // Pass to next display

  }

  // Write one character to the screen referenced in the last setRow() call.
  size_t write(uint8_t c) override {
    if (_selectedDisplayNo == _displayNo) {
      // Write character to buffer (if there's space)
      if (_colNo < _numCols) {
        _buffer[_rowNo*_numCols+_colNo++] = c;
      }
    }
    if (_nextDisplay) 
      _nextDisplay->write(c);
    return 1;
  }

  // Write blanks to all of the screen (blocks until complete)
  void clear (uint8_t displayNo) override {
    if (displayNo == _displayNo) {
      // Clear buffer
      for (_rowNo = 0; _rowNo < _numRows; _rowNo++) {
        setRow(displayNo, _rowNo);
      }
      _rowNo = 0;
    }
    if (_nextDisplay)
      _nextDisplay->clear(displayNo);  // Pass to next display
  }
  
  // Display information about the device.
  void _display() {
    DIAG(F("TFTDisplay %d Configured addr %s"), _displayNo, _I2CAddress.toString());
  }

};

#endif // IO_TFTDDISPLAY_H