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
 * without waiting for the next 2.5 second refresh.  However, no scrolling
 * takes place, so if the line specified is off the screen then the text
 * will instead be shown on the bottom line of the screen.
 * 
 * To install, use the following command in myHal.cpp:

 *    OLEDDisplay::create(address, width, height);
 * 
 * where address is the I2C address (0x3c or 0x3d),
 * width is the width in pixels of the display, and
 * height is the height in pixels of the display.
 * 
 * Valid width and height are 128x32 (SSD1306 controller), 
 * 128x64 (SSD1306) and 132x64 (SH1106).  The driver uses
 * a 5x7 character set in a 6x8 pixel cell.
 */


#ifndef IO_OLEDDISPLAY_H
#define IO_OLEDDISPLAY_H

#include "IODevice.h"
#include "DisplayInterface.h"
#include "SSD1306Ascii.h"
#include "version.h"

class OLEDDisplay : public IODevice, DisplayInterface {
private:
  // Here we define the device-specific variables.  
  uint8_t _height; // in pixels
  uint8_t _width;  // in pixels
  SSD1306AsciiWire *oled;
  uint8_t _rowNo = 0;   // Row number being written by caller
  uint8_t _colNo = 0;  // Position in line being written by caller
  uint8_t _numRows;
  uint8_t _numCols;
  char *_buffer = NULL;
  uint8_t *_rowGeneration = NULL;
  uint8_t *_lastRowGeneration = NULL;
  uint8_t _rowNoToScreen = 0; 
  uint8_t _charPosToScreen = 0;

public:
  //  Static function to handle "OLEDDisplay::create(...)" calls.
  static void create(I2CAddress i2cAddress, int width = 128, int height=64) {
    /* if (checkNoOverlap(i2cAddress)) */ new OLEDDisplay(i2cAddress, width, height);
  } 

protected:
  // Constructor
  OLEDDisplay(I2CAddress i2cAddress, int width, int height) {
    _I2CAddress = i2cAddress;
    _width = width;
    _height = height;
    _numCols = _width / 6;    // character block 6 x 8 
    _numRows = _height / 8; 

    // Allocate arrays
    _buffer = (char *)calloc(_numRows*_numCols, sizeof(char));
    _rowGeneration = (uint8_t *)calloc(_numRows, sizeof(uint8_t));
    _lastRowGeneration = (uint8_t *)calloc(_numRows, sizeof(uint8_t));
    
    addDevice(this);
  }

  // Device-specific initialisation
  void _begin() override {
    // Create OLED driver
    oled = new SSD1306AsciiWire();
    // Initialise device
    if (oled->begin(_I2CAddress, _width, _height)) {
      // Store pointer to this object into CS display hook, so that we
      // will intercept any subsequent calls to lcdDisplay methods.
      DisplayInterface::lcdDisplay = this;

      DIAG(F("OLEDDisplay installed on address x%x"), (int)_I2CAddress);

      // First clear the entire screen
      oled->clearNative();
      
      // Set first two lines on screen
      LCD(0,F("DCC++ EX v%S"),F(VERSION));
      LCD(1,F("Lic GPLv3"));
    }
  }

  /////////////////////////////////////////////////
  // DisplayInterface functions
  // 
  // TODO: Limit to one call to setRowNative or writeNative
  // per entry so that the function doesn't block waiting
  // for I2C to complete.
  /////////////////////////////////////////////////
  DisplayInterface* loop2(bool force) override {

    // Loop through the buffer and if a row has changed
    // (rowGeneration[row] is changed) then start writing the
    // characters from the buffer, one character per entry, 
    // to the screen until that row has been refreshed.
    // TODO: Currently this is done all in one go!  Split 
    // it up so that at most one call is made to either
    // setRowNative or writeNative per loop entry - then
    // we shan't have to wait for I2C.
    for (uint8_t row = 0; row < _numRows; row++) {
      if (_rowGeneration[row] != _lastRowGeneration[row]) {
        // Row has been modified, write to screen
        oled->setRowNative(row);
        for (uint8_t _col = 0; _col < _numCols; _col++) {
          oled->writeNative(_buffer[(uint16_t)row*_numCols+_col]);
        }
        _lastRowGeneration[row] = _rowGeneration[row];
      }
    }
    return NULL;
  }
  
  // Position on nominated line number (0 to number of lines -1)
  // Clear the line in the buffer ready for updating
  void setRow(byte line) override {
    if (line >= _numRows) line = _numRows-1;
    _rowNo = line;
    // Fill line with blanks
    for (_colNo = 0; _colNo < _numCols; _colNo++)
      _buffer[_rowNo*_numCols+_colNo] = ' ';
    _colNo = 0;
    // Mark that the buffer has been touched.  It will be 
    // sent to the screen on the next loop entry.
    _rowGeneration[_rowNo]++;
  }

  // Write blanks to all of the screen (blocks until complete)
  void clear () override {
    // Clear buffer
    for (_rowNo = 0; _rowNo < _numRows; _rowNo++) {
      setRow(_rowNo);
    }
    _rowNo = 0;
  }

  // Write one character
  size_t write(uint8_t c) override {
    // Write character to buffer (if space)
    if (_colNo < _numCols)
      _buffer[_rowNo*_numCols+_colNo++] = c;
    return 1;
  }

  // Display information about the device.
  void _display() {
    DIAG(F("OLEDDisplay Configured addr x%x"), (int)_I2CAddress);
  }

};

#endif // IO_OLEDDISPLAY_H