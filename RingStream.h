#ifndef RingStream_h
#define RingStream_h
/*
 *  © 2020-2021 Chris Harlow
 *  All rights reserved.
 *  
 *  This file is part of DCC-EX CommandStation-EX
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

#include <Arduino.h>
#include "FSH.h"
  
class RingStream : public Print {

  public:
    RingStream( const uint16_t len);
    static const int THIS_IS_A_RINGSTREAM=777;
    virtual size_t write(uint8_t b);

    // This availableForWrite function is subverted from its original intention so that a caller 
    // can destinguish between a normal stream and a RingStream. 
    // The Arduino compiler does not support runtime dynamic cast to perform
    // an instranceOf check. 
    // This is necessary since the Print functions are mostly not virtual so 
    // we cant override the print(__FlashStringHelper *) function.
   virtual int availableForWrite() override;
    using Print::write;
    size_t printFlash(const FSH * flashBuffer);
    int read();
    int count();
    int freeSpace();
    void mark(uint8_t b);
    bool commit();
    uint8_t peekTargetMark();
    void flush();
    void info();
    byte readRawByte();
    inline int peek() {
      if ((_pos_read==_pos_write) && !_overflow) return -1;  // empty
      return _buffer[_pos_read];
    };
    static const byte NO_CLIENT=255;
 private:
   int _len;
   int _pos_write;
   int _pos_read;
   bool _overflow;
   int _mark;
   int _count;
   byte * _buffer;
   char * _flashInsert;
   byte _ringClient = NO_CLIENT;
};

#endif
