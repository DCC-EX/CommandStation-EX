#ifndef RingStream_h
#define RingStream_h
/*
 *  © 2020, Chris Harlow. All rights reserved.
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
  
class RingStream : public Print {

  public:
    RingStream( const uint16_t len);
  
    virtual size_t write(uint8_t b);
    using Print::write;
    int read();
    int count();
    int freeSpace();
    void mark(uint8_t b);
    bool commit();
    uint8_t peekTargetMark();
    
 private:
   int _len;
   int _pos_write;
   int _pos_read;
   bool _overflow;
   int _mark;
   int _count;
   byte * _buffer;
};

#endif
