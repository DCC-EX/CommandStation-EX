 /*
 *  © 2022 Chris Harlow
 *  All rights reserved.
 *  
 *  This file is part of DCC++EX
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

#ifndef StringBuffer_h
#define StringBuffer_h
#include <Arduino.h>

class StringBuffer : public Print {
  public:
    StringBuffer(); 
    // Override Print default
    virtual size_t write(uint8_t b);
    void flush();
    char * getString();
  private:
    static const int  buffer_max=64; // enough for long text msgs to throttles  
    int16_t _pos_write;
    char _buffer[buffer_max+2];
};

#endif