/*
 *  © 2022 Chris Harlow
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

#include "StringBuffer.h"
#include "DIAG.h"

StringBuffer::StringBuffer() {
    flush();
};

char * StringBuffer::getString() { 
   return _buffer;
}

void StringBuffer::flush() {
    _pos_write=0;
    _buffer[0]='\0';
}

size_t StringBuffer::write(uint8_t b) {
  if (_pos_write>=buffer_max) return 0;
  _buffer[_pos_write] = b;
  ++_pos_write;
  _buffer[_pos_write]='\0';
  return 1;
}


