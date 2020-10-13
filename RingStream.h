#ifndef RingStream_h
#define RingStream_h
/*

 (c) 2015 Ingo Fischer
 buffer serial device
 based on Arduino SoftwareSerial

 Constructor warning messages fixed by Chris Harlow.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

*/
#include <Arduino.h>
  
class RingStream : public Print {

  public:
    RingStream( const uint16_t len);
  
    virtual size_t write(uint8_t b);
    using Print::write;
    int read();
    int count();

 private:
   int _len;
   int _pos_write;
   int _pos_read;
   bool _overflow;
   byte * _buffer;
};

#endif
