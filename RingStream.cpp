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

#include "RingStream.h"
#include "DIAG.h"

RingStream::RingStream( const uint16_t len)
{
  _len=len;
  _buffer=new byte[len];
  _pos_write=0;
  _pos_read=0;
  _buffer[0]=0;
  _overflow=false;
  _mark=0;
  _count=0; 
}

size_t RingStream::write(uint8_t b) {
  if (_overflow) return 0;
  _buffer[_pos_write] = b;
  ++_pos_write;
  if (_pos_write==_len) _pos_write=0;
  if (_pos_write==_pos_read) {
    _overflow=true; 
    return 0;
  }
  _count++;
  return 1;
}

int RingStream::read() {
  if ((_pos_read==_pos_write) && !_overflow) return -1;  // empty  
  byte b=_buffer[_pos_read];
  _pos_read++;
  if (_pos_read==_len) _pos_read=0;
  _overflow=false;
  return b;
}


int RingStream::count() {
  return (read()<<8) | read(); 
  }

int RingStream::freeSpace() {
  // allow space for client flag and length bytes
  if (_pos_read>_pos_write) return _pos_read-_pos_write-3;
  else return _len - _pos_write + _pos_read-3;  
}


// mark start of message with client id (0...9)
void RingStream::mark(uint8_t b) {
    _mark=_pos_write;
    write(b); // client id
    write((uint8_t)0);  // count MSB placemarker
    write((uint8_t)0);  // count LSB placemarker
    _count=0;
}

// peekTargetMark is used by the parser stash routines to know which client
// to send a callback response to some time later. 
uint8_t RingStream::peekTargetMark() {
  return _buffer[_mark];
}

bool RingStream::commit() {
  if (_overflow) {
        DIAG(F("RingStream(%d) commit(%d) OVERFLOW"),_len, _count);
        // just throw it away 
        _pos_write=_mark;
        _overflow=false;
        return false; // commit failed
  }
  if (_count==0) {
    // ignore empty response 
    _pos_write=_mark;
    return true; // true=commit ok
  }
  // Go back to the _mark and inject the count 1 byte later
  _mark++;
  if (_mark==_len) _mark=0;
  _buffer[_mark]=highByte(_count);
  _mark++;
  if (_mark==_len) _mark=0;
  _buffer[_mark]=lowByte(_count);
  return true; // commit worked
}
