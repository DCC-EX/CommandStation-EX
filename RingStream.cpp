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
#include "defines.h"
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
#if defined(ARDUINO_ARCH_ESP32)
  _bufMux = portMUX_INITIALIZER_UNLOCKED;
#endif
}

size_t RingStream::write(uint8_t b) {
  if (_overflow) return 0;
  portENTER_CRITICAL(&_bufMux);
  _buffer[_pos_write] = b;
  ++_pos_write;
  if (_pos_write==_len) _pos_write=0;
  if (_pos_write==_pos_read) {
    _overflow=true; 
    portEXIT_CRITICAL(&_bufMux);
    return 0;
  }
  _count++;
  portEXIT_CRITICAL(&_bufMux);
  return 1;
}

int RingStream::read(byte advance) {
  if ((_pos_read==_pos_write) && !_overflow) return -1;  // empty
  if (_pos_read == _mark) return -1;
  portENTER_CRITICAL(&_bufMux);
  byte b=_buffer[_pos_read];
  _pos_read += advance;
  if (_pos_read==_len) _pos_read=0;
  _overflow=false;
  portEXIT_CRITICAL(&_bufMux);
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
    //DIAG(F("Mark1 len=%d count=%d pr=%d pw=%d m=%d"),_len, _count,_pos_read,_pos_write,_mark);
    portENTER_CRITICAL(&_bufMux);
    _mark=_pos_write;
    write(b); // client id
    write((uint8_t)0);  // count MSB placemarker
    write((uint8_t)0);  // count LSB placemarker
    _count=0;
    portEXIT_CRITICAL(&_bufMux);
}

// peekTargetMark is used by the parser stash routines to know which client
// to send a callback response to some time later. 
uint8_t RingStream::peekTargetMark() {
  return _buffer[_mark];
}

void RingStream::info() {
  DIAG(F("Info len=%d count=%d pr=%d pw=%d m=%d"),_len, _count,_pos_read,_pos_write,_mark);
}

bool RingStream::commit() {
  //DIAG(F("Commit1 len=%d count=%d pr=%d pw=%d m=%d"),_len, _count,_pos_read,_pos_write,_mark);
  portENTER_CRITICAL(&_bufMux);
  if (_overflow) {
        DIAG(F("RingStream(%d) commit(%d) OVERFLOW"),_len, _count);
        // just throw it away 
        _pos_write=_mark;
        _overflow=false;
	portEXIT_CRITICAL(&_bufMux);
	return false; // commit failed
  }
  if (_count==0) {
    // ignore empty response 
    _pos_write=_mark;
    portEXIT_CRITICAL(&_bufMux);
    return true; // true=commit ok
  }
  // Go back to the _mark and inject the count 1 byte later
  _mark++;
  if (_mark==_len) _mark=0;
  _buffer[_mark]=highByte(_count);
  _mark++;
  if (_mark==_len) _mark=0;
  _buffer[_mark]=lowByte(_count);
  _mark=_len+1;
  //DIAG(F("Commit2 len=%d count=%d pr=%d pw=%d m=%d"),_len, _count,_pos_read,_pos_write,_mark);
  portEXIT_CRITICAL(&_bufMux);
  return true; // commit worked
}
void RingStream::flush() {
  _pos_write=0;
  _pos_read=0;
  _buffer[0]=0;
}
void RingStream::printBuffer(Print * stream) {
  _buffer[_pos_write]='\0';
  stream->print((char *)_buffer);
}
