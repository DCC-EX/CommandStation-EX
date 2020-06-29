/*

  (c) 2015 Ingo FIscher
  buffer serial device
  based on Arduino SoftwareSerial

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

#ifndef MemStream_h
#define MemStream_h

#include <inttypes.h>
#include <Stream.h>
#include <avr/pgmspace.h>

class MemStream : public Stream
{
private:
   uint8_t * _buffer;
   const uint16_t _len;
   bool _buffer_overflow;
   uint16_t _pos_read;
   uint16_t _pos_write;
   bool _allowWrite;


public:
  // public methods
  MemStream(uint8_t *buffer, const uint16_t len, uint16_t content_len = 0, bool allowWrite=true);
  ~MemStream() {}

  operator const uint8_t *() const { return _buffer; }
  operator const char *() const { return (const char*)_buffer; }

  uint16_t current_length() const { return _pos_write; }

  bool listen() { return true; }
  void end() {}
  bool isListening() { return true; }
  bool overflow() { bool ret = _buffer_overflow; _buffer_overflow = false; return ret; }
  int peek();

  virtual size_t write(uint8_t byte);
  virtual int read();
  virtual int available();
  virtual void flush();

  void setBufferContent(uint8_t *buffer, uint16_t content_len);
  void setBufferContentFromProgmem(uint8_t *buffer, uint16_t content_len);
  void setBufferContentPosition(uint16_t read_pos, uint16_t write_pos);

  using Print::write;
};

#endif
