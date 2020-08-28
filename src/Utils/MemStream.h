/*
 *  MemStream.h
 *  (c) 2015 Ingo Fischer
 * 
 *  This file is part of CommandStation-EX.
 *
 *  CommandStation-EX is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  CommandStation-EX is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with CommandStation-EX.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef UTILS_MEMSTREAM_H_
#define UTILS_MEMSTREAM_H_

#include <inttypes.h>
#include <Stream.h>
#include <avr/pgmspace.h>

class MemStream : public Stream {
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

#endif  // UTILS_MEMSTREAM_H_