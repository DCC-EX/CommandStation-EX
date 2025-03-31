/*
 *  © 2025 Harald Barth
 *  
 *  This file is part of CommandStation-EX
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
#ifndef DCCPacket_h
#define DCCPacket_h
#include <strings.h>

class DCCPacket {
public:
  DCCPacket() {
    _len = 0;
    _data = NULL;
  };
  DCCPacket(byte *d, byte l) {
    _len = l;
    _data = new byte[_len];
    for (byte n = 0; n<_len; n++)
      _data[n] = d[n];
  };
  DCCPacket(const DCCPacket &old) {
    _len = old._len;
    _data = new byte[_len];
    for (byte n = 0; n<_len; n++)
      _data[n] = old._data[n];
  };
  DCCPacket &operator=(const DCCPacket &rhs) {
    if (this == &rhs)
      return *this;
    delete[]_data;
    _len = rhs._len;
    _data = new byte[_len];
    for (byte n = 0; n<_len; n++)
      _data[n] = rhs._data[n];
    return *this;
  };
  ~DCCPacket() {
    if (_len) {
      delete[]_data;
      _len = 0;
      _data = NULL;
    }
  };
  inline bool operator==(const DCCPacket &right) {
    if (_len != right._len)
      return false;
    if (_len == 0)
      return true;
    return (bcmp(_data, right._data, _len) == 0);
  };
  void print(HardwareSerial &s) {
    s.print("<* DCCPAKET ");
    for (byte n = 0; n< _len; n++) {
//      byte b = 8;
//      while (b--) {
//	s.print(_data[n]&(1<<b)?"1":"0");
//      }
      s.print(_data[n], HEX);
      s.print(" ");
    }
    s.print("*>\n");
  };
  inline byte len() {return _len;};
  inline byte *data() {return _data;};
private:
  byte _len = 0;
  byte *_data = NULL;
};
#endif
