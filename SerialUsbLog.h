#ifndef SerialUsbLog_h
#define SerialUsbLog_h
/*
 *  © 2026 Chris Harlow
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
  
class SerialUsbLog : public Stream {

  public:
    SerialUsbLog( const uint16_t len, HardwareSerial* serialPort  );
    void begin( unsigned long baud );
    virtual size_t write(uint8_t b);
    using Print::write;
    void streamOut(Print * targetStream);
    bool operator!() const;
    void shoveToBuffer(uint8_t b);
    virtual int available();
    virtual int read();
    virtual int peek();
    void webserverLoop();

 private:
   HardwareSerial * _serialPort;
   int _pos_write;
   bool _overflow;
   byte * _buffer;
   int _bufferSize;
};
extern SerialUsbLog SerialLog;

#endif
