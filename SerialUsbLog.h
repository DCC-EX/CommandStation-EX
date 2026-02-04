#ifndef SerialUsbLog_h
#define SerialUsbLog_h
/*
 *  © 2026 Chris Harlow
 *  © 2026 Paul M. Antoine
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
    // NEW: incremental streaming support
    uint32_t getWriteSeq() const;
    size_t streamOutFrom(Print* targetStream, uint32_t fromSeq, size_t maxBytes, uint32_t& nextSeq);
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
   // NEW
   volatile uint32_t _seq_write;

#if defined(ARDUINO_ARCH_ESP32)
   // protect buffer/seq from concurrent access
   portMUX_TYPE _mux = portMUX_INITIALIZER_UNLOCKED;
#endif
};
extern SerialUsbLog SerialLog;

#endif
