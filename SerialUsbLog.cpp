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

/* This code acts as a serial output log collector.
 * This is pointed to by defines.h as the USB_SERIAL object so that the remainder of the code 
 * does not have to be aware of the difference.
 * All output to the serial log is collected in a rolling buffer
 * that can be transmitted as simple wen page to a requesting web browser. 
 */

#include "SerialUsbLog.h"
#include "Arduino.h"
#include "DIAG.h"

#if defined(ARDUINO_ARCH_ESP32)
#include <WiFi.h>
WiFiServer server(80);
#define LOG_BUFFER 4096
#else
#define LOG_BUFFER 1024
#endif

// Global instance
SerialUsbLog SerialLog(LOG_BUFFER, &Serial);

/**
 * Constructor
 * @param len Maximum length of the log buffer
 */
SerialUsbLog::SerialUsbLog(const uint16_t len, HardwareSerial* serialPort ) {
    _bufferSize = len;
    _buffer = new byte[len];
    _pos_write = 0;
    _overflow = false;
    _serialPort = serialPort;
}

/**
 * Write a single byte to the log buffer
 * @param b The byte to write
 * @return Number of bytes written (1 if successful)
 */
size_t SerialUsbLog::write(uint8_t b) {
    _serialPort->write(b);
    // HTML Char entities must be translated
    if (b=='<') {
        shoveToBuffer('&');
        shoveToBuffer('l');
        shoveToBuffer('t');
        shoveToBuffer(';');
    }    
    else if (b=='>') {
        shoveToBuffer('&');
        shoveToBuffer('g');
        shoveToBuffer('t');
        shoveToBuffer(';');
    }
    else shoveToBuffer(b);
    return 1;
}

void SerialUsbLog::shoveToBuffer(uint8_t b) {
    if (_pos_write >= _bufferSize) {
        _overflow = true;
        _pos_write = 0;
    }
    _buffer[_pos_write++] = b;
}

/**
 * Stream the log buffer contents to a target Print stream
 * @param targetStream The Print stream to output to
 */
void SerialUsbLog::streamOut(Print * targetStream) {
    if (targetStream == nullptr) {
        return;
    }
    
    // Output the buffer contents
    if (_overflow) {
        // If overflow occurred, output from current position to end, then start to current position
        targetStream->write(_buffer + _pos_write, _bufferSize - _pos_write);
        targetStream->write(_buffer, _pos_write);
    } else {
        // Otherwise just output from start to current position
        targetStream->write(_buffer, _pos_write);
    }
}

// begin() shim
void SerialUsbLog::begin( unsigned long baud ) {
      _serialPort->begin(baud);
    }

// while(!Serial) shim 
bool SerialUsbLog::operator!() const {
    return _serialPort == nullptr;
}

// available() shim
int SerialUsbLog::available() {
    return _serialPort->available();
}

// read()) shim
int SerialUsbLog::read() {
    return _serialPort->read();
}

// peek() shim
int SerialUsbLog::peek() {
    return _serialPort->peek();
}

void SerialUsbLog::webserverLoop() {
    // This is a no-op if not ESP32. 
    #ifdef ARDUINO_ARCH_ESP32
    static bool started=false;
    if (!started) {
      server.begin();
      started=true;
      return;
    }
    auto client= server.available();
  if (!client) return;
  auto header= client.readStringUntil('\r'); // read the first line of the request
  if (header.indexOf(" / ")<0) return;
  DIAG(F("http:%s"),header.c_str());          // print a message out in the serial port
  // USB_SERIAL.flush();
  client.println(
    "HTTP/1.1 200 OK\r\n" 
    "Content-type:text/html\r\n"
    "Connection: close\r\n\r\n"
    "<!DOCTYPE html>\r\n"
    "<html><head><title>DCC-EX Server Log</title></head>"
    "<body><pre><code>");
  SerialLog.streamOut(&client);
  client.println("\r\n</code></pre></body></html>\n");
  client.flush();
  #endif
}