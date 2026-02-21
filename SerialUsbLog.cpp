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

/*
 * SerialUsbLog.cpp
 *
 * This code acts as a serial output log collector.
 * This is pointed to by defines.h as the USB_SERIAL object so that the remainder
 * of the code does not have to be aware of the difference.
 *
 * All output to the serial log is collected in a rolling buffer that can be:
 *  - dumped as a one-shot stream (streamOut)
 *  - or streamed incrementally via a lightweight HTTP endpoint
 *
 * ESP32 Web UI improvements:
 *  - Small HTML “console” page at "/" (no giant page reloads)
 *  - Incremental log feed at "/log?from=<seq>&chunk=<n>"
 *  - Full buffer download at "/dump"
 *  - Nicer UX: Follow (tail) behaviour only when user is at bottom, Pause/Resume,
 *    Wrap toggle, Clear, Copy, simple Filter.
 *
 * Notes:
 *  - For incremental output, the buffer includes < > & chars as plain text
 */

#include "defines.h"
#ifdef ENABLE_SERIAL_LOG
// This entire file is ignored if ENABLE_SERIAL_LOG is not defined in defines.h.
#include "Arduino.h"
#include "DIAG.h"
#include "SerialUsbLog.h"
#include "StringBuffer.h"

#if WIFI_ON
  #include <WiFi.h>
  WiFiServer server(80);
#else
  #include <STM32Ethernet.h>
  EthernetServer server(80);
#endif

#include "SerialUsbLog.h"

  // Log buffer size. You you have RAM to spare on thyese devices, so feel free to bump this.
  // Keep it sensible; very large buffers make /dump and filter operations heavier.
  #ifndef LOG_BUFFER
    #define LOG_BUFFER 8192
  #endif

  // Maximum bytes returned per /log request (upper safety bound)
  #ifndef LOG_CHUNK_MAX
    #define LOG_CHUNK_MAX 1000
  #endif

// Global instance
SerialUsbLog SerialLog(LOG_BUFFER, &Serial);

// --------------------------- Small helpers (ESP32 only) ---------------------------

static inline bool startsWithNoCase(const String& s, const char* prefix) {
  int n = (int)strlen(prefix);
  if ((int)s.length() < n) return false;
  for (int i = 0; i < n; i++) {
    char a = s[i];
    char b = prefix[i];
    if (a >= 'A' && a <= 'Z') a = (char)(a - 'A' + 'a');
    if (b >= 'A' && b <= 'Z') b = (char)(b - 'A' + 'a');
    if (a != b) return false;
  }
  return true;
}

static String urlPathOnly(const String& uri) {
  int q = uri.indexOf('?');
  return (q >= 0) ? uri.substring(0, q) : uri;
}

static int queryParamInt(const String& uri, const char* key, int defaultValue) {
  int q = uri.indexOf('?');
  if (q < 0) return defaultValue;

  String qs = uri.substring(q + 1);
  String k = String(key) + "=";

  int p = qs.indexOf(k);
  if (p < 0) return defaultValue;

  int vStart = p + k.length();
  int amp = qs.indexOf('&', vStart);
  String v = (amp >= 0) ? qs.substring(vStart, amp) : qs.substring(vStart);

  v.trim();
  if (!v.length()) return defaultValue;
  return v.toInt();
}

static void drainHttpHeaders(Client& client) {
  // Read until blank line. Keep it short to avoid blocking too long.
  uint32_t start = millis();
  while (client.connected() && (millis() - start) < 50) {
    String line = client.readStringUntil('\n');
    if (line.length() == 0 || line == "\r") break;
  }
}


/**
 * Constructor
 * @param len Maximum length of the log buffer
 * @param serialPort underlying serial port (eg. &Serial)
 */
SerialUsbLog::SerialUsbLog(const uint16_t len, HardwareSerial* serialPort) {
  _bufferSize = len;
  _buffer = new byte[len];
  _pos_write = 0;
  _overflow = false;
  _serialPort = serialPort;

  // Monotonic write sequence counter (increments per byte stored into the ring).
  _seq_write = 0;
}

/**
 * Write a single byte to the log buffer and to the underlying serial port.
 */
size_t SerialUsbLog::write(uint8_t b) {
  _serialPort->write(b);

  // Store directly (no translation)
  shoveToBuffer(b);

  return 1;
}

/**
 * Internal ring-buffer write (single byte).
 * On ESP32 we guard ring/seq updates with a critical section.
 */
void SerialUsbLog::shoveToBuffer(uint8_t b) {
// #if defined(ARDUINO_ARCH_ESP32)
//   portENTER_CRITICAL(&_mux);
// #endif

  if (_pos_write >= _bufferSize) {
    _overflow = true;
    _pos_write = 0;
  }
  _buffer[_pos_write++] = b;
  _seq_write++;

// #if defined(ARDUINO_ARCH_ESP32)
//   portEXIT_CRITICAL(&_mux);
// #endif
}

/**
 * Stream the entire log buffer contents to a target Print stream.
 * This is the legacy one-shot dump of the current ring buffer snapshot.
 */
void SerialUsbLog::streamOut(Print* targetStream) {
  if (targetStream == nullptr) return;

// #if defined(ARDUINO_ARCH_ESP32)
//   portENTER_CRITICAL(&_mux);
// #endif

  if (_overflow) {
    // output from current position to end, then start to current position
    targetStream->write(_buffer + _pos_write, _bufferSize - _pos_write);
    targetStream->write(_buffer, _pos_write);
  } else {
    targetStream->write(_buffer, _pos_write);
  }

// #if defined(ARDUINO_ARCH_ESP32)
//   portEXIT_CRITICAL(&_mux);
// #endif
}

/**
 * Return current write sequence number.
 * (Monotonic counter of bytes written into the ring.)
 */
uint32_t SerialUsbLog::getWriteSeq() const {
  return _seq_write;
}

/**
 * Stream from a given sequence number up to maxBytes (incremental streaming).
 *
 * fromSeq:
 *   - Client's last seen sequence number.
 *   - If too old, we fast-forward to earliest available.
 *
 * nextSeq (out):
 *   - Updated sequence number after streaming.
 *
 * Returns:
 *   - Number of bytes written to targetStream.
 */
size_t SerialUsbLog::streamOutFrom(Print* targetStream, uint32_t fromSeq, size_t maxBytes, uint32_t& nextSeq) {
  if (!targetStream) { nextSeq = fromSeq; return 0; }

// #if defined(ARDUINO_ARCH_ESP32)
//   portENTER_CRITICAL(&_mux);
// #endif

  const uint32_t writeSeq = _seq_write;
  const uint32_t earliest = (writeSeq > (uint32_t)_bufferSize) ? (writeSeq - (uint32_t)_bufferSize) : 0;

  uint32_t start = fromSeq;
  if (start < earliest) start = earliest;
  if (start > writeSeq) start = writeSeq;

  uint32_t available = writeSeq - start;
  if (available > (uint32_t)maxBytes) available = (uint32_t)maxBytes;

  for (uint32_t s = 0; s < available; s++) {
    const uint32_t seq = start + s;
    const uint32_t pos = seq % (uint32_t)_bufferSize;
    targetStream->write(_buffer[pos]);
  }

  nextSeq = start + available;

// #if defined(ARDUINO_ARCH_ESP32)
//   portEXIT_CRITICAL(&_mux);
// #endif

  return (size_t)available;
}

// begin() shim
void SerialUsbLog::begin(unsigned long baud) {
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

// read() shim
int SerialUsbLog::read() {
  return _serialPort->read();
}

// peek() shim
int SerialUsbLog::peek() {
  return _serialPort->peek();
}

/**
 * Lightweight HTTP server loop 
 *
 * Endpoints:
 *  - GET /                : HTML log console (polling)
 *  - GET /log?from=N&chunk=M : returns text/plain chunk and X-Next-Seq header
 *  - GET /dump            : full buffer dump as text/plain download
 */
void SerialUsbLog::loop() {

  static bool started = false;
  if (!started) {
    server.begin();
    started = true;
    return;
  }

  auto client = server.available();
  if (!client) return;

  StringBuffer dummyClient(2048); // buffering client for response construction
  
  // Read request line: "GET /path?... HTTP/1.1"
  String reqLine = client.readStringUntil('\r');
  if (reqLine.length() == 0) { client.stop(); return; }

  int sp1 = reqLine.indexOf(' ');
  int sp2 = reqLine.indexOf(' ', sp1 + 1);
  if (sp1 < 0 || sp2 < 0) { client.stop(); return; }

  String method = reqLine.substring(0, sp1);
  String uri    = reqLine.substring(sp1 + 1, sp2);
  String path   = urlPathOnly(uri);

  // Drain the rest of the headers to keep the TCP state clean-ish.
  drainHttpHeaders(client);
   
  if (method != "GET") {
    dummyClient.println(
      "HTTP/1.1 405 Method Not Allowed\r\n"
      "Connection: close\r\n\r\n"
    );
  }

  // ----------------------------- /log incremental feed -----------------------------
  else if (path == "/log") {
    uint32_t from = (uint32_t)queryParamInt(uri, "from", 0);

    int chunk = queryParamInt(uri, "chunk", 1024);
    if (chunk < 64) chunk = 64;
    if (chunk > LOG_CHUNK_MAX) chunk = LOG_CHUNK_MAX;
    
    // Compute next seq (for header) using the same logic as streamOutFrom.
    uint32_t writeSeq = SerialLog.getWriteSeq();
    uint32_t earliest = (writeSeq > (uint32_t)_bufferSize) ? (writeSeq - (uint32_t)_bufferSize) : 0;
    uint32_t start = from;
    if (start < earliest) start = earliest;
    if (start > writeSeq) start = writeSeq;
    uint32_t avail = writeSeq - start;
    if (avail > (uint32_t)chunk) avail = (uint32_t)chunk;
    uint32_t next = start + avail;

    dummyClient.print(
      "HTTP/1.1 200 OK\r\n"
      "Content-Type: text/plain; charset=utf-8\r\n"
      "Cache-Control: no-store\r\n"
      "Connection: close\r\n"
      "X-Next-Seq: "
    );
    dummyClient.print(next);
    dummyClient.print("\r\n\r\n");

    uint32_t nextSeqOut = from;
    SerialLog.streamOutFrom(&dummyClient, from, (size_t)chunk, nextSeqOut);
  }

  // --------------------------------- /dump full dump --------------------------------
  else if (path == "/dump") {
    dummyClient.print(
      "HTTP/1.1 200 OK\r\n"
      "Content-Type: text/plain; charset=utf-8\r\n"
      "Cache-Control: no-store\r\n"
      "Connection: close\r\n"
      "Content-Disposition: attachment; filename=\"dcc-ex-log.txt\"\r\n\r\n"
    );

    // One-shot dump of current ring snapshot
    SerialLog.streamOut(&dummyClient);
}

  // --------------------------------- / HTML shell ---------------------------------
else  if (path == "/" ) {
    dummyClient.print(
      "HTTP/1.1 200 OK\r\n"
      "Content-Type: text/html; charset=utf-8\r\n"
      "Cache-Control: no-store\r\n"
      "Connection: close\r\n\r\n"
      #include "SerialUsbLog.html.h"
    );

  }

  else if (path == "/style.css" ) {
    dummyClient.print(
      "HTTP/1.1 200 OK\r\n"
      "Content-Type: text/css; charset=utf-8\r\n"
      "Cache-Control: no-store\r\n"
      "Connection: close\r\n\r\n"
      #include "SerialUsbLog.style.css.h"
    );
  }

 else if (path == "/script1.js" ) {
    dummyClient.print(
      "HTTP/1.1 200 OK\r\n"
      "Content-Type: text/javascript; charset=utf-8\r\n"
      "Cache-Control: no-store\r\n"
      "Connection: close\r\n\r\n"
      #include "SerialUsbLog.script1.js.h"
    );

  }
  
  else if (path == "/script2.js" ) {
    dummyClient.print(
      "HTTP/1.1 200 OK\r\n"
      "Content-Type: text/javascript; charset=utf-8\r\n"
      "Cache-Control: no-store\r\n"
      "Connection: close\r\n\r\n"
      #include "SerialUsbLog.script2.js.h"
    );
  }
else {
  // --------------------------------- 404 ---------------------------------
  dummyClient.print(
    "HTTP/1.1 404 Not Found\r\n"
    "Connection: close\r\n\r\n"
  );
}
  auto length=dummyClient.getLength();
  DIAG(F("http %s %s length=%d"), method.c_str(),path.c_str(), length);
  client.print(dummyClient.getString());
  client.stop();
}
// --------------------------- End of SerialUsbLog.cpp ---------------------------
#endif // ENABLE_SERIAL_LOG
