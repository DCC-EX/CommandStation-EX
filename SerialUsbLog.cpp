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
 *  - Incremental log feed at "/log?from=<seq>&chunk=<n>
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
#include "DCCEXParser.h"
#include "SerialUsbLog.html.h"
#include "SerialUsbLog.style.css.h"
#include "SerialUsbLog.script1.js.h"
#include "SerialUsbLog.script2.js.h"
#include "SerialUsbLog.script3.js.h"
#include "NVSTableEditor.html.h"
#include "NVSTable.h"
#include "SerialUsbLog.favicon.h"


#if WIFI_ON
  #include <WiFi.h>
  #include "WifiESP32.h"
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
SerialUsbLog SerialLog(LOG_BUFFER);
StringBuffer dummyClient(2048); // buffering client for response construction

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

static String queryParamRaw(const String& uri, const char* key, String defaultValue) {
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
  return v;
}
static int queryParamInt(const String& uri, const char* key, int defaultValue) {
  String v = queryParamRaw(uri, key, "");
  if (v.length() == 0) return defaultValue;
  return v.toInt();
}

static String uriDecode(const String& str) {
  String result;
  for (int i = 0; i < str.length(); i++) {
    if (str[i] == '%' && i + 2 < str.length()) {
      // hex decode: %20 → space, %2B → +
      char hex[3] = { str[i+1], str[i+2], 0 };
      char c = (char)strtol(hex, NULL, 16);
      result += c;
      i += 2;
    } else if (str[i] == '+') {
      result += ' ';
    } else {
      result += str[i];
    }
  }
  return result;
}

static String queryParamString(const String& uri, const char* key, String defaultValue) {
  String v = queryParamRaw(uri, key, "");
  if (v.length() == 0) return defaultValue;
  return uriDecode(v);
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
 */
SerialUsbLog::SerialUsbLog(const uint16_t len) {
  _bufferSize = len;
  _buffer = new byte[len];
  _pos_write = 0;
  _overflow = false;
  // Monotonic write sequence counter (increments per byte stored into the ring).
  _seq_write = 0;
  _timestampPending = true; // first write will trigger timestamp
}

/**
 * Write a single byte to the log buffer and to the underlying serial port.
 */
size_t SerialUsbLog::write(uint8_t b) {
  Serial.write(b);
  if (_timestampPending) {
    auto timestamp = millis();
    shoveToBuffer('[');
    shoveToBuffer((timestamp / 10000) % 10 + '0');
    shoveToBuffer((timestamp /  1000) % 10 + '0');
    shoveToBuffer((timestamp /   100) % 10 + '0');
    shoveToBuffer((timestamp /    10) % 10 + '0');
    shoveToBuffer((timestamp        ) % 10 + '0');
    shoveToBuffer(']');
    _timestampPending = false;
  }
  // Store directly (no translation)
  shoveToBuffer(b);
  _timestampPending = b=='\n';
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
    Serial.flush();
    Serial.begin(baud);
  }


// while(!Serial) shim
bool SerialUsbLog::operator!() const {
  return !Serial;
}

// available() shim
int SerialUsbLog::available() {
  return Serial.available();
}

// read() shim
int SerialUsbLog::read() {
  return Serial.read();
}

// peek() shim
int SerialUsbLog::peek() {
  return Serial.peek();
}

const char htmlHeader[] PROGMEM = 
  "HTTP/1.1 200 OK\r\n"
  "Content-Type: text/html; charset=utf-8\r\n"
  "Cache-Control: no-store\r\n"
  "Connection: close\r\n\r\n"
;
const char jsHeader[] PROGMEM = 
  "HTTP/1.1 200 OK\r\n"
  "Content-Type: text/javascript; charset=utf-8\r\n"
  "Cache-Control: no-store\r\n"
  "Connection: close\r\n\r\n"
  ;
const char cssHeader[] PROGMEM = 
  "HTTP/1.1 200 OK\r\n"
  "Content-Type: text/css; charset=utf-8\r\n"
  "Cache-Control: no-store\r\n"
  "Connection: close\r\n\r\n"
  ;
const char faviconHeader[] PROGMEM = 
  "HTTP/1.1 200 OK\r\n"
  "Content-Type: image/x-icon\r\n"
  "Cache-Control: public, max-age=86400\r\n"
  "Connection: close\r\n\r\n"
  ;

class LogPage{
  public:
    static LogPage* first;
    LogPage* next; 
    const String path;
    const String displayName;
    String content;
    LogPage(const String& _path,const String& _data, const String& _displayName="") : path(_path), displayName(_displayName), content(_data) {
      next=first;
      first=this;
    }
    void render(Print* targetStream) {
      if (targetStream==nullptr) return;
      if (path.endsWith(".css")) targetStream->print(cssHeader);
      else if (path.endsWith(".js")) targetStream->print(jsHeader);
      else targetStream->print(htmlHeader);
      targetStream->print(content);
    }
};
LogPage* LogPage::first = nullptr;

/**
 * Lightweight HTTP server loop 
 *
 * Endpoints:
 *  - GET /                : HTML log console (polling)
 *  - GET /log?from=N&chunk=M : returns text/plain chunk and X-Next-Seq header
 */
void SerialUsbLog::loop() {

  static bool started = false;
  if (!started 
  #if WIFI_ON  
     && WifiESP::isUp()
  #endif
  ) {
    new LogPage("/style.css", SerialUsbLog_style_css);
    new LogPage("/script1.js", SerialUsbLog_script1_js);
    new LogPage("/script2.js", SerialUsbLog_script2_js);
    new LogPage("/script3.js", SerialUsbLog_script3_js);
    new LogPage("/", SerialUsbLog_html);
    new LogPage("/nvstableeditor.html", NVSTTableEditor_html,"Raw NVS Editor");  
    // user pages may be added later with exrail
    server.begin();
    started = true;
    return;
  }

  auto client = server.available();
  if (!client) return;
  
  // Read request line: "GET /path?... HTTP/1.1"
  String reqLine = client.readStringUntil('\r');
  if (reqLine.length() == 0) { client.stop(); return; }
  if (Diag::WIFI || Diag::ETHERNET) {
    StringFormatter::send(Serial,F("<* http: %s *>\n"), reqLine.c_str());
  }
  int sp1 = reqLine.indexOf(' ');
  int sp2 = reqLine.indexOf(' ', sp1 + 1);
  if (sp1 < 0 || sp2 < 0) { client.stop(); return; }

  String method = reqLine.substring(0, sp1);
  String uri    = reqLine.substring(sp1 + 1, sp2);
  String path   = urlPathOnly(uri);

  // Drain the rest of the headers to keep the TCP state clean-ish.
  drainHttpHeaders(client);
  
  if (method == "POST" && path == "/savenvs") {
    DIAG(F("POST /savenvs"));
    // Handle POST request to NVSTable save endpoint.
    // Preferred format is id=value pairs, values may be int or string.
    String body = client.readStringUntil('\0'); // Read until end of stream 
    DIAG(F("POST body: %s"), body.c_str());
    auto headerEnd = body.indexOf("\r\n\r\n");
    if (headerEnd >= 0) {
      body = body.substring(headerEnd + 4); // Skip past headers
    }
    DIAG(F("POST data: %s"), body.c_str());
    
    // read id=value, id="value" pairs from the full body.
    if (body.indexOf('=') >= 0) {
      NVSTable::applyChanges(body);
    }
    client.print(
      "HTTP/1.1 200 OK\r\n"
      "Content-Type: text/plain; charset=utf-8\r\n"
      "Cache-Control: no-store\r\n"
      "Connection: close\r\n\r\n"
    );
    client.stop();
    return;
  }

  if (method != "GET") {
      client.print(
        "HTTP/1.1 405 Method Not Allowed\r\n"
        "Connection: close\r\n\r\n"
      );
      client.stop();
      return;
    }

  // ----------------------------- /log incremental feed -----------------------------
  if (path == "/log") {
    String cmd= queryParamString(uri, "cmd", "");
    if (cmd.length()>0)  DCCEXParser::parse(cmd.c_str());

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

    client.print(
      "HTTP/1.1 200 OK\r\n"
      "Content-Type: text/plain; charset=utf-8\r\n"
      "Cache-Control: no-store\r\n"
      "Connection: close\r\n"
      "X-Next-Seq: "
    );
    client.print(next);
    client.print("\r\n\r\n");

    uint32_t nextSeqOut = from;
    SerialLog.streamOutFrom(&client, from, (size_t)chunk, nextSeqOut);
    client.stop();
    return;
  }
  
  if (path == "/configs.js") {
    client.print(jsHeader);
    for (auto page = LogPage::first; page != nullptr; page = page->next) {
      if (page->displayName && page->displayName.length() > 0) {
        client.print("addConfig(\"");
        client.print(page->displayName);
        client.print("\", \"");
        client.print(page->path);
        client.print("\");\n");
      }
    }
    client.stop();
    return;
  }

  if (path == "/nvsValues.js") {
    client.print(jsHeader);
    NVSTable::streamJSArray(&client);
    client.stop();
    return;
  }

  if (path == "/favicon.ico") {
    client.print(faviconHeader);
    client.write(favicon_ico, sizeof(favicon_ico));
    client.stop();
    return;
  }

    // try for registered page 
  for (auto page = LogPage::first; page != nullptr; page = page->next) {
    // DIAG(F("SerialUsbLog: %s checking page %s"), path.c_str(), page->path.c_str());
    if (path == page->path) {
      // DIAG(F("SerialUsbLog:found"));
      page->render(&client);
      client.stop();
      return;
    }
  }
  
  // --------------------------------- 404 ---------------------------------
  DIAG(F("SerialUsbLog: 404 %s"), path.c_str());
  client.print(
    "HTTP/1.1 404 Not Found\r\n"
    "Connection: close\r\n\r\n"
  );
  client.stop();
}

void SerialUsbLog::addUserPage(const String& path, const String& content, const String& displayname) {
  // For future expansion: allow user to add custom pages to the web server.
  new LogPage(path, content,displayname);
}
// --------------------------- End of SerialUsbLog.cpp ---------------------------
#endif // ENABLE_SERIAL_LOG
