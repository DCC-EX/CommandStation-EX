#ifndef IO_EX_IOEXPANDER_TCP_H
#define IO_EX_IOEXPANDER_TCP_H

/*
 * IO_EXIOExpander_TCP.h
 *
 * IODevice subclass for EX-IOExpander over TCP/IP.
 *
 * CommandStation connects as a TCP client to an IOExpander TCP server.
 * Protocol framing:
 *   Request:  [CMD][LEN][PAYLOAD...]
 *   Response: [RCMD][RLEN][PAYLOAD...]
 *
 * IMPORTANT: For maximum compatibility with the existing I2C handshake logic,
 * this driver expects the EXIOINIT response payload to be:
 *   RCMD = EXIOPINS, RLEN = 3, PAYLOAD = [EXIOPINS][numDigitalPins][numAnaloguePins]
 * (i.e. the first payload byte repeats EXIOPINS as a tag).
 *
 * Usage in myAutomation.h (suggested numeric-only form):
 *   HAL(EXIOExpander_TCP,800,18,192,168,1,200)          // default port 2560
 *   HAL(EXIOExpander_TCP,800,18,192,168,1,200,2560)     // explicit port
 *
 * IMPORTANT LIMITATION:
 * - ESP-AT (WifiInterface) is AT-command Stream based and does NOT expose an Arduino Client.
 *   This TCP HAL requires a Client-based network stack (ESP32 WiFi or Ethernet).
 */

#include "IODevice.h"
#include "DIAG.h"
#include "FSH.h"

#include <Client.h>
#include "EXIO_TCPClientProvider.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////
class EXIOExpander_TCP : public IODevice {
public:
  enum ProfileType : uint8_t {
    Instant    = 0,
    UseDuration= 0,
    Fast       = 1,
    Medium     = 2,
    Slow       = 3,
    Bounce     = 4,
    NoPowerOff = 0x80,
  };

  static EXIOExpander_TCP* findByVpin(VPIN vpin) {
    for (EXIOExpander_TCP* ex = _expanderHead(); ex; ex = ex->_nextExpander) {
      if (vpin >= ex->_firstVpin && vpin < (VPIN)(ex->_firstVpin + ex->_nPins)) return ex;
    }
    return nullptr;
  }

  // HAL factory (numeric-only, macro-friendly)
  static void create(VPIN vpin, int nPins,
                     uint8_t ip0, uint8_t ip1, uint8_t ip2, uint8_t ip3,
                     uint16_t port = 2560) {
    // For TCP there is no I2CAddress, so we pass 0 to checkNoOverlap's address parameter.
    if (checkNoOverlap(vpin, nPins, 0)) new EXIOExpander_TCP(vpin, nPins, IPAddress(ip0, ip1, ip2, ip3), port);
  }

  VPIN getFirstVpin() const { return _firstVpin; }
  int  getVPINCount() const { return _nPins; }

  // Shift register support (mirrors I2C driver helpers)
  bool shiftInBytes(uint8_t clkPin, uint8_t latchPin, uint8_t dataPin,
                    uint8_t nBytes, uint8_t* out) {
    if (nBytes == 0 || nBytes > 16) return false;

    // Request payload: [clk][latch][data][nBytes]
    uint8_t payload[4] = { clkPin, latchPin, dataPin, nBytes };

    // Response payload: [EXIORDY][bytes...]
    uint8_t resp[1 + 16];
    if (!_rpc(EXIOSHIFTIN, payload, sizeof(payload), resp, (uint8_t)(1 + nBytes))) return false;
    if (resp[0] != EXIORDY) return false;
    memcpy(out, &resp[1], nBytes);
    return true;
  }

  bool shiftOutBytes(uint8_t clkPin, uint8_t latchPin, uint8_t dataPin,
                     uint8_t nBytes, const uint8_t* in) {
    if (nBytes == 0 || nBytes > 16) return false;

    // Request payload: [clk][latch][data][nBytes][payload...]
    uint8_t payload[4 + 16];
    payload[0] = clkPin;
    payload[1] = latchPin;
    payload[2] = dataPin;
    payload[3] = nBytes;
    memcpy(&payload[4], in, nBytes);

    // Response payload: [EXIORDY]
    uint8_t resp[1];
    if (!_rpc(EXIOSHIFTOUT, payload, (uint8_t)(4 + nBytes), resp, sizeof(resp))) return false;
    return resp[0] == EXIORDY;
  }

private:
  EXIOExpander_TCP* _nextExpander = nullptr;

  static EXIOExpander_TCP*& _expanderHead() {
    static EXIOExpander_TCP* head = nullptr;
    return head;
  }

  EXIOExpander_TCP(VPIN firstVpin, int nPins, IPAddress ip, uint16_t port) {
    _firstVpin = firstVpin;
    if (nPins > 256) nPins = 256;
    _nPins = nPins;
    _ip = ip;
    _port = port;

    // Link into expander-only list
    _nextExpander = _expanderHead();
    _expanderHead() = this;

    addDevice(this);
  }

  // ---- IODevice overrides ---------------------------------------------------

  void _begin() override {
    _deviceState = DEVSTATE_INITIALISING;
    _state = ST_DISCONNECTED;

    _lastConnAttemptMicros = 0;
    _readState = RDS_IDLE;

    // Attempt initial connection+handshake (blocking).
    if (!_connectAndHandshake(true)) {
      _deviceState = DEVSTATE_FAILED;
      return;
    }

    _deviceState = DEVSTATE_NORMAL;

#ifdef DIAG_IO
    _display();
#endif
  }

  // Digital input config
  bool _configure(VPIN vpin, ConfigTypeEnum configType, int paramCount, int params[]) override {
    if (_deviceState == DEVSTATE_FAILED) return false;
    if (paramCount != 1) return false;

    int pin = vpin - _firstVpin;

    if (configType == CONFIGURE_INPUT) {
      uint8_t pullup = params[0] ? 1 : 0;
      uint8_t payload[2] = { (uint8_t)pin, pullup };
      uint8_t resp[1];

      if (!_rpc(EXIODPUP, payload, sizeof(payload), resp, sizeof(resp))) return false;
      if (resp[0] == EXIORDY) return true;

      DIAG(F("EX-IOExpander TCP: Vpin %u cannot be used as a digital input pin"), (int)vpin);
      return false;
    }

    return false;
  }

  // Analogue input config
  int _configureAnalogIn(VPIN vpin) override {
    if (_deviceState == DEVSTATE_FAILED) return false;

    int pin = vpin - _firstVpin;
    uint8_t payload[1] = { (uint8_t)pin };
    uint8_t resp[1];

    if (!_rpc(EXIOENAN, payload, sizeof(payload), resp, sizeof(resp))) return false;
    if (resp[0] == EXIORDY) return true;

    DIAG(F("EX-IOExpander TCP: Vpin %u cannot be used as an analogue input pin"), (int)vpin);
    return false;
  }

  // Main loop: poll digital/analogue like the I2C driver (non-blocking state machine)
  void _loop(unsigned long currentMicros) override {
    if (_deviceState == DEVSTATE_FAILED) return;

    if (!_client || !_client->connected()) {
      _state = ST_DISCONNECTED;
      _readState = RDS_IDLE;

      if ((currentMicros - _lastConnAttemptMicros) > _reconnectIntervalMicros) {
        _lastConnAttemptMicros = currentMicros;
        (void)_connectAndHandshake(false);
      }
      return;
    }

    if (_readState != RDS_IDLE) {
      if (!_tryCompleteRead()) return;
      _readState = RDS_IDLE;
    }

    if (_readState == RDS_IDLE) {
      if (_numDigitalPins > 0 && (currentMicros - _lastDigitalReadMicros) > _digitalRefreshMicros) {
        if (_sendFrame(EXIORDD, nullptr, 0)) {
          _pendingStartMicros = micros();
          _pendingCmd = EXIORDD;
          _pendingLen = (uint8_t)((_numDigitalPins + 7) / 8);
          _pendingBuf = _digitalInputStates;
          _readDeadlineMicros = currentMicros + _responseTimeoutMicros;
          _readState = RDS_DIGITAL;
          _lastDigitalReadMicros = currentMicros;
        }
      } else if (_numAnaloguePins > 0 && (currentMicros - _lastAnalogueReadMicros) > _analogueRefreshMicros) {
        if (_sendFrame(EXIORDAN, nullptr, 0)) {
          _pendingStartMicros = micros();
          _pendingCmd = EXIORDAN;
          _pendingLen = (uint8_t)(_numAnaloguePins * 2);
          _pendingBuf = _analogueInputBuffer;
          _readDeadlineMicros = currentMicros + _responseTimeoutMicros;
          _readState = RDS_ANALOGUE;
          _lastAnalogueReadMicros = currentMicros;
        }
      }
    }
  }

  // Analogue read
  int _readAnalogue(VPIN vpin) override {
    if (_deviceState == DEVSTATE_FAILED) return 0;

    int pin = vpin - _firstVpin;
    for (uint8_t aPin = 0; aPin < _numAnaloguePins; aPin++) {
      if (_analoguePinMap[aPin] == pin) {
        uint8_t lsb = aPin * 2;
        uint8_t msb = lsb + 1;
        return (_analogueInputStates[msb] << 8) + _analogueInputStates[lsb];
      }
    }
    return -1;
  }

  // Digital read
  int _read(VPIN vpin) override {
    if (_deviceState == DEVSTATE_FAILED) return 0;
    int pin = vpin - _firstVpin;
    uint8_t pinByte = pin / 8;
    return bitRead(_digitalInputStates[pinByte], pin - pinByte * 8);
  }

  // Digital write
  void _write(VPIN vpin, int value) override {
    if (_deviceState == DEVSTATE_FAILED) return;

    int pin = vpin - _firstVpin;
    uint8_t payload[2] = { (uint8_t)pin, (uint8_t)value };
    uint8_t resp[1];

    if (!_rpc(EXIOWRD, payload, sizeof(payload), resp, sizeof(resp))) {
      _markOffline();
      return;
    }
    if (resp[0] != EXIORDY) {
      DIAG(F("EX-IOExpander TCP: Vpin %u cannot be used as a digital output pin"), (int)vpin);
    }
  }

  // Analogue write (PWM/Servo)
  void _writeAnalogue(VPIN vpin, int value, uint8_t profile, uint16_t duration) override {
    if (_deviceState == DEVSTATE_FAILED) return;

    int pin = vpin - _firstVpin;

#ifdef DIAG_IO
    DIAG(F("EXIO(TCP) WriteAnalogue Vpin:%u Value:%d Profile:%d Duration:%d"),
         vpin, value, profile, duration);
#endif

    uint8_t payload[6];
    payload[0] = (uint8_t)pin;
    payload[1] = (uint8_t)(value & 0xFF);
    payload[2] = (uint8_t)((value >> 8) & 0xFF);
    payload[3] = profile;
    payload[4] = (uint8_t)(duration & 0xFF);
    payload[5] = (uint8_t)((duration >> 8) & 0xFF);

    uint8_t resp[1];
    if (!_rpc(EXIOWRAN, payload, sizeof(payload), resp, sizeof(resp))) {
      _markOffline();
      return;
    }
    if (resp[0] != EXIORDY) {
      DIAG(F("EX-IOExpander TCP: Vpin %u cannot be used as a servo/PWM pin"), (int)vpin);
    }
  }

  void _display() override {
    DIAG(F("EX-IOExpander TCP:%d.%d.%d.%d:%d v%d.%d.%d Vpins %u-%u RTT %ums avg %ums %S"),
         _ip[0], _ip[1], _ip[2], _ip[3], _port,
         _majorVer, _minorVer, _patchVer,
         (int)_firstVpin, (int)_firstVpin + _nPins - 1,
         (unsigned)_rttLastMs, (unsigned)_rttAvgMs,
         _deviceState == DEVSTATE_FAILED ? F("OFFLINE") : F(""));
  }

  // ---- Protocol constants ---------------------------------------------------

  enum : uint8_t {
    EXIOINIT     = 0xE0,
    EXIORDY      = 0xE1,
    EXIODPUP     = 0xE2,
    EXIOVER      = 0xE3,
    EXIORDAN     = 0xE4,
    EXIOWRD      = 0xE5,
    EXIORDD      = 0xE6,
    EXIOENAN     = 0xE7,
    EXIOINITA    = 0xE8,
    EXIOPINS     = 0xE9,
    EXIOWRAN     = 0xEA,
    EXIOSHIFTIN  = 0xEB,
    EXIOSHIFTOUT = 0xEC,
    EXIOERR      = 0xEF,
  };

  // ---- Transport + handshake ------------------------------------------------

  enum { ST_DISCONNECTED, ST_CONNECTED } _state = ST_DISCONNECTED;
  enum { RDS_IDLE, RDS_DIGITAL, RDS_ANALOGUE } _readState = RDS_IDLE;

  bool _connectAndHandshake(bool blocking) {
    if (_client && _client->connected()) return true;

    if (!EXIO_TCPClientProvider::networkReady()) {
      if (blocking) DIAG(F("EX-IOExpander TCP: network not ready"));
      return false;
    }

    if (!_client) {
      _client = EXIO_TCPClientProvider::createClient();
      if (!_client) {
        if (blocking) {
          DIAG(F("EX-IOExpander TCP: no Client-based network stack in this build"));
          DIAG(F("  (ESP-AT/WifiInterface is AT-command Stream based, not Arduino Client)"));
        }
        return false;
      }
    }

    if (!_client->connect(_ip, _port)) {
      if (blocking) {
        DIAG(F("EX-IOExpander TCP:%d.%d.%d.%d:%d connect failed"),
             _ip[0], _ip[1], _ip[2], _ip[3], _port);
      }
      _markOffline();
      return false;
    }

    // Handshake: EXIOINIT -> expect EXIOPINS payload [EXIOPINS][dig][ana]
    uint8_t initPayload[3] = { (uint8_t)_nPins, (uint8_t)(_firstVpin & 0xFF), (uint8_t)(_firstVpin >> 8) };

    uint8_t pinsPayload[3];
    if (!_rpc(EXIOINIT, initPayload, sizeof(initPayload), pinsPayload, sizeof(pinsPayload))) {
      DIAG(F("EX-IOExpander TCP handshake failed (EXIOINIT)"));
      _markOffline();
      return false;
    }
    if (pinsPayload[0] != EXIOPINS) {
      DIAG(F("EX-IOExpander TCP handshake expected EXIOPINS, got 0x%x"), pinsPayload[0]);
      _markOffline();
      return false;
    }

    _numDigitalPins = pinsPayload[1];
    _numAnaloguePins = pinsPayload[2];

    if (!_allocBuffers()) {
      _markOffline();
      return false;
    }

    if (_numAnaloguePins > 0) {
      if (!_sendFrame(EXIOINITA, nullptr, 0)) { _markOffline(); return false; }

      uint8_t hdr[2];
      if (!_readHeaderBlocking(hdr, blocking)) { _markOffline(); return false; }

      uint8_t rcmd = hdr[0];
      uint8_t rlen = hdr[1];
      if (rcmd != EXIOINITA || rlen != _numAnaloguePins) { _drain(rlen); _markOffline(); return false; }

      if (!_readBytesBlocking(_analoguePinMap, _numAnaloguePins, blocking)) { _markOffline(); return false; }
    }

    uint8_t ver[3];
    if (_rpc(EXIOVER, nullptr, 0, ver, sizeof(ver))) {
      _majorVer = ver[0];
      _minorVer = ver[1];
      _patchVer = ver[2];
    }

    _state = ST_CONNECTED;
    return true;
  }

  void _markOffline() {
    if (_client) {
      _client->stop();
      delete _client;
      _client = nullptr;
    }
    _state = ST_DISCONNECTED;
    _readState = RDS_IDLE;
  }

  bool _allocBuffers() {
    if (_numDigitalPins > 0) {
      size_t need = (_numDigitalPins + 7) / 8;
      if (_digitalPinBytes < need) {
        if (_digitalPinBytes > 0) free(_digitalInputStates);
        _digitalInputStates = (uint8_t*)calloc(need, 1);
        if (!_digitalInputStates) {
          DIAG(F("EX-IOExpander TCP ERROR alloc digital %d bytes"), (int)need);
          _digitalPinBytes = 0;
          return false;
        }
        _digitalPinBytes = (uint8_t)need;
      }
    }

    if (_numAnaloguePins > 0) {
      size_t need = _numAnaloguePins * 2;
      if (_analoguePinBytes < need) {
        if (_analoguePinBytes > 0) {
          free(_analogueInputBuffer);
          free(_analogueInputStates);
          free(_analoguePinMap);
        }

        _analogueInputStates = (uint8_t*)calloc(need, 1);
        _analogueInputBuffer = (uint8_t*)calloc(need, 1);
        _analoguePinMap       = (uint8_t*)calloc(_numAnaloguePins, 1);

        if (!_analogueInputStates || !_analogueInputBuffer || !_analoguePinMap) {
          DIAG(F("EX-IOExpander TCP ERROR alloc analog buffers"));
          _analoguePinBytes = 0;
          return false;
        }

        _analoguePinBytes = (uint8_t)need;
      }
    }

    return true;
  }

  // Frame send: [CMD][LEN][PAYLOAD...]
  bool _sendFrame(uint8_t cmd, const uint8_t* payload, uint8_t len) {
    if (!_client || !_client->connected()) return false;
    uint8_t hdr[2] = { cmd, len };
    if (_client->write(hdr, 2) != 2) return false;
    if (len && payload) {
      if (_client->write(payload, len) != len) return false;
    }
    return true;
  }

  // Blocking RPC:
  bool _rpc(uint8_t cmd,
            const uint8_t* payload, uint8_t payloadLen,
            uint8_t* resp, uint8_t respLen) {
    if (!_connectAndHandshake(true)) return false;

    unsigned long t0 = micros();

    if (!_sendFrame(cmd, payload, payloadLen)) return false;

    uint8_t hdr[2];
    if (!_readHeaderBlocking(hdr, true)) return false;

    uint8_t rcmd = hdr[0];
    uint8_t rlen = hdr[1];

    if (rlen != respLen) {
      _drain(rlen);
      return false;
    }

    if (respLen && resp) {
      if (!_readBytesBlocking(resp, respLen, true)) return false;
    } else {
      _drain(rlen);
    }

    if (rcmd == EXIOERR) return false;

    _recordRttMicros(t0);
    (void)rcmd;
    return true;
  }

  bool _readHeaderBlocking(uint8_t hdr[2], bool blocking) {
    unsigned long start = millis();
    while (_client && _client->connected() && _client->available() < 2) {
      if (!blocking) return false;
      if ((millis() - start) > _responseTimeoutMs) return false;
      delay(1);
    }
    if (!_client || !_client->connected()) return false;
    hdr[0] = (uint8_t)_client->read();
    hdr[1] = (uint8_t)_client->read();
    return true;
  }

  bool _readBytesBlocking(uint8_t* out, uint8_t len, bool blocking) {
    unsigned long start = millis();
    while (_client && _client->connected() && _client->available() < len) {
      if (!blocking) return false;
      if ((millis() - start) > _responseTimeoutMs) return false;
      delay(1);
    }
    if (!_client || !_client->connected()) return false;
    for (uint8_t i = 0; i < len; i++) out[i] = (uint8_t)_client->read();
    return true;
  }

  void _drain(uint8_t len) {
    while (len--) {
      if (!_client || !_client->connected()) return;
      while (_client->connected() && _client->available() == 0) delay(1);
      if (_client->available()) (void)_client->read();
    }
  }

  bool _tryCompleteRead() {
    unsigned long nowMicros = micros();
    if ((long)(nowMicros - _readDeadlineMicros) > 0) {
      _markOffline();
      return true;
    }

    const uint16_t need = (uint16_t)(2 + _pendingLen);
    if (!_client || _client->available() < (int)need) return false;

    uint8_t rcmd = (uint8_t)_client->read();
    uint8_t rlen = (uint8_t)_client->read();

    if (rcmd != _pendingCmd || rlen != _pendingLen) {
      _drain(rlen);
      return true;
    }

    if (_pendingBuf && rlen) {
      for (uint8_t i = 0; i < rlen; i++) _pendingBuf[i] = (uint8_t)_client->read();
    } else {
      _drain(rlen);
    }

    if (_readState == RDS_ANALOGUE) {
      memcpy(_analogueInputStates, _analogueInputBuffer, _analoguePinBytes);
    }

    _recordRttMicros(_pendingStartMicros);
    return true;
  }

  // ---- Latency (RTT) tracking ----------------------------------------------

  void _recordRttMicros(unsigned long startMicros) {
    unsigned long dt = micros() - startMicros;
    uint16_t ms = (uint16_t)(dt / 1000UL);
    _rttLastMs = ms;

    if (_rttSamples == 0) _rttAvgMs = ms;
    else _rttAvgMs = (uint16_t)(((_rttAvgMs * 7UL) + ms) / 8UL);

    _rttSamples++;
  }

  // ---- Members --------------------------------------------------------------

  IPAddress _ip;
  uint16_t _port = 2560;

  Client* _client = nullptr;

  uint8_t _numDigitalPins = 0;
  uint8_t _numAnaloguePins = 0;

  uint8_t _majorVer = 0;
  uint8_t _minorVer = 0;
  uint8_t _patchVer = 0;

  uint8_t* _digitalInputStates  = nullptr;
  uint8_t* _analogueInputStates = nullptr;
  uint8_t* _analogueInputBuffer = nullptr;
  uint8_t* _analoguePinMap      = nullptr;

  uint8_t _digitalPinBytes  = 0;
  uint8_t _analoguePinBytes = 0;

  // Poll timing
  unsigned long _lastDigitalReadMicros = 0;
  unsigned long _lastAnalogueReadMicros = 0;
  const unsigned long _digitalRefreshMicros  = 10000UL;   // 10ms
  const unsigned long _analogueRefreshMicros = 50000UL;   // 50ms

  // Reconnect / timeouts
  unsigned long _lastConnAttemptMicros = 0;
  const unsigned long _reconnectIntervalMicros = 1000000UL; // 1s
  unsigned long _readDeadlineMicros = 0;
  const unsigned long _responseTimeoutMicros = 200000UL;    // 200ms
  const unsigned long _responseTimeoutMs = 200;

  // Pending periodic read
  uint8_t _pendingCmd = 0;
  uint8_t _pendingLen = 0;
  uint8_t* _pendingBuf = nullptr;
  unsigned long _pendingStartMicros = 0;

  // RTT stats
  uint16_t _rttLastMs = 0;
  uint16_t _rttAvgMs  = 0;
  uint32_t _rttSamples = 0;
};

#endif
