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
 */

#include "IODevice.h"
#include "DIAG.h"
#include "FSH.h"

#include <Client.h>
#include "EXIO_TCPClientProvider.h"

#define DIAG_IO

/////////////////////////////////////////////////////////////////////////////////////////////////////
class EXIOExpander_TCP : public IODevice {
public:
  enum ProfileType : uint8_t {
    Instant     = 0,
    UseDuration = 0,
    Fast        = 1,
    Medium      = 2,
    Slow        = 3,
    Bounce      = 4,
    NoPowerOff  = 0x80,
  };

  static EXIOExpander_TCP* findByVpin(VPIN vpin) {
    for (EXIOExpander_TCP* ex = _expanderHead(); ex; ex = ex->_nextExpander) {
      if (vpin >= ex->_firstVpin && vpin < (VPIN)(ex->_firstVpin + ex->_nPins)) return ex;
    }
    return nullptr;
  }

  static void create(VPIN vpin, int nPins,
                     uint8_t ip0, uint8_t ip1, uint8_t ip2, uint8_t ip3,
                     uint16_t port = 2560) {
    if (checkNoOverlap(vpin, nPins, 0)) new EXIOExpander_TCP(vpin, nPins, IPAddress(ip0, ip1, ip2, ip3), port);
  }

  VPIN getFirstVpin() const { return _firstVpin; }
  int  getVPINCount() const { return _nPins; }

  bool shiftInBytes(uint8_t clkPin, uint8_t latchPin, uint8_t dataPin,
                    uint8_t nBytes, uint8_t* out) {
    if (nBytes == 0 || nBytes > 16) return false;

    uint8_t payload[4] = { clkPin, latchPin, dataPin, nBytes };
    uint8_t resp[1 + 16];

    if (!_rpc(EXIOSHIFTIN, payload, sizeof(payload), resp, (uint8_t)(1 + nBytes))) return false;
    if (resp[0] != EXIORDY) return false;

    memcpy(out, &resp[1], nBytes);
    return true;
  }

  bool shiftOutBytes(uint8_t clkPin, uint8_t latchPin, uint8_t dataPin,
                     uint8_t nBytes, const uint8_t* in) {
    if (nBytes == 0 || nBytes > 16) return false;

    uint8_t payload[4 + 16];
    payload[0] = clkPin;
    payload[1] = latchPin;
    payload[2] = dataPin;
    payload[3] = nBytes;
    memcpy(&payload[4], in, nBytes);

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

    Client* probe = EXIO_TCPClientProvider::createClient();
    if (!probe) {
      _deviceState = DEVSTATE_FAILED;
      DIAG(F("EX-IOExpander TCP: no Client-based network stack in this build"));
      return;
    }
    delete probe;

    if (_connectAndHandshake(true)) {
      _deviceState = DEVSTATE_NORMAL;
    } else {
      _deviceState = DEVSTATE_NORMAL; // allow retries in _loop()
      _state = ST_DISCONNECTED;
    }

#ifdef DIAG_IO
    _display();
#endif
  }

  bool _configure(VPIN vpin, ConfigTypeEnum configType, int paramCount, int params[]) override {
    if (_deviceState == DEVSTATE_FAILED) return false;
    if (paramCount != 1) return false;

    int pin = (int)(vpin - _firstVpin);

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

  int _configureAnalogIn(VPIN vpin) override {
    if (_deviceState == DEVSTATE_FAILED) return false;

    int pin = (int)(vpin - _firstVpin);
    uint8_t payload[1] = { (uint8_t)pin };
    uint8_t resp[1];

    if (!_rpc(EXIOENAN, payload, sizeof(payload), resp, sizeof(resp))) return false;
    if (resp[0] == EXIORDY) return true;

    DIAG(F("EX-IOExpander TCP: Vpin %u cannot be used as an analogue input pin"), (int)vpin);
    return false;
  }

  void _loop(unsigned long currentMicros) override {
    if (_deviceState == DEVSTATE_FAILED) return;

    if (!_client || !_client->connected() || _state != ST_CONNECTED) {
      _state = ST_DISCONNECTED;
      _readState = RDS_IDLE;

      if ((currentMicros - _lastConnAttemptMicros) > _reconnectIntervalMicros) {
        _lastConnAttemptMicros = currentMicros;

        // Use a short reconnect budget (150ms) rather than “instant fail”.
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

  int _readAnalogue(VPIN vpin) override {
    if (_deviceState == DEVSTATE_FAILED) return 0;

    int pin = (int)(vpin - _firstVpin);
    for (uint8_t aPin = 0; aPin < _numAnaloguePins; aPin++) {
      if (_analoguePinMap && _analoguePinMap[aPin] == pin) {
        uint8_t lsb = aPin * 2;
        uint8_t msb = lsb + 1;
        return (_analogueInputStates[msb] << 8) + _analogueInputStates[lsb];
      }
    }
    return -1;
  }

  int _read(VPIN vpin) override {
    if (_deviceState == DEVSTATE_FAILED) return 0;
    int pin = (int)(vpin - _firstVpin);
    uint8_t pinByte = (uint8_t)(pin / 8);
    return bitRead(_digitalInputStates[pinByte], pin - (int)pinByte * 8);
  }

  void _write(VPIN vpin, int value) override {
    if (_deviceState == DEVSTATE_FAILED) return;

    int pin = (int)(vpin - _firstVpin);
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

  void _writeAnalogue(VPIN vpin, int value, uint8_t profile, uint16_t duration) override {
    if (_deviceState == DEVSTATE_FAILED) return;

    int pin = (int)(vpin - _firstVpin);

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
    if (_client && _client->connected() && _state == ST_CONNECTED) return true;

    // Non-blocking reconnect still needs to cover real RTT + jitter.
    // Use a floor of 300ms, otherwise scale from observed RTT average.
    unsigned long budgetMs = _responseTimeoutMs;
    if (!blocking) {
      const unsigned long rtt = (unsigned long)_rttAvgMs;              // ms
      const unsigned long adaptive = (rtt ? (rtt * 3UL + 100UL) : 400UL); // ~3x RTT + 100ms
      budgetMs = adaptive;
      if (budgetMs < 300UL) budgetMs = 300UL;
      if (budgetMs > _responseTimeoutMs) budgetMs = _responseTimeoutMs;
    }


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

    // Give the IOExpander a moment to finish booting its TCP task,
    // and clear any stale bytes from a previous session.
    delay(20);
    while (_client->available() > 0) (void)_client->read();

    // Handshake: EXIOINIT -> expect payload [EXIOPINS][dig][ana]
    uint8_t initPayload[3] = {
      (uint8_t)_nPins,
      (uint8_t)(_firstVpin & 0xFF),
      (uint8_t)(_firstVpin >> 8)
    };

    uint8_t pinsPayload[3];
    if (!_rpcNoConnect(EXIOINIT, initPayload, sizeof(initPayload),
                       pinsPayload, sizeof(pinsPayload), budgetMs)) {
      if (blocking) DIAG(F("EX-IOExpander TCP handshake failed (EXIOINIT)"));
      _markOffline();
      return false;
    }

    if (pinsPayload[0] != EXIOPINS) {
      if (blocking) DIAG(F("EX-IOExpander TCP handshake expected EXIOPINS, got 0x%x"), pinsPayload[0]);
      _markOffline();
      return false;
    }

    _numDigitalPins  = pinsPayload[1];
    _numAnaloguePins = pinsPayload[2];

    if (!_allocBuffers()) {
      _markOffline();
      return false;
    }

    if (_numAnaloguePins > 0) {
      if (!_sendFrame(EXIOINITA, nullptr, 0)) { _markOffline(); return false; }

      uint8_t hdr[2];
      if (!_readHeaderTimeout(hdr, budgetMs)) { _markOffline(); return false; }

      const uint8_t rcmd = hdr[0];
      const uint8_t rlen = hdr[1];

      if (rcmd != EXIOINITA || rlen != _numAnaloguePins) {
#ifdef DIAG_IO
        DIAG(F("EX-IOExpander TCP handshake FAIL at INITA: got rcmd=0x%02X rlen=%u expected rcmd=0x%02X rlen=%u"),
             rcmd, rlen, EXIOINITA, _numAnaloguePins);
#endif
        _drain(rlen);
        _markOffline();
        return false;
      }

      if (!_readBytesTimeout(_analoguePinMap, _numAnaloguePins, budgetMs)) { _markOffline(); return false; }
    }

    // best-effort version fetch
    uint8_t ver[3];
    if (_rpcNoConnect(EXIOVER, nullptr, 0, ver, sizeof(ver), budgetMs)) {
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

  // ---- Buffers --------------------------------------------------------------

  bool _allocBuffers() {
    if (_numDigitalPins > 0) {
      size_t need = (_numDigitalPins + 7) / 8;
      if (_digitalPinBytes < need) {
        if (_digitalPinBytes > 0 && _digitalInputStates) free(_digitalInputStates);
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
          if (_analogueInputBuffer) free(_analogueInputBuffer);
          if (_analogueInputStates) free(_analogueInputStates);
          if (_analoguePinMap) free(_analoguePinMap);
        }

        _analogueInputStates  = (uint8_t*)calloc(need, 1);
        _analogueInputBuffer  = (uint8_t*)calloc(need, 1);
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

  // ---- Framing --------------------------------------------------------------

  bool _sendFrame(uint8_t cmd, const uint8_t* payload, uint8_t len) {
    if (!_client || !_client->connected()) return false;
    uint8_t hdr[2] = { cmd, len };
    if (_client->write(hdr, 2) != 2) return false;
    if (len && payload) {
      if (_client->write(payload, len) != len) return false;
    }
    return true;
  }

  bool _finishPendingPoll(unsigned long budgetMs) {
    if (_readState == RDS_IDLE) return true;

    unsigned long start = millis();
    while (_readState != RDS_IDLE) {
      if (_tryCompleteRead()) {
        _readState = RDS_IDLE;
        return true;
      }

      if ((millis() - start) > budgetMs) {
        // We have a poll in-flight but it never completed; safest is to reset the link.
        _markOffline();
        return false;
      }

      delay(1);
    }

    return true;
  }


  bool _rpc(uint8_t cmd,
            const uint8_t* payload, uint8_t payloadLen,
            uint8_t* resp, uint8_t respLen) {

    // Wait long enough for the in-flight poll to finish (WiFi RTT is ~200ms+)
    unsigned long pollBudgetMs = 200UL;
    if (_rttAvgMs) pollBudgetMs = (_rttAvgMs * 3UL) + 100UL;   // ~3x RTT + 100ms
    if (pollBudgetMs < 200UL) pollBudgetMs = 200UL;
    if (pollBudgetMs > _responseTimeoutMs) pollBudgetMs = _responseTimeoutMs;

    if (!_finishPendingPoll(pollBudgetMs)) return false;

    if (!_client || !_client->connected() || _state != ST_CONNECTED) {
      if (!_connectAndHandshake(true)) return false;
    }

    return _rpcNoConnect(cmd, payload, payloadLen, resp, respLen, _responseTimeoutMs);
  }

  bool _rpcNoConnect(uint8_t cmd,
                     const uint8_t* payload, uint8_t payloadLen,
                     uint8_t* resp, uint8_t respLen,
                     unsigned long budgetMs) {
    if (!_client || !_client->connected()) return false;

#ifdef DIAG_IO
    DIAG(F("EXIO(TCP) RPC tx cmd=0x%x len=%u budgetMs=%lu"), cmd, payloadLen, budgetMs);
#endif

    const unsigned long t0 = micros();

    if (!_sendFrame(cmd, payload, payloadLen)) return false;

    uint8_t hdr[2];
    if (!_readHeaderTimeout(hdr, budgetMs)) return false;

    const uint8_t rcmd = hdr[0];
    const uint8_t rlen = hdr[1];

#ifdef DIAG_IO
    DIAG(F("EXIO(TCP) RPC rx rcmd=0x%x rlen=%u expect=%u"), rcmd, rlen, respLen);
#endif

    if (rcmd == EXIOERR) {
      _drain(rlen);
      return false;
    }

    if (rlen != respLen) {
      _drain(rlen);
      return false;
    }

    if (respLen && resp) {
      if (!_readBytesTimeout(resp, respLen, budgetMs)) return false;
#ifdef DIAG_IO
      DIAG(F("EXIO(TCP) RPC payload: %02X %02X %02X"),
           resp[0], respLen > 1 ? resp[1] : 0, respLen > 2 ? resp[2] : 0);
#endif
    } else if (rlen) {
      _drain(rlen);
    }

    _recordRttMicros(t0);
    return true;
  }

  bool _readHeaderTimeout(uint8_t hdr[2], unsigned long timeoutMs) {
    unsigned long start = millis();
    while (_client && _client->connected() && _client->available() < 2) {
      if ((millis() - start) > timeoutMs) return false;
      delay(1);
    }
    if (!_client || !_client->connected()) return false;
    hdr[0] = (uint8_t)_client->read();
    hdr[1] = (uint8_t)_client->read();
    return true;
  }

  bool _readBytesTimeout(uint8_t* out, uint8_t len, unsigned long timeoutMs) {
    unsigned long start = millis();
    while (_client && _client->connected() && _client->available() < len) {
      if ((millis() - start) > timeoutMs) return false;
      delay(1);
    }
    if (!_client || !_client->connected()) return false;
    for (uint8_t i = 0; i < len; i++) out[i] = (uint8_t)_client->read();
    return true;
  }

  void _drain(uint8_t len) {
    while (len--) {
      if (!_client || !_client->connected()) return;
      unsigned long start = millis();
      while (_client->connected() && _client->available() == 0) {
        if ((millis() - start) > _responseTimeoutMs) return;
        delay(1);
      }
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

  uint8_t _numDigitalPins   = 0;
  uint8_t _numAnaloguePins  = 0;

  uint8_t _majorVer = 0;
  uint8_t _minorVer = 0;
  uint8_t _patchVer = 0;

  uint8_t* _digitalInputStates  = nullptr;
  uint8_t* _analogueInputStates = nullptr;
  uint8_t* _analogueInputBuffer = nullptr;
  uint8_t* _analoguePinMap      = nullptr;

  uint8_t _digitalPinBytes  = 0;
  uint8_t _analoguePinBytes = 0;

  unsigned long _lastDigitalReadMicros  = 0;
  unsigned long _lastAnalogueReadMicros = 0;
  const unsigned long _digitalRefreshMicros  = 250000UL;
  const unsigned long _analogueRefreshMicros = 500000UL;

  unsigned long _lastConnAttemptMicros = 0;
  const unsigned long _reconnectIntervalMicros = 1000000UL;
  unsigned long _readDeadlineMicros = 0;
  const unsigned long _responseTimeoutMicros = 1000000UL;
  const unsigned long _responseTimeoutMs = 1000UL;

  uint8_t _pendingCmd = 0;
  uint8_t _pendingLen = 0;
  uint8_t* _pendingBuf = nullptr;
  unsigned long _pendingStartMicros = 0;

  uint16_t _rttLastMs  = 0;
  uint16_t _rttAvgMs   = 0;
  uint32_t _rttSamples = 0;
};

#endif
