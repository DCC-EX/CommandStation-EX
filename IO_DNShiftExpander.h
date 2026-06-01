#pragma once
#include "IODevice.h"
#include "IO_EXIOExpander.h"
#include "DIAG.h"

class IO_DNShiftExpander : public IODevice {
public:
  IO_DNShiftExpander(VPIN firstVpin, int nPins,
                     VPIN clk, VPIN latch, VPIN data,
                     bool input,
                     const uint8_t* pinMap = nullptr)
    : IODevice(firstVpin, nPins),
      _clk(clk), _latch(latch), _data(data),
      _input(input),
      _pinMap(pinMap)
  {
    _nShiftBytes = (nPins + 7) / 8;
    _pinValues = (uint8_t*)calloc(_nShiftBytes, 1);

    if (!_pinValues) {
      DIAG(F("DNShiftExpander: alloc failed (%d bytes)"), _nShiftBytes);
      _deviceState = DEVSTATE_FAILED;
      return;
    }

    EXIOExpander* exClk   = EXIOExpander::findByVpin(_clk);
    EXIOExpander* exLatch = EXIOExpander::findByVpin(_latch);
    EXIOExpander* exData  = EXIOExpander::findByVpin(_data);

    if (!exClk || exClk != exLatch || exClk != exData) {
      DIAG(F("DNShiftExpander: clk/latch/data not on same EXIOExpander"));
      _deviceState = DEVSTATE_FAILED;
      return;
    }
    _expander = exClk;

    // Extra sanity check (optional, but fine)
    VPIN first = _expander->getFirstVpin();
    VPIN last  = (VPIN)(first + _expander->getVPINCount());
    if (_clk   < first || _clk   >= last ||
        _latch < first || _latch >= last ||
        _data  < first || _data  >= last) {
      DIAG(F("DNShiftExpander: pins not within EXIOExpander vpin range"));
      _deviceState = DEVSTATE_FAILED;
      return;
    }

    _deviceState = DEVSTATE_NORMAL;
    IODevice::addDevice(this);
  }

  void _begin() override {
    // leave failed state as failed
    if (_deviceState != DEVSTATE_FAILED) _deviceState = DEVSTATE_NORMAL;
  }

  void _display() override {
    DIAG(F("DN%S node via EXIO Vpins:%u-%u bytes=%d"),
        _input ? F("IN") : F("OU"),
        (int)_firstVpin,
        (int)_firstVpin + _nPins - 1,
        (int)_nShiftBytes);
  }

  void _loop(unsigned long now) override {
    if (_deviceState == DEVSTATE_FAILED) return;

    if (_input) {
      if (now - _lastPoll < POLL_US) return;
      _lastPoll = now;

      if (_nShiftBytes > 16) return; // safety (matches expander firmware limit)

      uint8_t tmp[16];
      if (_expander->shiftInBytes(pin(_clk), pin(_latch), pin(_data), _nShiftBytes, tmp)) {
        //DIAG(F("DNIN8V read: %02X %02X %02X %02X"), tmp[0], tmp[1], tmp[2], tmp[3]);
        for (uint8_t i = 0; i < _nShiftBytes; i++) {
          _pinValues[i] = remapInByte(tmp[i]);
        }
      }
    } else {
      if (!_xmitPending) return;
      _xmitPending = false;
      _expander->shiftOutBytes(pin(_clk), pin(_latch), pin(_data), _nShiftBytes, _pinValues);
    }
  }

  int _read(VPIN vpin) override {
    int bit = (int)(vpin - _firstVpin);
    int byteIndex = bit / 8;
    int bitIndex  = bit % 8;

    uint8_t mask = (uint8_t)(0x80 >> bitIndex);
    return (_pinValues[byteIndex] & mask) ? 1 : 0;
  }

  void _write(VPIN vpin, int value) override {
    int bit = (int)(vpin - _firstVpin);
    int byteIndex = bit / 8;
    int bitIndex  = bit % 8;

    uint8_t mask = (uint8_t)(0x80 >> bitIndex);
    uint8_t oldv = _pinValues[byteIndex];

    if (value) _pinValues[byteIndex] |= mask;
    else       _pinValues[byteIndex] &= (uint8_t)~mask;

    if (_pinValues[byteIndex] != oldv) _xmitPending = true;
  }

private:
  static const unsigned long POLL_US = 100000;

  EXIOExpander* _expander = nullptr;
  VPIN _clk, _latch, _data;
  bool _input;

  const uint8_t* _pinMap = nullptr;

  uint8_t* _pinValues = nullptr;
  uint8_t _nShiftBytes = 0;

  bool _xmitPending = false;
  unsigned long _lastPoll = 0;

  uint8_t pin(VPIN v) const { return (uint8_t)(v - _expander->getFirstVpin()); }

  uint8_t remapInByte(uint8_t raw) const {
    if (!_pinMap) return raw;

    uint8_t newByte = 0;
    // raw bit 7 is first shifted-in bit (xmitBit 0), matches your earlier assumption
    for (int xmitBit = 0; xmitBit < 8; xmitBit++) {
      bool data = (raw & (1 << (7 - xmitBit))) != 0;
      uint8_t map = _pinMap[xmitBit];
      if (data) newByte |= map;
      else      newByte &= (uint8_t)~map;
    }
    return newByte;
  }
};
