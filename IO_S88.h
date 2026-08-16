/*
 *  S88 and S88-N feedback bus HAL driver.
 *
 *  This file is part of CommandStation-EX
 *
 *  This is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This driver deliberately uses the cooperative IODevice scheduler.  S88
 *  devices are allowed to require relatively long clock high and low times;
 *  doing one edge per scheduler entry avoids blocking the DCC and command
 *  station loops while the bus is being shifted.
 */

#ifndef IO_S88_H
#define IO_S88_H

#include <Arduino.h>
#include "DIAG.h"
#include "IODevice.h"

class S88 : public IODevice {
public:
  // The IODevice overlap API uses an 8-bit pin count.  Keep the same limit
  // here so a wrapped count can never silently allocate the wrong VPIN range.
  static const uint16_t MAX_PINS = 255;
  static const uint16_t MAX_STATE_BYTES = 32;
  static const uint32_t MIN_CLOCK_PERIOD_MICROS = 30;
  static const uint32_t DEFAULT_CLOCK_PERIOD_MICROS = 200;
  static const uint32_t DEFAULT_SCAN_INTERVAL_MICROS = 10000;

  static constexpr uint16_t stateBytes(uint16_t nPins) {
    return (nPins + 7) / 8;
  }

  static_assert((MAX_PINS + 7) / 8 <= MAX_STATE_BYTES,
    "S88 state storage limit must cover the supported VPIN range");

  /*
   * Create one S88 chain.
   *
   * The first bit shifted out by the chain is exposed at firstVpin.  A
   * complete S88 frame is shifted before scanIntervalMicros is added as an
   * idle gap.  clockPeriodMicros is the complete clock period, so each high
   * and low phase is approximately half that value.
   */
  static void create(VPIN firstVpin, uint16_t nPins,
                     uint8_t clockPin, uint8_t loadPin,
                     uint8_t resetPin, uint8_t dataPin,
                     uint32_t clockPeriodMicros = DEFAULT_CLOCK_PERIOD_MICROS,
                     uint32_t scanIntervalMicros = DEFAULT_SCAN_INTERVAL_MICROS,
                     bool invertData = false,
                     bool dataPullup = false) {
#if defined(IO_NO_HAL)
    // The small IO_NO_HAL build has no device list to attach to.  Keep the
    // API harmless if an automation file is shared with a full HAL build.
    (void)firstVpin;
    (void)nPins;
    (void)clockPin;
    (void)loadPin;
    (void)resetPin;
    (void)dataPin;
    (void)clockPeriodMicros;
    (void)scanIntervalMicros;
    (void)invertData;
    (void)dataPullup;
    return;
#else
    if (!validParameters(firstVpin, nPins, clockPin, loadPin, resetPin,
                         dataPin, clockPeriodMicros)) {
      DIAG(F("S88: invalid configuration"));
      return;
    }
    if (!IODevice::checkNoOverlap(firstVpin, (uint8_t)nPins))
      return;

    S88 *device = new S88(firstVpin, nPins, clockPin, loadPin, resetPin,
                          dataPin, clockPeriodMicros, scanIntervalMicros,
                          invertData, dataPullup);
    if (!device->_ready)
      delete device;
#endif
  }

private:
  enum Step : uint8_t {
    STEP_START,
    STEP_LOAD_CLOCK_HIGH,
    STEP_LOAD_CLOCK_LOW,
    STEP_RELEASE_LOAD,
    STEP_SAMPLE_FIRST,
    STEP_RELEASE_RESET,
    STEP_CLOCK_HIGH,
    STEP_CLOCK_LOW,
    STEP_SAMPLE_CLOCKED,
    STEP_FINISH,
    STEP_PUBLISH
  };

  S88(VPIN firstVpin, uint16_t nPins,
      uint8_t clockPin, uint8_t loadPin,
      uint8_t resetPin, uint8_t dataPin,
      uint32_t clockPeriodMicros, uint32_t scanIntervalMicros,
      bool invertData, bool dataPullup) :
    IODevice(firstVpin, (int)nPins),
    _clockPin(clockPin),
    _loadPin(loadPin),
    _resetPin(resetPin),
    _dataPin(dataPin),
    _phaseMicros(clockPeriodMicros / 2),
    _scanIntervalMicros(scanIntervalMicros),
    _invertData(invertData),
    _dataPullup(dataPullup),
    _stateByteCount(stateBytes(nPins)) {
    // Keep both the current frame and the last published frame packed.  The
    // allocation is bounded by MAX_STATE_BYTES and is checked before the
    // device is attached to the HAL list.
    _states = (uint8_t *)calloc((size_t)_stateByteCount * 2, 1);
    if (!_states) {
      DIAG(F("S88: unable to allocate %u bytes"),
        (unsigned)(_stateByteCount * 2));
      return;
    }

    _publishedStates = _states + _stateByteCount;
    _hasCallback = true;
    _ready = true;
    IODevice::addDevice(this);
  }

  static bool validParameters(VPIN firstVpin, uint16_t nPins,
                              uint8_t clockPin, uint8_t loadPin,
                              uint8_t resetPin, uint8_t dataPin,
                              uint32_t clockPeriodMicros) {
    if (nPins == 0 || nPins > MAX_PINS)
      return false;
    if (firstVpin > VPIN_MAX ||
        nPins > (uint16_t)(VPIN_MAX - firstVpin + 1))
      return false;
    if (clockPeriodMicros < MIN_CLOCK_PERIOD_MICROS ||
        (clockPeriodMicros & 1))
      return false;
    if (clockPin == loadPin || clockPin == resetPin ||
        clockPin == dataPin || loadPin == resetPin ||
        loadPin == dataPin || resetPin == dataPin)
      return false;
#if defined(NUM_DIGITAL_PINS)
    if (clockPin >= NUM_DIGITAL_PINS || loadPin >= NUM_DIGITAL_PINS ||
        resetPin >= NUM_DIGITAL_PINS || dataPin >= NUM_DIGITAL_PINS)
      return false;
#endif
    return true;
  }

  void _begin() override {
    if (!_ready)
      return;

    pinMode(_clockPin, OUTPUT);
    pinMode(_loadPin, OUTPUT);
    pinMode(_resetPin, OUTPUT);
    pinMode(_dataPin, _dataPullup ? INPUT_PULLUP : INPUT);

    // S88 control signals are active high and idle low.
    ArduinoPins::fastWriteDigital(_clockPin, LOW);
    ArduinoPins::fastWriteDigital(_loadPin, LOW);
    ArduinoPins::fastWriteDigital(_resetPin, LOW);

    _deviceState = DEVSTATE_NORMAL;
    _step = STEP_START;
    _bitIndex = 0;
    _publishIndex = 0;
    _display();
  }

  bool _configure(VPIN vpin, ConfigTypeEnum configType,
                  int paramCount, int params[]) override {
    if (configType != CONFIGURE_INPUT || paramCount != 1 ||
        !owns(vpin))
      return false;

    // Sensor pull-up configuration is meaningful only for the physical data
    // line.  Accept it for compatibility with Sensor::create(), but keep the
    // line configuration in one place.
    _dataPullup = params[0] != 0;
    pinMode(_dataPin, _dataPullup ? INPUT_PULLUP : INPUT);
    return true;
  }

  int _read(VPIN vpin) override {
    int index = (int)vpin - (int)_firstVpin;
    if (index < 0 || index >= _nPins)
      return 0;
    return (_states[index >> 3] & (uint8_t)(1 << (index & 7))) ? 1 : 0;
  }

  void _loop(unsigned long currentMicros) override {
    switch (_step) {
    case STEP_START:
      ArduinoPins::fastWriteDigital(_loadPin, HIGH);
      _step = STEP_LOAD_CLOCK_HIGH;
      schedule(currentMicros);
      break;

    case STEP_LOAD_CLOCK_HIGH:
      // A clock edge while LOAD is asserted transfers the parallel input
      // state into the shift registers used by the chain.
      ArduinoPins::fastWriteDigital(_clockPin, HIGH);
      _step = STEP_LOAD_CLOCK_LOW;
      schedule(currentMicros);
      break;

    case STEP_LOAD_CLOCK_LOW:
      ArduinoPins::fastWriteDigital(_clockPin, LOW);
      _step = STEP_RELEASE_LOAD;
      schedule(currentMicros);
      break;

    case STEP_RELEASE_LOAD:
      ArduinoPins::fastWriteDigital(_loadPin, LOW);
      _step = STEP_SAMPLE_FIRST;
      schedule(currentMicros);
      break;

    case STEP_SAMPLE_FIRST:
      // The first bit is present before the reset pulse.  Reset clears the
      // upstream event latches ready for the next scan.
      sampleBit(0);
      _bitIndex = 1;
      ArduinoPins::fastWriteDigital(_resetPin, HIGH);
      _step = STEP_RELEASE_RESET;
      schedule(currentMicros);
      break;

    case STEP_RELEASE_RESET:
      ArduinoPins::fastWriteDigital(_resetPin, LOW);
      if (_bitIndex >= (uint16_t)_nPins)
        _step = STEP_FINISH;
      else
        _step = STEP_CLOCK_HIGH;
      schedule(currentMicros);
      break;

    case STEP_CLOCK_HIGH:
      ArduinoPins::fastWriteDigital(_clockPin, HIGH);
      _step = STEP_CLOCK_LOW;
      schedule(currentMicros);
      break;

    case STEP_CLOCK_LOW:
      ArduinoPins::fastWriteDigital(_clockPin, LOW);
      _step = STEP_SAMPLE_CLOCKED;
      schedule(currentMicros);
      break;

    case STEP_SAMPLE_CLOCKED:
      // S88-N specifies that the data output changes on the falling clock
      // edge.  Sampling after the low phase provides the required settling
      // time even when the bus contains slower microcontroller modules.
      sampleBit(_bitIndex);
      _bitIndex++;
      if (_bitIndex >= (uint16_t)_nPins) {
        _step = STEP_FINISH;
        schedule(currentMicros);
      } else {
        ArduinoPins::fastWriteDigital(_clockPin, HIGH);
        _step = STEP_CLOCK_LOW;
        schedule(currentMicros);
      }
      break;

    case STEP_FINISH:
      _publishIndex = 0;
      _step = STEP_PUBLISH;
      delayUntil(currentMicros);
      break;

    case STEP_PUBLISH:
      publishOne(currentMicros);
      break;
    }
  }

  void sampleBit(uint16_t index) {
    bool value = ArduinoPins::fastReadDigital(_dataPin);
    if (_invertData)
      value = !value;

    uint8_t mask = (uint8_t)(1 << (index & 7));
    if (value)
      _states[index >> 3] |= mask;
    else
      _states[index >> 3] &= (uint8_t)~mask;
  }

  void publishOne(unsigned long currentMicros) {
    if (_publishIndex < (uint16_t)_nPins) {
      uint8_t mask = (uint8_t)(1 << (_publishIndex & 7));
      uint8_t byteIndex = (uint8_t)(_publishIndex >> 3);
      bool current = (_states[byteIndex] & mask) != 0;
      bool previous = (_publishedStates[byteIndex] & mask) != 0;
      if (current != previous) {
        if (current)
          _publishedStates[byteIndex] |= mask;
        else
          _publishedStates[byteIndex] &= (uint8_t)~mask;
        if (IONotifyCallback::hasCallback())
          IONotifyCallback::invokeAll(_firstVpin + _publishIndex,
                                      current ? 1 : 0);
      }
      _publishIndex++;
      delayUntil(currentMicros);
      return;
    }

    _step = STEP_START;
    delayUntil(currentMicros + _scanIntervalMicros);
  }

  void schedule(unsigned long currentMicros) {
    delayUntil(currentMicros + _phaseMicros);
  }

  void _display() override {
    DIAG(F("S88 configured on Vpins:%u-%u clock:%u load:%u reset:%u data:%u "
            "period:%luus gap:%luus RAM:%uB"),
         (unsigned)_firstVpin,
         (unsigned)(_firstVpin + _nPins - 1),
         (unsigned)_clockPin,
         (unsigned)_loadPin,
         (unsigned)_resetPin,
         (unsigned)_dataPin,
         (unsigned long)(_phaseMicros * 2),
         (unsigned long)_scanIntervalMicros,
         (unsigned)(_stateByteCount * 2));
  }

  uint8_t _clockPin;
  uint8_t _loadPin;
  uint8_t _resetPin;
  uint8_t _dataPin;
  unsigned long _phaseMicros;
  unsigned long _scanIntervalMicros;
  bool _invertData;
  bool _dataPullup;
  uint16_t _stateByteCount;
  uint8_t *_states = NULL;
  uint8_t *_publishedStates = NULL;
  uint16_t _bitIndex = 0;
  uint16_t _publishIndex = 0;
  Step _step = STEP_START;
  bool _ready = false;
};

#endif // IO_S88_H
