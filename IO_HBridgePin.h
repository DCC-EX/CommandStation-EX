/*
 * Non-blocking, momentary digital output used by HBridgeTurnout.
 */
#ifndef IO_HBRIDGE_PIN_H
#define IO_HBRIDGE_PIN_H

#include "IODevice.h"
#include <Arduino.h>

class HBridgePin : public IODevice {
public:
  static const uint16_t MaxPulseMillis = 500;

  static void create(VPIN pin, uint16_t pulseMillis) {
    new HBridgePin(pin, pulseMillis);
  }

protected:
  HBridgePin(VPIN pin, uint16_t pulseMillis) : IODevice(pin, 1),
      _pulseMicros((unsigned long)(pulseMillis > MaxPulseMillis ? MaxPulseMillis : pulseMillis) * 1000UL) {
    IODevice *controlledDevice = IODevice::findDevice(pin);
    if (controlledDevice != NULL) {
      addDevice(this, controlledDevice);
    } else {
      _deviceState = DEVSTATE_FAILED;
    }
  }

  void _begin() override {
    ArduinoPins::fastWriteDigital(_firstVpin, LOW);
  }

  void _write(VPIN vpin, int value) override {
    if (_deviceState == DEVSTATE_FAILED || vpin != _firstVpin) return;
    ArduinoPins::fastWriteDigital(_firstVpin, value ? HIGH : LOW);
    if (value) {
      delayUntil(micros() + _pulseMicros);
    } else {
      delayUntil(micros() + 0x7fffffff);
    }
  }

  void _loop(unsigned long currentMicros) override {
    ArduinoPins::fastWriteDigital(_firstVpin, LOW);
    delayUntil(currentMicros + 0x7fffffff);
  }

private:
  unsigned long _pulseMicros;
};

#endif
