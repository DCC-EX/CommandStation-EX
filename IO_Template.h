
/*
* Creation - a create() function and constructor are required;
* Initialisation - a _begin() function is written (optional);
* Background operations - a _loop() function is written (optional);
* Operations - you can optionally supply any of _write() (digital) function, _writeAnalogue() function, _read() (digital) function and _readAnalogue() function.
*
*
*
*
*
*
*/


#ifndef IO_MYDEVICE_H
#define IO_MYDEVICE_H

#include "IODevice.h"
#include "DIAG.h"  // for DIAG calls

class MyDevice: public IODevice {
public:
  // Constructor
  MyDevice(VPIN firstVpin, int nPins) {
    _firstVpin = firstVpin;
    _nPins = min(nPins,16);
    // Other object initialisation here
    // ...
    addDevice(this);
  }
  static void create(VPIN firstVpin, int nPins, uint8_t i2cAddress) {
    new MyDevice(firstVpin, nPins);
  }
private:
  void _begin() override {
    // Initialise device
    // ...
  }
  void _loop(unsigned long currentMicros) override {
    // Regular operations, e.g. acquire data
    // ...
    delayUntil(currentMicros + 10*1000UL);  // 10ms till next entry
  }
  int _readAnalogue(VPIN vpin) override {
    // Return acquired data value, e.g.
    int pin = vpin - _firstVpin;
    return _value[pin];
  }
  int _read(VPIN vpin) override {
    // Return acquired data value, e.g.
    int pin = vpin - _firstVpin;
    return _value[pin];
  }
  void write(VPIN vpin, int value) override {
    // Do something with value , e.g. write to device.
    // ...
  }
  void writeAnalogue(VPIN vpin, int value) override {
    // Do something with value, e.g. write to device.
    // ...
  }
  void _display() override {
    DIAG(F("MyDevice Configured on Vpins:%d-%d %S"), _firstVpin, _firstVpin+_nPins-1,
      _deviceState == DEVSTATE_FAILED ? F("OFFLINE") : F(""));
  }
  uint16_t _value[16];
};
#endif // IO_MYDEVICE_H