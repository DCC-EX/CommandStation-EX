/*
 *  © 2023, Sergei Kotlyachkov. All rights reserved.
 *
 *  This file is part of DCC++EX API
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


#ifndef IO_SCHEDULED_PIN_H
#define IO_SCHEDULED_PIN_H

#include "IODevice.h"
#include <Arduino.h>
#include "defines.h"

/**
 * Bounces back single Arduino Pin to specified state after set period of time.
 *
 * It will establish itself as owner of the pin over ArduinoPins class that typically responds to it and
 * activates itself during loop() phase. It restores scheduled state and does not try again until
 * another write()
 *
 * Example usage:
 * Create: ScheduledPin::create(5, LOW, 20000);
 *
 * Then, when neeeded, just call:
 * IODevice::write(5, HIGH); // this will call fastWriteDigital(5, HIGH)
 *
 * In 20 milliseconds, it will also call fastWriteDigital(5, LOW)
 *
 * In edge case where write() is called twice before responding in the loop,
 * the schedule will restart and double the bounce back time.
 */
class ScheduledPin : public IODevice {
private:
  int _scheduledValue;
  uint32_t _durationMicros;

public:
  //  Static function to handle create calls.
  static void create(VPIN pin, int scheduledValue, uint32_t durationMicros) {
    new ScheduledPin(pin, scheduledValue, durationMicros);
  }

protected:
  // Constructor.
  ScheduledPin(VPIN pin, int scheduledValue, uint32_t durationMicros) : IODevice(pin, 1)  {
    _scheduledValue = scheduledValue;
    _durationMicros = durationMicros;
    // Typically returned device will be ArduinoPins
    IODevice* controlledDevice = IODevice::findDevice(pin);
    if (controlledDevice != NULL) {
      addDevice(this, controlledDevice);
    }
    else {
      DIAG(F("ScheduledPin Controlled device not found for VPIN:%d"), pin);
      _deviceState = DEVSTATE_FAILED;
    }
  }

  // Device-specific initialisation
  void _begin() override {
    #ifdef DIAG_IO
    _display();
    #endif
    pinMode(_firstVpin, OUTPUT);
    ArduinoPins::fastWriteDigital(_firstVpin, _scheduledValue);
  }

  void _write(VPIN vpin, int value) override {
    if (_deviceState == DEVSTATE_FAILED) return;
    if (vpin != _firstVpin) {
      #ifdef DIAG_IO
      DIAG(F("ScheduledPin Error VPIN:%u not equal to %u"), vpin, _firstVpin);
      #endif
      return;
    }
    #ifdef DIAG_IO
    DIAG(F("ScheduledPin Write VPIN:%u Value:%d Micros:%l"), vpin, value, micros());
    #endif
    unsigned long currentMicros = micros();
    delayUntil(currentMicros + _durationMicros);
    ArduinoPins::fastWriteDigital(_firstVpin, value);
  }


  void _loop(unsigned long currentMicros) {
    if (_deviceState == DEVSTATE_FAILED) return;
    #ifdef DIAG_IO
    DIAG(F("ScheduledPin Bounce VPIN:%u Value:%d Micros:%l"), _firstVpin, _scheduledValue, micros());
    #endif
    ArduinoPins::fastWriteDigital(_firstVpin, _scheduledValue);
    delayUntil(currentMicros + 0x7fffffff); // Largest time in the future!  Effectively disable _loop calls.
  }

  // Display information about the device, and perhaps its current condition (e.g. active, disabled etc).
  void _display() {
    DIAG(F("ScheduledPin Configured:%u Value:%d Duration:%l"), (int)_firstVpin,
      _scheduledValue, _durationMicros);
  }
};

#endif // IO_SCHEDULED_PIN_H
