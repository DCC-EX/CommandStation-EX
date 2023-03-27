/*
 *  © 2021, Neil McKechnie. All rights reserved.
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

/*
 * The HC-SR04 module has an ultrasonic transmitter (40kHz) and a receiver.
 * It is operated through two signal pins.  When the transmit pin is set to 1
 * for 10us, on the falling edge the transmitter sends a short transmission of
 * 8 pulses (like a sonar 'ping').  This is reflected off objects and received
 * by the receiver.  A pulse is sent on the receive pin whose length is equal
 * to the delay between the transmission of the pulse and the detection of
 * its echo.  The distance of the reflecting object is calculated by halving
 * the time (to allow for the out and back distance), then multiplying by the
 * speed of sound (assumed to be constant).
 *
 * This driver polls the HC-SR04 by sending the trigger pulse and then measuring
 * the length of the received pulse.  If the calculated distance is less than
 * the threshold, the output _state returned by a read() call changes to 1.  If
 * the distance is greater than the threshold plus a hysteresis margin, the
 * output changes to 0. The device also supports readAnalogue(), which returns
 * the measured distance in cm, or 32767 if the distance exceeds the
 * offThreshold.
 *
 * It might be thought that the measurement would be more reliable if interrupts
 * were disabled while the pulse is being timed.  However, this would affect
 * other functions in the CS so the measurement is being performed with
 * interrupts enabled.  Also, we could use an interrupt pin in the Arduino for
 * the timing, but the same consideration applies.  In any case, the DCC
 * interrupt occurs once every 58us, so any IRC code is much faster than that.
 * And 58us corresponds to 1cm in the calculation, so the effect of
 * interrupts is negligible.
 *
 * Note: The timing accuracy required for measuring the pulse length means that
 * the pins have to be direct Arduino pins; GPIO pins on an IO Extender cannot
 * provide the required accuracy.
 * 
 * Example configuration:
 *  HCSR04::create(23000, 32, 33, 80, 85);
 * 
 * Where 23000 is the VPIN allocated,
 *       32 is the pin connected to the HCSR04 trigger terminal,
 *       33 is the pin connected to the HCSR04 echo terminal,
 *       80 is the distance in cm below which pin 23000 will be active,
 *   and 85 is the distance in cm above which pin 23000 will be inactive.
 * 
 * Alternative configuration, which hogs the processor until the measurement is complete
 * (old behaviour, more accurate but higher impact on other CS tasks):
 *  HCSR04::create(23000, 32, 33, 80, 85, HCSR04::LOOP);
 * 
 */

#ifndef IO_HCSR04_H
#define IO_HCSR04_H

#include "IODevice.h"

class HCSR04 : public IODevice {

private:
  // pins must be arduino GPIO pins, not extender pins or HAL pins.
  int _trigPin = -1;
  int _echoPin = -1;
  // Thresholds for setting active _state in cm.
  uint8_t _onThreshold;  // cm
  uint8_t _offThreshold; // cm
  // Last measured distance in cm.
  uint16_t _distance;
  // Active=1/inactive=0 _state 
  uint8_t _value = 0;
  // Factor for calculating the distance (cm) from echo time (us).
  //  Based on a speed of sound of 345 metres/second.
  const uint16_t factor = 58; // us/cm
  // Limit the time spent looping by dropping out when the expected
  // worst case threshold value is greater than an arbitrary value.
  const uint16_t maxPermittedLoopTime = 10 * factor; // max in us
  unsigned long _startTime = 0;
  unsigned long _maxTime = 0;
  enum {DORMANT, MEASURING}; // _state values
  uint8_t _state = DORMANT;
  uint8_t _counter = 0;
  uint16_t _options = 0;

public:
  enum Options {
    LOOP = 1,  // Option HCSR04::LOOP reinstates old behaviour, i.e. complete measurement in one loop entry.
  };
 
  // Static create function provides alternative way to create object
  static void create(VPIN vpin, int trigPin, int echoPin, uint16_t onThreshold, uint16_t offThreshold, uint16_t options = 0) {
    if (checkNoOverlap(vpin))
        new HCSR04(vpin, trigPin, echoPin, onThreshold, offThreshold, options);
  }

protected:
  // Constructor performs static initialisation of the device object
  HCSR04 (VPIN vpin, int trigPin, int echoPin, uint16_t onThreshold, uint16_t offThreshold, uint16_t options) {
    _firstVpin = vpin;
    _nPins = 1;
    _trigPin = trigPin;
    _echoPin = echoPin;
    _onThreshold = onThreshold;
    _offThreshold = offThreshold;
    _options = options;
    addDevice(this);
  }
 // _begin function called to perform dynamic initialisation of the device
  void _begin() override {
    _state = 0;
    pinMode(_trigPin, OUTPUT);
    pinMode(_echoPin, INPUT);
    ArduinoPins::fastWriteDigital(_trigPin, 0);
#if defined(DIAG_IO)
    _display();
#endif
  }

  // _read function - just return _value (calculated in _loop).
  int _read(VPIN vpin) override {
    (void)vpin;  // avoid compiler warning
    return _value;
  }

  int _readAnalogue(VPIN vpin) override {
    (void)vpin; // avoid compiler warning
    return _distance;
  }

  // _loop function - read HC-SR04 once every 100 milliseconds.
  void _loop(unsigned long currentMicros) override {
    unsigned long waitTime;
    switch(_state) {
      case DORMANT: // Issue pulse
        // If receive pin is still set on from previous call, do nothing till next entry.
        if (ArduinoPins::fastReadDigital(_echoPin)) return;

        // Send 10us pulse to trigger transmitter
        ArduinoPins::fastWriteDigital(_trigPin, 1);
        delayMicroseconds(10);
        ArduinoPins::fastWriteDigital(_trigPin, 0);

        // Wait, with timeout, for echo pin to become set.
        // Measured time delay is just under 500us, so 
        // wait for max of 1000us.
        _startTime = micros();
        _maxTime = 1000;

        while (!ArduinoPins::fastReadDigital(_echoPin)) {
          // Not set yet, see if we've timed out.
          waitTime = micros() - _startTime;
          if (waitTime > _maxTime) {
            // Timeout waiting for pulse start, abort the read and start again
            _state = DORMANT;
            return;
          }
        }

        // Echo pulse started, so wait for echo pin to reset, and measure length of pulse
        _startTime = micros();
        _maxTime = factor * _offThreshold;
        _state = MEASURING;
        // If maximum measurement time is high, then skip until next loop entry before 
        // starting to look for pulse end.
        // This gives better accuracy at shorter distance thresholds but without extending 
        // loop execution time for longer thresholds.  If LOOP option is set on, then
        // the entire measurement will be done in one loop entry, i.e. the code will fall
        // through into the measuring phase.
        if (!(_options & LOOP) && _maxTime > maxPermittedLoopTime) break;
        /* fallthrough */

      case MEASURING: // Check if echo pulse has finished
        do {
          waitTime = micros() - _startTime;
          if (!ArduinoPins::fastReadDigital(_echoPin)) {
            // Echo pulse completed; check if pulse length is below threshold and if so set value.
            if (waitTime <= factor * _onThreshold) {
              // Measured time is within the onThreshold, so value is one.
              _value = 1;
              // If the new distance value is less than the current, use it immediately.
              // But if the new distance value is longer, then it may be erroneously long
              // (because of extended loop times delays), so apply a delay to distance increases.
              uint16_t estimatedDistance = waitTime / factor;
              if (estimatedDistance < _distance) 
                _distance = estimatedDistance;
              else
                _distance += 1;  // Just increase distance slowly.
              _counter = 0;
              //DIAG(F("HCSR04: Pulse Len=%l Distance=%d"), waitTime, _distance);
            }
            _state = DORMANT;
          } else {
            // Echo pulse hasn't finished, so check if maximum time has elapsed
            // If pulse is too long then set return value to zero,
            //  and finish without waiting for end of pulse.
            if (waitTime > _maxTime) {
              // Pulse length longer than maxTime, value is provisionally zero.
              // But don't change _value unless provisional value is zero for 10 consecutive measurements
              if (_value == 1) {
                if (++_counter >= 10) {
                  _value = 0;
                  _distance = 32767;
                  _counter = 0;
                }
              }
              _state = DORMANT; // start again
            }
          }
          // If there's lots of time remaining before the expected completion time,
          // then exit and wait for next loop entry.  Otherwise, loop until we finish.
          // If option LOOP is set, then we loop until finished anyway.
          uint32_t remainingTime = _maxTime - waitTime;
          if (!(_options & LOOP) && remainingTime < maxPermittedLoopTime) return;
        } while (_state == MEASURING) ;
        break;
    }
    // Datasheet recommends a wait of at least 60ms between measurement cycles
    if (_state == DORMANT)
      delayUntil(currentMicros+60000UL); // wait 60ms till next measurement

  }

  void _display() override {
    DIAG(F("HCSR04 Configured on VPIN:%u TrigPin:%d EchoPin:%d On:%dcm Off:%dcm"),
      _firstVpin, _trigPin, _echoPin, _onThreshold, _offThreshold);
  }

};

#endif //IO_HCSR04_H
