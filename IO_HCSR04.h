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
 * the threshold, the output state returned by a read() call changes to 1.  If
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
 */

#ifndef IO_HCSR04_H
#define IO_HCSR04_H

#include "IODevice.h"

class HCSR04 : public IODevice {

private:
  // pins must be arduino GPIO pins, not extender pins or HAL pins.
  int _trigPin = -1;
  int _echoPin = -1;
  // Thresholds for setting active state in cm.
  uint8_t _onThreshold;  // cm
  uint8_t _offThreshold; // cm
  // Last measured distance in cm.
  uint16_t _distance;
  // Active=1/inactive=0 state 
  uint8_t _value = 0;
  // Factor for calculating the distance (cm) from echo time (ms).
  //  Based on a speed of sound of 345 metres/second.
  const uint16_t factor = 58; // ms/cm

public:
  // Constructor perfroms static initialisation of the device object
  HCSR04 (VPIN vpin, int trigPin, int echoPin, uint16_t onThreshold, uint16_t offThreshold) {
    _firstVpin = vpin;
    _nPins = 1;
    _trigPin = trigPin;
    _echoPin = echoPin;
    _onThreshold = onThreshold;
    _offThreshold = offThreshold;
    addDevice(this);
  }

  // Static create function provides alternative way to create object
  static void create(VPIN vpin, int trigPin, int echoPin, uint16_t onThreshold, uint16_t offThreshold) {
    new HCSR04(vpin, trigPin, echoPin, onThreshold, offThreshold);
  }

protected:
  // _begin function called to perform dynamic initialisation of the device
  void _begin() override {
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

  // _loop function - read HC-SR04 once every 50 milliseconds.
  void _loop(unsigned long currentMicros) override {
    read_HCSR04device();
    // Delay next loop entry until 50ms have elapsed.
    delayUntil(currentMicros + 50000UL);
  }

  void _display() override {
    DIAG(F("HCSR04 Configured on Vpin:%d TrigPin:%d EchoPin:%d On:%dcm Off:%dcm"),
      _firstVpin, _trigPin, _echoPin, _onThreshold, _offThreshold);
  }

private:
  // This polls the HC-SR04 device by sending a pulse and measuring the duration of
  //  the pulse observed on the receive pin.  In order to be kind to the rest of the CS
  //  software, no interrupts are used and interrupts are not disabled.  The pulse duration
  //  is measured in a loop, using the micros() function.  Therefore, interrupts from other
  //  sources may affect the result.  However, interrupts response code in CS typically takes
  //  much less than the 58us frequency for the DCC interrupt, and 58us corresponds to only 1cm
  //  in the HC-SR04.
  //  To reduce chatter on the output, hysteresis is applied on reset: the output is set to 1 when the 
  //  measured distance is less than the onThreshold, and is set to 0 if the measured distance is
  //  greater than the offThreshold.
  //
  void read_HCSR04device() {
    // uint16 enough to time up to 65ms
    uint16_t startTime, waitTime, currentTime, maxTime;  

    // If receive pin is still set on from previous call, abort the read.
    if (ArduinoPins::fastReadDigital(_echoPin))
      return;

    // Send 10us pulse to trigger transmitter
    ArduinoPins::fastWriteDigital(_trigPin, 1);
    delayMicroseconds(10);
    ArduinoPins::fastWriteDigital(_trigPin, 0);

    // Wait for receive pin to be set
    startTime = currentTime = micros();
    maxTime = factor * _offThreshold * 2;
    while (!ArduinoPins::fastReadDigital(_echoPin)) {
      // lastTime = currentTime;
      currentTime = micros();
      waitTime = currentTime - startTime;
      if (waitTime > maxTime) {
        // Timeout waiting for pulse start, abort the read
        return;
      }
    }

    // Wait for receive pin to reset, and measure length of pulse
    startTime = currentTime = micros();
    maxTime = factor * _offThreshold;
    while (ArduinoPins::fastReadDigital(_echoPin)) {
      currentTime = micros();
      waitTime = currentTime - startTime;
      // If pulse is too long then set return value to zero,
      //  and finish without waiting for end of pulse.
      if (waitTime > maxTime) {
        // Pulse length longer than maxTime, reset value.
        _value = 0;
        _distance = 32767;
        return;
      }
    } 
    // Check if pulse length is below threshold, if so set value.
    //DIAG(F("HCSR04: Pulse Len=%l Distance=%d"), waitTime, distance);
    _distance = waitTime / factor; // in centimetres
    if (_distance < _onThreshold) 
      _value = 1;
  }

};

#endif //IO_HCSR04_H