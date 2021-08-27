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
 * It is operated through two signal pins.  When the transmit pin is set to 1 for 
 * 10us, on the falling edge the transmitter sends a short transmission of 
 * 8 pulses (like a sonar 'ping').  This is reflected off objects and received
 * by the receiver.  A pulse is sent on the receive pin whose length is equal
 * to the delay between the transmission of the pulse and the detection of
 * its echo.  The distance of the reflecting object is calculated by halving
 * the time (to allow for the out and back distance), then multiplying by the
 * speed of sound (assumed to be constant).
 * 
 * This driver polls the HC-SR04 by sending the trigger pulse and then measuring
 * the length of the received pulse.  If the calculated distance is less than the
 * threshold, the output changes to 1.  If it is greater than the threshold plus
 * a hysteresis margin, the output changes to 0.
 * 
 * The measurement would be more reliable if interrupts were disabled while the
 * pulse is being timed.  However, this would affect other functions in the CS
 * so the measurement is being performed with interrupts enabled.  Also, we could
 * use an interrupt pin in the Arduino for the timing, but the same consideration 
 * applies.
 * 
 * Note: The timing accuracy required by this means that the pins have to be 
 * direct Arduino pins; GPIO pins on an IO Extender cannot provide the required
 * accuracy.
 */

#ifndef IO_HCSR04_H
#define IO_HCSR04_H

#include "IODevice.h"

class HCSR04 : public IODevice {

private:
  // pins must be arduino GPIO pins, not extender pins or HAL pins.
  int _transmitPin = -1;
  int _receivePin = -1;
  // Thresholds for setting active state in cm.
  uint8_t _onThreshold;  // cm
  uint8_t _offThreshold; // cm
  // Active=1/inactive=0 state 
  uint8_t _value = 0;
  // Time of last loop execution
  unsigned long _lastExecutionTime;
  // Factor for calculating the distance (cm) from echo time (ms).
  //  Based on a speed of sound of 345 metres/second.
  const uint16_t factor = 58; // ms/cm

public:
  // Constructor perfroms static initialisation of the device object
  HCSR04 (VPIN vpin, int transmitPin, int receivePin, uint16_t onThreshold, uint16_t offThreshold) {
    _firstVpin = vpin;
    _nPins = 1;
    _transmitPin = transmitPin;
    _receivePin = receivePin;
    _onThreshold = onThreshold;
    _offThreshold = offThreshold;
    addDevice(this);
  }

  // Static create function provides alternative way to create object
  static void create(VPIN vpin, int transmitPin, int receivePin, uint16_t onThreshold, uint16_t offThreshold) {
    new HCSR04(vpin, transmitPin, receivePin, onThreshold, offThreshold);
  }

protected:
  // _begin function called to perform dynamic initialisation of the device
  void _begin() override {
    pinMode(_transmitPin, OUTPUT);
    pinMode(_receivePin, INPUT);
    ArduinoPins::fastWriteDigital(_transmitPin, 0);
    _lastExecutionTime = micros();
#if defined(DIAG_IO)
    _display();
#endif
  }

  // _read function - just return _value (calculated in _loop).
  int _read(VPIN vpin) override {
    (void)vpin;  // avoid compiler warning
    return _value;
  }

  // _loop function - read HC-SR04 once every 50 milliseconds.
  void _loop(unsigned long currentMicros) override {
    if (currentMicros - _lastExecutionTime > 50000UL) {
      _lastExecutionTime = currentMicros;

      _value = read_HCSR04device();
    }
  }

  void _display() override {
    DIAG(F("HCSR04 Configured on Vpin:%d TrigPin:%d EchoPin:%d On:%dcm Off:%dcm"),
      _firstVpin, _transmitPin, _receivePin, _onThreshold, _offThreshold);
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
  uint8_t read_HCSR04device() {
    // uint16 enough to time up to 65ms
    uint16_t startTime, waitTime, currentTime, maxTime;  

    // If receive pin is still set on from previous call, abort the read.
    if (ArduinoPins::fastReadDigital(_receivePin)) return _value;

    // Send 10us pulse to trigger transmitter
    ArduinoPins::fastWriteDigital(_transmitPin, 1);
    delayMicroseconds(10);
    ArduinoPins::fastWriteDigital(_transmitPin, 0);

    // Wait for receive pin to be set
    startTime = currentTime = micros();
    maxTime = factor * _offThreshold * 2;
    while (!ArduinoPins::fastReadDigital(_receivePin)) {
      // lastTime = currentTime;
      currentTime = micros();
      waitTime = currentTime - startTime;
      if (waitTime > maxTime) {
        // Timeout waiting for pulse start, abort the read
        return _value;
      }
    }

    // Wait for receive pin to reset, and measure length of pulse
    startTime = currentTime = micros();
    maxTime = factor * _offThreshold;
    while (ArduinoPins::fastReadDigital(_receivePin)) {
      currentTime = micros();
      waitTime = currentTime - startTime;
      // If pulse is too long then set return value to zero,
      //  and finish without waiting for end of pulse.
      if (waitTime > maxTime) {
        // Pulse length longer than maxTime, reset value.
        return 0;
      }
    } 
    // Check if pulse length is below threshold, if so set value.
    //DIAG(F("HCSR04: Pulse Len=%l Distance=%d"), waitTime, distance);
    uint16_t distance = waitTime / factor; // in centimetres
    if (distance < _onThreshold) 
      return 1;

    return _value;
  }

};

#endif //IO_HCSR04_H