/*
 * @file IO_ServoESP.h
 *
 *  © 2026 Ross Scanlon
 * 
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

#ifndef IO_SERVOESP_H
#define IO_SERVOESP_H

#ifdef DCCEX_NODE

#include "IODevice.h"
#include "DIAG.h"

#if defined(ARDUINO_ARCH_ESP32)
#include <Arduino.h>
#if defined(__has_include)
#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#endif
#endif

#ifndef SERVO_PWM_FREQUENCY
#define SERVO_PWM_FREQUENCY 50
#endif
#ifndef SERVO_PWM_RESOLUTION
#define SERVO_PWM_RESOLUTION 16
#endif


class ServoESP : public IODevice {
public:
  enum ProfileType : uint8_t {
    Instant = 0,
    UseDuration = 0,
    Fast = 1,
    Medium = 2,
    Slow = 3,
    NoPowerOff = 0x80,
  };

  static void create(VPIN firstVpin, int nPins, uint8_t firstGpioPin, uint8_t firstLedcChannel = 8) {
    if (nPins < 1 || nPins > 2 || firstLedcChannel + nPins > 16) {
      DIAG(F("!!firstVpin: %d nPins:  %d firstLedcChannel : %d error!!"), firstVpin, nPins, firstLedcChannel);
      return;
    }
    if (checkNoOverlap(firstVpin, nPins)) {
      new ServoESP(firstVpin, nPins, firstGpioPin, firstLedcChannel);
    }

  }

private:
  struct ServoData {
    uint16_t activePosition;
    uint16_t inactivePosition;
    uint16_t currentPosition;
    uint16_t startPosition;
    uint16_t targetPosition;
    uint32_t travelTimeMs;
    uint16_t duration;
    unsigned long startedAt;
    unsigned long lastActionAt;
    uint8_t profile;
    bool moving;
    bool outputAttached;
    bool keepPowerOn;
    bool positionInitialized;
  };

  static const uint8_t MaxServos = 2;
  static const uint16_t MaxPosition = 4095;
  static const unsigned long DetachDelayMs = 200;
  static const unsigned long RefreshIntervalMs = 50;

  uint8_t _firstGpioPin;
  uint8_t _firstLedcChannel;
  ServoData _servoData[MaxServos] = {};

/**
 * @brief Provides servo control on ESP32-Wroom MCU without external hardware.
 *        Maximum 2 servos and needs two consecutive gpio
 *        Example gpio 13-14, 16-17, 18-19 
 * @param firstVpin first vpin for IO_Device
 * @param nPins number of vpins
 * @param firstGpioPin first ESP32 gpio to use
 * @param firstLedcChannel first ledc channel to use (default = 8)
 * @note  Only provide the firstLedcChannel if you really know what you are doing.
 **/

  ServoESP(VPIN firstVpin, int nPins, uint8_t firstGpioPin, uint8_t firstLedcChannel) {
    _firstVpin = firstVpin;
    _nPins = (nPins > MaxServos) ? MaxServos : nPins;
    _firstGpioPin = firstGpioPin;
    _firstLedcChannel = firstLedcChannel;
    for (int pin = 0; pin < _nPins; pin++) {
      _servoData[pin].activePosition = MaxPosition;
      _servoData[pin].inactivePosition = 0;
      _servoData[pin].profile = Instant | NoPowerOff;
    }
    addDevice(this);
  }

  bool _configure(VPIN vpin, ConfigTypeEnum configType, int paramCount, int params[]) override {
    if (configType != CONFIGURE_SERVO || paramCount != 5 || vpin < _firstVpin || vpin >= _firstVpin + _nPins) {
      return false;
    }
    const uint8_t pin = vpin - _firstVpin;
    ServoData &servo = _servoData[pin];
    servo.activePosition = clampPosition(params[0]);
    servo.inactivePosition = clampPosition(params[1]);
    servo.profile = params[2];
    servo.duration = params[3] < 0 ? 0 : params[3];
    if (params[4] != -1) {
      _writeAnalogue(vpin, params[4] ? servo.activePosition : servo.inactivePosition, servo.profile, servo.duration);
    }
    return true;
  }

  void _write(VPIN vpin, int value) override {
    if (vpin < _firstVpin || vpin >= _firstVpin + _nPins) {
      return;
    }
    const uint8_t pin = vpin - _firstVpin;
    ServoData &servo = _servoData[pin];
    _writeAnalogue(vpin, value ? servo.activePosition : servo.inactivePosition, servo.profile, servo.duration);
  }

/**
 * @brief Device specific writeAnalogue function, invoked from IODevice::writeAnalogue()
 * @param vpin CS vpin
 * @param value postion value
 * @param profile movement profile (default = 0) see ProfileType
 * @param duration duration of movement (default = 0)
 **/
  void _writeAnalogue(VPIN vpin, int value, uint8_t profile = 0, uint16_t duration = 0) override {
    if (_deviceState == DEVSTATE_FAILED || vpin < _firstVpin || vpin >= _firstVpin + _nPins) {
      return;
    }
    ServoData &servo = _servoData[vpin - _firstVpin];
    const unsigned long now = millis();
    servo.keepPowerOn = (profile & NoPowerOff) != 0;
    servo.travelTimeMs = travelTime(profile & ~NoPowerOff, duration);
    servo.targetPosition = clampPosition(value);
    servo.startedAt = now;
    servo.lastActionAt = now;
    if (!servo.positionInitialized) {
      servo.currentPosition = servo.targetPosition;
      servo.positionInitialized = true;
      servo.moving = false;
    } else {
      servo.startPosition = servo.currentPosition;
      servo.moving = servo.startPosition != servo.targetPosition && servo.travelTimeMs != 0;
    }

    if (!attachOutput(vpin - _firstVpin)) {
      return;
    }
    if (!servo.moving) {
      servo.currentPosition = servo.targetPosition;
      writePosition(vpin - _firstVpin, servo.currentPosition);
    }
  }

  int _read(VPIN vpin) override {
    if (vpin < _firstVpin || vpin >= _firstVpin + _nPins) return 0;
    return _servoData[vpin - _firstVpin].moving;
  }

  void _loop(unsigned long currentMicros) override {
    const unsigned long now = millis();
    for (uint8_t pin = 0; pin < _nPins; pin++) updatePosition(pin, now);
    delayUntil(currentMicros + RefreshIntervalMs * 1000UL);
  }

  void _display() override {
    DIAG(F("ServoESP GPIO:%u-%u Vpins:%u-%u %S"), _firstGpioPin, _firstGpioPin + _nPins - 1, (int)_firstVpin,
            (int)_firstVpin + _nPins - 1, (_deviceState == DEVSTATE_FAILED) ? F("OFFLINE") : F(""));
  }

  static uint16_t clampPosition(int value) {
    if (value < 0) {
      return 0;
    }
    return value > MaxPosition ? MaxPosition : value;
  }

  static uint32_t travelTime(uint8_t profile, uint16_t duration) {
    if (profile == Fast) return 500;
    if (profile == Medium) return 1000;
    if (profile == Slow) return 2000;
    return (uint32_t)duration * 100;
  }

  bool attachOutput(uint8_t pin) {
    ServoData &servo = _servoData[pin];
    if (servo.outputAttached) {
      return true;
    }

    const uint8_t gpioPin = _firstGpioPin + pin;
    pinMode(gpioPin, OUTPUT);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    servo.outputAttached = ledcAttach(gpioPin, SERVO_PWM_FREQUENCY, SERVO_PWM_RESOLUTION);
#else
    const uint8_t channel = _firstLedcChannel + pin;
    servo.outputAttached = ledcSetup(channel, SERVO_PWM_FREQUENCY, SERVO_PWM_RESOLUTION) > 0;
    if (servo.outputAttached) ledcAttachPin(gpioPin, channel);
#endif
    return servo.outputAttached;
  }

/**
 * @brief takes a pin in the range 0 to nPins-1 and a value between 0 and 4095 for the PWM mark-to-period ratio, with 4095 being 100%
 * @param pin range 0 to _nPins -1
 * @param position PWM mark-to-period ratio, 0 - 4095, 4095 = 100%
 **/
  void writePosition(uint8_t pin, uint16_t position) {
    const uint32_t resolution = UINT32_C(1) << SERVO_PWM_RESOLUTION;
    const uint32_t duty = position == MaxPosition ? resolution - 1 :
      (uint32_t)position * resolution / (MaxPosition + 1);

    DIAG(F("ServoESP position:%u duty:%lu"), position, (unsigned long)duty);
    

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(_firstGpioPin + pin, duty);
#else
    ledcWrite(_firstLedcChannel + pin, duty);
#endif
  }

  void detachOutput(uint8_t pin) {
    ServoData &servo = _servoData[pin];
    if (!servo.outputAttached) return;
    const uint8_t gpioPin = _firstGpioPin + pin;
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcDetach(gpioPin);
#else
    ledcDetachPin(gpioPin);
#endif
    digitalWrite(gpioPin, LOW);
    servo.outputAttached = false;
  }

  void updatePosition(uint8_t pin, unsigned long now) {
    ServoData &servo = _servoData[pin];
    if (servo.moving) {
      const unsigned long elapsed = now - servo.startedAt;
      if (elapsed >= servo.travelTimeMs) {
        servo.currentPosition = servo.targetPosition;
        servo.moving = false;
        servo.lastActionAt = now;
      } else {
        const int32_t change = (int32_t)servo.targetPosition - servo.startPosition;
        servo.currentPosition = servo.startPosition +
          (int32_t)(change * (int32_t)elapsed / (int32_t)servo.travelTimeMs);
        servo.lastActionAt = now;
      }
      writePosition(pin, servo.currentPosition);
    } else {
      if (servo.outputAttached && !servo.keepPowerOn && now - servo.lastActionAt >= DetachDelayMs) {
        detachOutput(pin);
      }
    }
  }
};

#endif // ARDUINO_ARCH_ESP32
#endif // DCCEX_NODE
#endif // IO_SERVOESP_H
