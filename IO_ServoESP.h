/*
 * @file IO_ServoESP.h
 *
 *  © 2026 Ross Scanlon
 */

#ifndef IO_SERVOESP_H
#define IO_SERVOESP_H

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
#ifndef SERVO_MIN_PULSE_US
#define SERVO_MIN_PULSE_US 1000
#endif
#ifndef SERVO_MAX_PULSE_US
#define SERVO_MAX_PULSE_US 2000
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

	static void create(VPIN firstVpin, int nPins, uint8_t firstGpioPin,
										 uint8_t firstLedcChannel = 0) {
		if (nPins < 1 || nPins > 2 || firstLedcChannel + nPins > 16) return;
		if (checkNoOverlap(firstVpin, nPins))
			new ServoESP(firstVpin, nPins, firstGpioPin, firstLedcChannel);
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
	};

	static const uint8_t MaxServos = 2;
	static const uint16_t MaxPosition = 4095;
	static const unsigned long DetachDelayMs = 200;
	static const unsigned long RefreshIntervalMs = 50;

	uint8_t _firstGpioPin;
	uint8_t _firstLedcChannel;
	ServoData _servoData[MaxServos] = {};

	ServoESP(VPIN firstVpin, int nPins, uint8_t firstGpioPin,
					 uint8_t firstLedcChannel) {
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

	bool _configure(VPIN vpin, ConfigTypeEnum configType, int paramCount,
									int params[]) override {
		if (configType != CONFIGURE_SERVO || paramCount != 5 ||
				vpin < _firstVpin || vpin >= _firstVpin + _nPins) return false;

		const uint8_t pin = vpin - _firstVpin;
		ServoData &servo = _servoData[pin];
		servo.activePosition = clampPosition(params[0]);
		servo.inactivePosition = clampPosition(params[1]);
		servo.profile = params[2];
		servo.duration = params[3] < 0 ? 0 : params[3];
		if (params[4] != -1)
			_writeAnalogue(vpin, params[4] ? servo.activePosition : servo.inactivePosition,
										 servo.profile, servo.duration);
		return true;
	}

	void _write(VPIN vpin, int value) override {
		if (vpin < _firstVpin || vpin >= _firstVpin + _nPins) return;
		const uint8_t pin = vpin - _firstVpin;
		ServoData &servo = _servoData[pin];
		_writeAnalogue(vpin, value ? servo.activePosition : servo.inactivePosition,
									 servo.profile, servo.duration);
	}

	void _writeAnalogue(VPIN vpin, int value, uint8_t profile = 0,
											uint16_t duration = 0) override {
		if (_deviceState == DEVSTATE_FAILED || vpin < _firstVpin ||
				vpin >= _firstVpin + _nPins) return;

		ServoData &servo = _servoData[vpin - _firstVpin];
		const unsigned long now = millis();
		servo.keepPowerOn = (profile & NoPowerOff) != 0;
		servo.travelTimeMs = travelTime(profile & ~NoPowerOff, duration);
		servo.startPosition = servo.currentPosition;
		servo.targetPosition = clampPosition(value);
		servo.startedAt = now;
		servo.lastActionAt = now;
		servo.moving = servo.startPosition != servo.targetPosition && servo.travelTimeMs != 0;

		if (!attachOutput(vpin - _firstVpin)) return;
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
		DIAG(F("ServoESP GPIO:%u-%u Vpins:%u-%u %S"), _firstGpioPin,
				 _firstGpioPin + _nPins - 1, (int)_firstVpin,
				 (int)_firstVpin + _nPins - 1,
				 (_deviceState == DEVSTATE_FAILED) ? F("OFFLINE") : F(""));
	}

	static uint16_t clampPosition(int value) {
		if (value < 0) return 0;
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
		if (servo.outputAttached) return true;

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

	void writePosition(uint8_t pin, uint16_t position) {
		const uint32_t angle = (uint32_t)position * 180 / MaxPosition;
		const uint32_t pwmPeriodUs = 1000000UL / SERVO_PWM_FREQUENCY;
		const uint32_t resolution = UINT32_C(1) << SERVO_PWM_RESOLUTION;
		const uint32_t minDuty = resolution * SERVO_MIN_PULSE_US / pwmPeriodUs;
		const uint32_t maxDuty = resolution * SERVO_MAX_PULSE_US / pwmPeriodUs;
		const uint32_t duty = minDuty + (maxDuty - minDuty) * angle / 180;
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
		} else if (servo.outputAttached && !servo.keepPowerOn &&
							 now - servo.lastActionAt >= DetachDelayMs) {
			detachOutput(pin);
		}
	}
};

#endif // ARDUINO_ARCH_ESP32
#endif // IO_SERVOESP_H