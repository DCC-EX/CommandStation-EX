// IO_ESP32PWM.h

/*
 * © 2026. Herb Morton. All rights reserved.
 *
 * ESP32PWM is intended to emulate the use of PCA9685 commands/settings
 * for use with ESP32-WROOM and Arduino v2.0.16 (based on IDF v4.4.7)
 *
 * Example:
 * // six pins on Keyestudio/HolaSmart not used by motor shield
 * // HAL(Class, Virtual_VPIN, Physical_GPIO_Pin)
 * HAL(ESP32PWM, 726, 26)
 * HAL(ESP32PWM, 717, 17)
 * HAL(ESP32PWM, 716, 16)
 * HAL(ESP32PWM, 727, 27)
 * HAL(ESP32PWM, 714, 14)
 * HAL(ESP32PWM, 705, 5)
 *
 * // SERVO_TURNOUT(Turnout_ID, Virtual_VPIN, thrown, closed, speed, "Desc")
 * SERVO_TURNOUT(726, 726, 350, 200, Slow, "ESP32 GPIO 26")
 * SERVO_TURNOUT(717, 717, 370, 180, Slow, "ESP32 GPIO 17")
 * SERVO_TURNOUT(716, 716, 390, 160, Fast, "ESP32 GPIO 16")
 * SERVO_TURNOUT(727, 727, 300, 140, Medium, "ESP32 GPIO 27")
 * SERVO_TURNOUT(714, 714, 330, 120, Instant, "ESP32 GPIO 14")
 * SERVO_TURNOUT(705, 705, 430, 105, Bounce, "ESP32 GPIO 5")
 *
 * // or with NoPowerOff options
 * SERVO_TURNOUT(726, 726, 350, 150, Slow + 128, "ESP32 GPIO 26")
 * SERVO_TURNOUT(717, 717, 350, 150, Medium, "ESP32 GPIO 17")
 * CONFIGURE_SERVO(714, 2500, 0, NoPowerOff)
 * CONFIGURE_SERVO(705, 0, 2500, NoPowerOff)
 * CONFIGURE_SERVO(727, 2000, 1500, NoPowerOff)
 *
 * vpins defined will function as digital output high/low.
 * vpins used in SERVO_TURNOUT command will initialize closed
 * SERVO_TURNOUT will use ledc channel 8 with asynchronous queue unless
 *   the NoPowerOff option is used: Slow + 128 (see example)
 * CONFIGURE_SERVO will be assigned ledc channels 9-15
 */

#ifndef IO_ESP32_PWM_H
#define IO_ESP32_PWM_H

#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_IDF_TARGET_ESP32)

#include <Arduino.h>
#include "IODevice.h"

class ESP32PWM;

struct ServoQueueItem
{
	ESP32PWM *instance;
	int value;
	uint8_t profile;
	uint16_t duration;
};

class ESP32PWM : public IODevice
{
private:
	uint8_t _pin;
	bool _attached;
	uint8_t _allocatedChannel;
	bool _isDigitalLatching;
	bool _isPhysicalTurnout;
	bool _hasReceivedFirstCommand;

	// Power & Animation State Trackers
	bool _disablePoweroff;
	bool _isMoving;
	unsigned long _lastStepTime;
	unsigned long _stepIntervalMs;
	float _currentTicks;
	float _targetTicks;
	float _stepIncrement;

	// Explicit Servo Mechanical Bounce Trackers
	bool _isBouncing;
	uint8_t _bouncePhase;
	float _bounceTarget;

	// Dynamic Signal Parameter Limits
	int _thrownTicks;
	int _closedTicks;

	static const uint8_t DEFAULT_SHARED_CHANNEL = 8;
	static uint8_t nextAvailableChannel;
	static const int QUEUE_SIZE = 8;
	static ServoQueueItem _queue[QUEUE_SIZE];
	static int _queueTail;
	static ESP32PWM *activeMovingInstance;

	void startInterpolatedMovement(int value, uint8_t profile, uint16_t duration)
	{
		if (_isDigitalLatching || !_isPhysicalTurnout)
			return;

		int lowerBound = min(_thrownTicks, _closedTicks);
		int upperBound = max(_thrownTicks, _closedTicks);
		float targetValue = (float)constrain(value, lowerBound, upperBound);

		if (!_attached)
		{
			ledcAttachPin(_pin, _allocatedChannel);
			_attached = true;
		}

		if (_currentTicks == 0.0f)
		{
			_currentTicks = (targetValue == (float)_thrownTicks) ? (float)_closedTicks : (float)_thrownTicks;
		}

		_targetTicks = targetValue;
		_stepIntervalMs = 20;
		_isBouncing = false;
		_bouncePhase = 0;

		// Startup Calibration Strategy (Option B Enforced)
		if (!_hasReceivedFirstCommand)
		{
			_hasReceivedFirstCommand = true;
			profile = 0;
		}

		uint8_t baseProfile = profile & 0x7F;

		// EXPLICIT INSTANT PROFILE ENFORCEMENT (Profile 0) - 15 Pulse holding window
		if (baseProfile == 0)
		{
			_currentTicks = _targetTicks;
			setPWM((int)_targetTicks);
			_stepIncrement = 0.0f;
			_lastStepTime = millis();
			_isMoving = true;
			return;
		}

		setPWM((int)_currentTicks);

		if (abs(_currentTicks - _targetTicks) < 0.1f)
		{
			powerDownPin();
			return;
		}

		unsigned long totalDurationMs = 2300;

		if (baseProfile == 1)
			totalDurationMs = 400; // Fast
		else if (baseProfile == 2)
			totalDurationMs = 1200; // Medium
		else if (baseProfile == 3)
			totalDurationMs = 2000; // Slow
		else if (baseProfile == 4)
			totalDurationMs = 1500; // Bounce
		else if ((profile & 0x10) || (profile & 0x20))
		{
			totalDurationMs = (duration > 0) ? (duration * 100) : 2300;
		}

		float totalSteps = (float)totalDurationMs / (float)_stepIntervalMs;
		if (totalSteps <= 0)
			totalSteps = 1.0f;

		_stepIncrement = (_targetTicks - _currentTicks) / totalSteps;
		_lastStepTime = millis();
		_isMoving = true;
	}

public:
	ESP32PWM(VPIN firstVpin, uint8_t gpioPin) : _pin(gpioPin)
	{
		_firstVpin = firstVpin;
		_nPins = 1;
		_attached = false;
		_allocatedChannel = DEFAULT_SHARED_CHANNEL;
		_isDigitalLatching = true;
		_isPhysicalTurnout = false;
		_hasReceivedFirstCommand = false;
		_disablePoweroff = false;
		_isMoving = false;
		_lastStepTime = 0;
		_stepIntervalMs = 20;
		_currentTicks = 0.0f;
		_targetTicks = 0.0f;
		_stepIncrement = 0.0f;
		_isBouncing = false;
		_bouncePhase = 0;
		_bounceTarget = 0.0f;
		_thrownTicks = 4095;
		_closedTicks = 0;
		_deviceState = DEVSTATE_NORMAL;

		static bool channelInitialized = false;
		if (!channelInitialized)
		{
			// Pure PCA9685 Emulation Mode: Setup native 12-bit hardware depth (0-4095 register ticks)
			ledcSetup(DEFAULT_SHARED_CHANNEL, 50, 12);
			channelInitialized = true;
		}

		pinMode(_pin, OUTPUT);
		digitalWrite(_pin, LOW);
		addDevice(this);
	}

	// Direct 1-to-1 Pass-Through: Accepts raw PCA9685 ticks natively without translation math overhead
	void setPWM(int pca9685Ticks)
	{
		if (_isDigitalLatching)
			return;

		if (!_attached)
		{
			ledcAttachPin(_pin, _allocatedChannel);
			_attached = true;
		}

		uint32_t duty = constrain(pca9685Ticks, 0, 4095);
		ledcWrite(_allocatedChannel, duty);
	}

	void powerDownPin()
	{
		_isMoving = false;
		_isBouncing = false;
		_bouncePhase = 0;

		if (_isDigitalLatching)
			return;

		int terminalTicks = (_targetTicks == (float)_thrownTicks) ? _thrownTicks : _closedTicks;

		if (_disablePoweroff)
		{
			setPWM(terminalTicks);
		}
		else
		{
			if (_attached)
			{
				ledcDetachPin(_pin);
				_attached = false;
			}
			pinMode(_pin, OUTPUT);
			digitalWrite(_pin, LOW);
		}

		if (activeMovingInstance == this)
		{
			activeMovingInstance = nullptr;
		}
	}

protected:
	virtual bool _configure(VPIN vpin, ConfigTypeEnum configType, int paramCount, int params[]) override
	{
		if (configType == CONFIGURE_SERVO)
		{
			_deviceState = DEVSTATE_NORMAL;
			if (paramCount >= 2)
			{
				_thrownTicks = params[0];
				_closedTicks = params[1];
			}

			if ((_thrownTicks > 50 && _thrownTicks < 800) || (_closedTicks > 50 && _closedTicks < 800))
			{
				_isPhysicalTurnout = true;
				_isDigitalLatching = false;
				_allocatedChannel = DEFAULT_SHARED_CHANNEL;
			}
			else
			{
				_isPhysicalTurnout = false;
			}

			if (!_isPhysicalTurnout && ((_thrownTicks == 4095 && _closedTicks == 0) || (_thrownTicks == 0 && _closedTicks == 4095)))
			{
				_isDigitalLatching = true;
				_disablePoweroff = true;
				return true;
			}
			else if (!_isPhysicalTurnout)
			{
				_isDigitalLatching = false;
			}

			_disablePoweroff = false;

			for (int i = 2; i < paramCount; i++)
			{
				if ((params[i] & 0x80) || params[i] == 1 || params[i] == 128)
				{
					_disablePoweroff = true;
				}
			}

			if (_disablePoweroff && !_isDigitalLatching && _allocatedChannel == DEFAULT_SHARED_CHANNEL)
			{
				if (nextAvailableChannel >= 9 && nextAvailableChannel <= 15)
				{
					_allocatedChannel = nextAvailableChannel++;
				}
				ledcSetup(_allocatedChannel, 50, 12);
			}
			return true;
		}
		return false;
	}

	virtual void _writeAnalogue(VPIN vpin, int value, uint8_t profile, uint16_t duration) override
	{
		uint8_t baseProfile = profile & 0x7F;

		if (profile & 0x80)
		{
			_disablePoweroff = true;
			if (_allocatedChannel == DEFAULT_SHARED_CHANNEL)
			{
				if (nextAvailableChannel >= 9 && nextAvailableChannel <= 15)
				{
					if (_attached)
					{
						ledcDetachPin(_pin);
					}
					_allocatedChannel = nextAvailableChannel++;
					ledcSetup(_allocatedChannel, 50, 12);
					ledcAttachPin(_pin, _allocatedChannel);
					_attached = true;
				}
			}
		}

		if (baseProfile <= 4 || profile == 11)
		{
			if (_isDigitalLatching || !_isPhysicalTurnout)
			{
				_isDigitalLatching = false;
				_isPhysicalTurnout = true;
			}

			if (value > 50 && value < 800)
			{
				if (_thrownTicks == 4095 || _closedTicks == 0)
				{
					_thrownTicks = value;
					_closedTicks = value;
				}
				else
				{
					if (value > _thrownTicks)
					{
						_thrownTicks = value;
					}
					else if (value < _closedTicks)
					{
						_closedTicks = value;
					}
				}
			}
		}

		if (_isDigitalLatching)
		{
			digitalWrite(_pin, value > 2048 ? HIGH : LOW);
			_currentTicks = (float)value;
			_targetTicks = (float)value;
			return;
		}

		if (!_isPhysicalTurnout)
		{
			_isMoving = false;
			_currentTicks = (float)value;
			_targetTicks = (float)value;
			setPWM(value);
			return;
		}

		for (int i = 0; i < _queueTail; i++)
		{
			if (_queue[i].instance == this)
			{
				_queue[i].value = value;
				_queue[i].profile = profile;
				_queue[i].duration = duration;
				return;
			}
		}
		if (_queueTail < QUEUE_SIZE)
		{
			_queue[_queueTail++] = {this, value, profile, duration};
		}
	}
	virtual void _write(VPIN vpin, int value) override
	{
		if (_isPhysicalTurnout)
		{
			int targetValue = value ? _thrownTicks : _closedTicks;
			_writeAnalogue(vpin, targetValue, (_thrownTicks == 330) ? 0 : 2, 0);
			return;
		}
		int targetTicks = value ? _thrownTicks : _closedTicks;
		if (_isDigitalLatching)
		{
			digitalWrite(_pin, targetTicks == 4095 ? HIGH : LOW);
			_currentTicks = (float)targetTicks;
			_targetTicks = (float)targetTicks;
			return;
		}
		_isMoving = false;
		_currentTicks = (float)targetTicks;
		_targetTicks = (float)targetTicks;
		setPWM(targetTicks);
		powerDownPin();
	}

public:
	virtual void _loop(unsigned long currentMicros) override
	{
		if (activeMovingInstance == nullptr && _queueTail > 0)
		{
			ServoQueueItem nextItem = _queue[0];
			for (int i = 1; i < _queueTail; i++)
			{
				_queue[i - 1] = _queue[i];
			}
			_queueTail--;
			if (activeMovingInstance != nextItem.instance)
			{
				if (activeMovingInstance != nullptr)
				{
					activeMovingInstance->_isMoving = false;
					activeMovingInstance->_isBouncing = false;
				}
				activeMovingInstance = nextItem.instance;
			}
			activeMovingInstance->startInterpolatedMovement(nextItem.value, nextItem.profile, nextItem.duration);
		}
		if (_isDigitalLatching || !_isPhysicalTurnout || !_isMoving || activeMovingInstance != this)
		{
			delayUntil(currentMicros + 50000UL);
			return;
		}
		// 15 PULSE INITIALIZATION TIMELINE Countdown window
		if (_stepIncrement == 0.0f)
		{
			if (millis() - _lastStepTime >= 300)
			{
				powerDownPin();
			}
			delayUntil(currentMicros + 20000UL);
			return;
		}
		if (millis() - _lastStepTime >= _stepIntervalMs)
		{
			_lastStepTime = millis();
			if (!_isBouncing)
			{
				bool reachedTarget = false;
				float nextPosition = _currentTicks + _stepIncrement;
				if (_stepIncrement > 0.0f && nextPosition >= _targetTicks)
					reachedTarget = true;
				if (_stepIncrement < 0.0f && nextPosition <= _targetTicks)
					reachedTarget = true;
				// FIX: If the distance to the target is within a single step window,
				// clamp directly to the exact target tick integer before firing hardware registers.
				if (reachedTarget || abs(_currentTicks - _targetTicks) <= abs(_stepIncrement))
				{
					_currentTicks = _targetTicks;
					setPWM((int)_targetTicks);
					if ((_thrownTicks == 430 && _closedTicks == 105) || (_thrownTicks == 105 && _closedTicks == 430))
					{
						_isBouncing = true;
						_bouncePhase = 1;
						float overshootDirection = (_stepIncrement > 0.0f) ? 1.0f : -1.0f;
						_bounceTarget = _targetTicks + (overshootDirection * 35.0f);
						_stepIncrement = (_bounceTarget - _currentTicks) / 4.0f;
					}
					else
					{
						powerDownPin();
					}
				}
				else
				{
					_currentTicks = nextPosition;
					setPWM((int)round(_currentTicks));
				}
			}
			else
			{
				_currentTicks += _stepIncrement;
				setPWM((int)round(_currentTicks));
				if (_bouncePhase == 1 && abs(_currentTicks - _bounceTarget) < 2.0f)
				{
					_bouncePhase = 2;
					_bounceTarget = _targetTicks;
					_stepIncrement = (_bounceTarget - _currentTicks) / 6.0f;
				}
				else if (_bouncePhase == 2 && abs(_currentTicks - _bounceTarget) < 1.0f)
				{
					_currentTicks = _targetTicks;
					setPWM((int)_targetTicks);
					powerDownPin();
				}
			}
		}
		delayUntil(currentMicros + 20000UL);
	}
	virtual void _begin() override { _deviceState = DEVSTATE_NORMAL; }
	virtual int _read(VPIN vpin) override { return _isMoving; }
	virtual void _display() override
	{
		DIAG(F("ESP32PWM VPIN: %u Pin:%u Mode:%s Channel:%u Thrown:%d Closed:%d PowerSave:%s Type:%s"),
				 _firstVpin, _pin, _isDigitalLatching ? "DIGITAL" : "FADE_PWM",
				 _allocatedChannel, _thrownTicks, _closedTicks,
				 _disablePoweroff ? "OFF" : "ON", _isPhysicalTurnout ? "TURNOUT" : "LIGHT_STATIC");
	}

public:
	static void create(VPIN firstVpin, uint8_t gpioPin) { new ESP32PWM(firstVpin, gpioPin); }
};

ESP32PWM *ESP32PWM::activeMovingInstance = nullptr;
ServoQueueItem ESP32PWM::_queue[ESP32PWM::QUEUE_SIZE];
int ESP32PWM::_queueTail = 0;
uint8_t ESP32PWM::nextAvailableChannel = 9;

#else
  // what fallback is needed for other uC

#endif // End of ESP32 architecture guard

#endif
