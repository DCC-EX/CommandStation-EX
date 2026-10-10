/*
 *  This file is part of CommandStation-EX
 *
 *  This is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 */

#ifndef IO_EXTURNTABLE_ONBOARD_H
#define IO_EXTURNTABLE_ONBOARD_H

#include "defines.h"
#include "AccelStepper.h"
#include "DIAG.h"
#include "IODevice.h"
#include "Turntables.h"
#include "CommandDistributor.h"
#include "NVSTable.h"

namespace EXTurntableOnboard {
  enum NVSId : uint8_t {
    StepPin = 1,
    DirPin,
    EnablePin,
    HomeSensorPin,
    LimitSensorPin,
    RelayPin,
    LedPin,
    AccessoryPin,
    ExtraPin1,
    ExtraPin2,
    ExtraPin3,
    ExtraPin4,
    MaxSpeed,
    Acceleration,
    FullStepCount,
    SanitySteps,
    HomeSensitivity,
    PhaseSwitchAngle,
    GearingFactor,
    HomeSensorActiveState,
    LimitSensorActiveState,
    RelayActiveState,
    DebounceMs,
    LedSlowMs,
    LedFastMs,
    Mode,
    RotationMode,
    PhaseSwitchMode,
    InvertDirection,
    InvertStep,
    InvertEnable,
    KeepOutputsEnabled,
    ExtraOutputsEnabled
  };

  struct Configuration {
    uint8_t stepPin;
    uint8_t dirPin;
    uint8_t enablePin;
    uint8_t homeSensorPin;
    uint8_t limitSensorPin;
    uint8_t relayPin;
    uint8_t ledPin;
    uint8_t accessoryPin;
    uint8_t extraPins[4];
    int16_t maxSpeed;
    int16_t acceleration;
    long fullStepCount;
    long sanitySteps;
    long homeSensitivity;
    uint16_t phaseSwitchAngle;
    uint8_t gearingFactor;
    uint8_t homeSensorActiveState;
    uint8_t limitSensorActiveState;
    uint8_t relayActiveState;
    uint16_t debounceMs;
    uint16_t ledSlowMs;
    uint16_t ledFastMs;
    uint8_t mode;
    uint8_t rotationMode;
    uint8_t phaseSwitchMode;
    bool invertDirection;
    bool invertStep;
    bool invertEnable;
    bool keepOutputsEnabled;
    bool extraOutputsEnabled;
  };

  const Configuration &configuration();
  void begin();
  void service();
  bool isBusy();
  bool command(long steps, uint8_t activity);
}

class EXTurntable_OB : public IODevice {
public:
  static void create(VPIN firstVpin, int nPins);
  EXTurntable_OB(VPIN firstVpin, int nPins);

  enum ActivityNumber : uint8_t {
    Turn = 0,
    Turn_PInvert = 1,
    Home = 2,
    Calibrate = 3,
    LED_On = 4,
    LED_Slow = 5,
    LED_Fast = 6,
    LED_Off = 7,
    Acc_On = 8,
    Acc_Off = 9,
  };

private:
  void _begin() override;
  void _loop(unsigned long currentMicros) override;
  int _read(VPIN vpin) override;
  void _broadcastStatus(VPIN vpin, uint8_t status, uint8_t activity);
  void _writeAnalogue(
      VPIN vpin, int value, uint8_t activity, uint16_t duration) override;
  void _display() override;
  uint8_t _stepperStatus;
  uint8_t _previousStatus;
  uint8_t _currentActivity;
};

#if defined(CONFIG_IDF_TARGET_ESP32S3)
  #define EXTT_DEFAULT_LIMIT_PIN D8
  #define EXTT_DEFAULT_HOME_PIN D5
  #define EXTT_DEFAULT_RELAY_PIN D4
  #define EXTT_DEFAULT_LED_PIN D6
  #define EXTT_DEFAULT_ACCESSORY_PIN D7
  #define EXTT_DEFAULT_STEP_PIN A0
  #define EXTT_DEFAULT_DIR_PIN A1
  #define EXTT_DEFAULT_ENABLE_PIN A2
  #define EXTT_DEFAULT_EXTRA_PIN_1 D9
  #define EXTT_DEFAULT_EXTRA_PIN_2 D10
  #define EXTT_DEFAULT_EXTRA_PIN_3 D11
  #define EXTT_DEFAULT_EXTRA_PIN_4 D12
#elif defined(ARDUINO_ARCH_ESP32)
  #define EXTT_DEFAULT_LIMIT_PIN 34
  #define EXTT_DEFAULT_HOME_PIN 35
  #define EXTT_DEFAULT_RELAY_PIN 27
  #define EXTT_DEFAULT_LED_PIN 32
  #define EXTT_DEFAULT_ACCESSORY_PIN 33
  #define EXTT_DEFAULT_STEP_PIN 17
  #define EXTT_DEFAULT_DIR_PIN 16
  #define EXTT_DEFAULT_ENABLE_PIN 18
  #define EXTT_DEFAULT_EXTRA_PIN_1 14
  #define EXTT_DEFAULT_EXTRA_PIN_2 13
  #define EXTT_DEFAULT_EXTRA_PIN_3 15
  #define EXTT_DEFAULT_EXTRA_PIN_4 19
#else
  #define EXTT_DEFAULT_LIMIT_PIN 8
  #define EXTT_DEFAULT_HOME_PIN 5
  #define EXTT_DEFAULT_RELAY_PIN 4
  #define EXTT_DEFAULT_LED_PIN 6
  #define EXTT_DEFAULT_ACCESSORY_PIN 7
  #define EXTT_DEFAULT_STEP_PIN A0
  #define EXTT_DEFAULT_DIR_PIN A1
  #define EXTT_DEFAULT_ENABLE_PIN A2
  #define EXTT_DEFAULT_EXTRA_PIN_1 9
  #define EXTT_DEFAULT_EXTRA_PIN_2 10
  #define EXTT_DEFAULT_EXTRA_PIN_3 11
  #define EXTT_DEFAULT_EXTRA_PIN_4 12
#endif

#ifndef EX_TURNTABLE_LIMIT_SENSOR_PIN
  #ifdef LIMIT_SENSOR_PIN
    #define EX_TURNTABLE_LIMIT_SENSOR_PIN LIMIT_SENSOR_PIN
  #else
  #define EX_TURNTABLE_LIMIT_SENSOR_PIN EXTT_DEFAULT_LIMIT_PIN
  #endif
#endif
#ifndef EX_TURNTABLE_HOME_SENSOR_PIN
  #ifdef HOME_SENSOR_PIN
    #define EX_TURNTABLE_HOME_SENSOR_PIN HOME_SENSOR_PIN
  #else
  #define EX_TURNTABLE_HOME_SENSOR_PIN EXTT_DEFAULT_HOME_PIN
  #endif
#endif
#ifndef EX_TURNTABLE_RELAY_PIN
  #ifdef RELAY_PIN
    #define EX_TURNTABLE_RELAY_PIN RELAY_PIN
  #else
  #define EX_TURNTABLE_RELAY_PIN EXTT_DEFAULT_RELAY_PIN
  #endif
#endif
#ifndef EX_TURNTABLE_LED_PIN
  #ifdef LED_PIN
    #define EX_TURNTABLE_LED_PIN LED_PIN
  #else
  #define EX_TURNTABLE_LED_PIN EXTT_DEFAULT_LED_PIN
  #endif
#endif
#ifndef EX_TURNTABLE_ACCESSORY_PIN
  #ifdef ACC_PIN
    #define EX_TURNTABLE_ACCESSORY_PIN ACC_PIN
  #else
  #define EX_TURNTABLE_ACCESSORY_PIN EXTT_DEFAULT_ACCESSORY_PIN
  #endif
#endif
#ifndef EX_TURNTABLE_STEP_PIN
  #ifdef STEPPER_STEP_PIN
    #define EX_TURNTABLE_STEP_PIN STEPPER_STEP_PIN
  #else
  #define EX_TURNTABLE_STEP_PIN EXTT_DEFAULT_STEP_PIN
  #endif
#endif
#ifndef EX_TURNTABLE_DIR_PIN
  #ifdef STEPPER_DIR_PIN
    #define EX_TURNTABLE_DIR_PIN STEPPER_DIR_PIN
  #else
  #define EX_TURNTABLE_DIR_PIN EXTT_DEFAULT_DIR_PIN
  #endif
#endif
#ifndef EX_TURNTABLE_ENABLE_PIN
  #ifdef STEPPER_ENABLE_PIN
    #define EX_TURNTABLE_ENABLE_PIN STEPPER_ENABLE_PIN
  #else
  #define EX_TURNTABLE_ENABLE_PIN EXTT_DEFAULT_ENABLE_PIN
  #endif
#endif
#ifndef EX_TURNTABLE_EXTRA_PIN_1
  #ifdef EXTRA_OUTPUT_PIN_1
    #define EX_TURNTABLE_EXTRA_PIN_1 EXTRA_OUTPUT_PIN_1
  #else
  #define EX_TURNTABLE_EXTRA_PIN_1 EXTT_DEFAULT_EXTRA_PIN_1
  #endif
#endif
#ifndef EX_TURNTABLE_EXTRA_PIN_2
  #ifdef EXTRA_OUTPUT_PIN_2
    #define EX_TURNTABLE_EXTRA_PIN_2 EXTRA_OUTPUT_PIN_2
  #else
  #define EX_TURNTABLE_EXTRA_PIN_2 EXTT_DEFAULT_EXTRA_PIN_2
  #endif
#endif
#ifndef EX_TURNTABLE_EXTRA_PIN_3
  #ifdef EXTRA_OUTPUT_PIN_3
    #define EX_TURNTABLE_EXTRA_PIN_3 EXTRA_OUTPUT_PIN_3
  #else
  #define EX_TURNTABLE_EXTRA_PIN_3 EXTT_DEFAULT_EXTRA_PIN_3
  #endif
#endif
#ifndef EX_TURNTABLE_EXTRA_PIN_4
  #ifdef EXTRA_OUTPUT_PIN_4
    #define EX_TURNTABLE_EXTRA_PIN_4 EXTRA_OUTPUT_PIN_4
  #else
  #define EX_TURNTABLE_EXTRA_PIN_4 EXTT_DEFAULT_EXTRA_PIN_4
  #endif
#endif

#ifndef EX_TURNTABLE_HOME_SENSOR_ACTIVE_STATE
  #ifdef HOME_SENSOR_ACTIVE_STATE
    #define EX_TURNTABLE_HOME_SENSOR_ACTIVE_STATE HOME_SENSOR_ACTIVE_STATE
  #else
  #define EX_TURNTABLE_HOME_SENSOR_ACTIVE_STATE LOW
  #endif
#endif
#ifndef EX_TURNTABLE_LIMIT_SENSOR_ACTIVE_STATE
  #ifdef LIMIT_SENSOR_ACTIVE_STATE
    #define EX_TURNTABLE_LIMIT_SENSOR_ACTIVE_STATE LIMIT_SENSOR_ACTIVE_STATE
  #else
  #define EX_TURNTABLE_LIMIT_SENSOR_ACTIVE_STATE LOW
  #endif
#endif
#ifndef EX_TURNTABLE_RELAY_ACTIVE_STATE
  #ifdef RELAY_ACTIVE_STATE
    #define EX_TURNTABLE_RELAY_ACTIVE_STATE RELAY_ACTIVE_STATE
  #else
  #define EX_TURNTABLE_RELAY_ACTIVE_STATE HIGH
  #endif
#endif
#ifndef EX_TURNTABLE_MAX_SPEED
  #ifdef STEPPER_MAX_SPEED
    #define EX_TURNTABLE_MAX_SPEED STEPPER_MAX_SPEED
  #else
  #define EX_TURNTABLE_MAX_SPEED 200
  #endif
#endif
#ifndef EX_TURNTABLE_ACCELERATION
  #ifdef STEPPER_ACCELERATION
    #define EX_TURNTABLE_ACCELERATION STEPPER_ACCELERATION
  #else
  #define EX_TURNTABLE_ACCELERATION 25
  #endif
#endif
#ifndef EX_TURNTABLE_SANITY_STEPS
  #ifdef SANITY_STEPS
    #define EX_TURNTABLE_SANITY_STEPS SANITY_STEPS
  #else
  #define EX_TURNTABLE_SANITY_STEPS 10000L
  #endif
#endif
#ifndef EX_TURNTABLE_HOME_SENSITIVITY
  #ifdef HOME_SENSITIVITY
    #define EX_TURNTABLE_HOME_SENSITIVITY HOME_SENSITIVITY
  #else
  #define EX_TURNTABLE_HOME_SENSITIVITY 300L
  #endif
#endif
#ifndef EX_TURNTABLE_PHASE_SWITCH_ANGLE
  #ifdef PHASE_SWITCH_ANGLE
    #define EX_TURNTABLE_PHASE_SWITCH_ANGLE PHASE_SWITCH_ANGLE
  #else
  #define EX_TURNTABLE_PHASE_SWITCH_ANGLE 45
  #endif
#endif
#ifndef EX_TURNTABLE_GEARING_FACTOR
  #ifdef STEPPER_GEARING_FACTOR
    #define EX_TURNTABLE_GEARING_FACTOR STEPPER_GEARING_FACTOR
  #else
  #define EX_TURNTABLE_GEARING_FACTOR 1
  #endif
#endif
#ifndef EX_TURNTABLE_LED_SLOW_MS
  #ifdef LED_SLOW
    #define EX_TURNTABLE_LED_SLOW_MS LED_SLOW
  #else
  #define EX_TURNTABLE_LED_SLOW_MS 500
  #endif
#endif
#ifndef EX_TURNTABLE_LED_FAST_MS
  #ifdef LED_FAST
    #define EX_TURNTABLE_LED_FAST_MS LED_FAST
  #else
  #define EX_TURNTABLE_LED_FAST_MS 100
  #endif
#endif
#if defined(TURNTABLE_EX_MODE) && defined(TRAVERSER) && \
    TURNTABLE_EX_MODE == TRAVERSER && !defined(EX_TURNTABLE_MODE_TRAVERSER)
  #define EX_TURNTABLE_MODE_TRAVERSER
#endif
#if defined(PHASE_SWITCHING) && defined(MANUAL) && \
    PHASE_SWITCHING == MANUAL && !defined(EX_TURNTABLE_MANUAL_PHASE_SWITCHING)
  #define EX_TURNTABLE_MANUAL_PHASE_SWITCHING
#endif
#if defined(ROTATE_FORWARD_ONLY) && !defined(EX_TURNTABLE_ROTATE_FORWARD_ONLY)
  #define EX_TURNTABLE_ROTATE_FORWARD_ONLY
#endif
#if defined(ROTATE_REVERSE_ONLY) && !defined(EX_TURNTABLE_ROTATE_REVERSE_ONLY)
  #define EX_TURNTABLE_ROTATE_REVERSE_ONLY
#endif
#ifndef EX_TURNTABLE_DEBOUNCE_MS
  #ifdef DEBOUNCE_DELAY
    #define EX_TURNTABLE_DEBOUNCE_MS DEBOUNCE_DELAY
  #elif defined(EX_TURNTABLE_MODE_TRAVERSER)
    #define EX_TURNTABLE_DEBOUNCE_MS 10
  #else
    #define EX_TURNTABLE_DEBOUNCE_MS 0
  #endif
#endif
#ifndef EX_TURNTABLE_FULL_STEP_COUNT
  #ifdef FULL_STEP_COUNT
    #define EX_TURNTABLE_FULL_STEP_COUNT FULL_STEP_COUNT
  #else
    #define EX_TURNTABLE_FULL_STEP_COUNT 0L
  #endif
#endif

namespace EXTurntableOnboardDetail {
inline constexpr uint8_t TurntableMode = 1;
inline constexpr uint8_t TraverserMode = 2;
inline constexpr uint8_t ShortestRotation = 1;
inline constexpr uint8_t ForwardRotation = 2;
inline constexpr uint8_t ReverseRotation = 3;
inline constexpr uint8_t AutomaticPhaseSwitching = 1;
inline constexpr uint8_t ManualPhaseSwitching = 2;
inline constexpr uint8_t StoredLowState = 1;
inline constexpr uint8_t StoredHighState = 2;

inline constexpr uint8_t ActivityTurnPhaseInvert = 1;
inline constexpr uint8_t ActivityHome = 2;
inline constexpr uint8_t ActivityCalibrate = 3;
inline constexpr uint8_t ActivityLedOn = 4;
inline constexpr uint8_t ActivityLedSlow = 5;
inline constexpr uint8_t ActivityLedFast = 6;
inline constexpr uint8_t ActivityLedOff = 7;
inline constexpr uint8_t ActivityAccessoryOn = 8;
inline constexpr uint8_t ActivityAccessoryOff = 9;

inline AccelStepper *stepper = nullptr;
inline EXTurntableOnboard::Configuration config{};
inline long fullTurnSteps = 0;
inline long halfTurnSteps = 0;
inline long phaseSwitchStartSteps = 0;
inline long phaseSwitchStopSteps = 0;
inline long lastStep = 0;
inline long lastTarget = EX_TURNTABLE_SANITY_STEPS;
inline uint8_t homed = 0;
inline uint8_t calibrationPhase = 0;
inline bool calibrating = false;
inline uint8_t ledState = ActivityLedOff;
inline bool ledOutput = false;
inline bool lastRunningState = false;
inline bool lastHomeSensorState = false;
inline bool lastLimitSensorState = false;
inline unsigned long lastHomeDebounce = 0;
inline unsigned long lastLimitDebounce = 0;
inline unsigned long ledMillis = 0;
inline bool configurationLoaded = false;
inline long gearingFactor = 1;

inline int16_t storedSetting(EXTurntableOnboard::NVSId nvsId) {
  return NVSTable::getNVS(nvsId, true);
}

inline uint8_t storedState(EXTurntableOnboard::NVSId nvsId, uint8_t defaultState) {
  const int16_t value = storedSetting(nvsId);
  if (value == StoredLowState) return LOW;
  if (value == StoredHighState) return HIGH;
  return defaultState;
}

inline bool storedBoolean(EXTurntableOnboard::NVSId nvsId, bool defaultValue) {
  const int16_t value = storedSetting(nvsId);
  if (value == 1) return true;
  if (value == 2) return false;
  return defaultValue;
}

inline void loadConfiguration() {
  config.stepPin = EX_TURNTABLE_STEP_PIN;
  config.dirPin = EX_TURNTABLE_DIR_PIN;
  config.enablePin = EX_TURNTABLE_ENABLE_PIN;
  config.homeSensorPin = EX_TURNTABLE_HOME_SENSOR_PIN;
  config.limitSensorPin = EX_TURNTABLE_LIMIT_SENSOR_PIN;
  config.relayPin = EX_TURNTABLE_RELAY_PIN;
  config.ledPin = EX_TURNTABLE_LED_PIN;
  config.accessoryPin = EX_TURNTABLE_ACCESSORY_PIN;
  config.extraPins[0] = EX_TURNTABLE_EXTRA_PIN_1;
  config.extraPins[1] = EX_TURNTABLE_EXTRA_PIN_2;
  config.extraPins[2] = EX_TURNTABLE_EXTRA_PIN_3;
  config.extraPins[3] = EX_TURNTABLE_EXTRA_PIN_4;
  config.maxSpeed = EX_TURNTABLE_MAX_SPEED;
  config.acceleration = EX_TURNTABLE_ACCELERATION;
  config.fullStepCount = EX_TURNTABLE_FULL_STEP_COUNT;
  config.sanitySteps = EX_TURNTABLE_SANITY_STEPS;
  config.homeSensitivity = EX_TURNTABLE_HOME_SENSITIVITY;
  config.phaseSwitchAngle = EX_TURNTABLE_PHASE_SWITCH_ANGLE;
  config.gearingFactor = EX_TURNTABLE_GEARING_FACTOR;
  gearingFactor = config.gearingFactor;
  config.homeSensorActiveState = EX_TURNTABLE_HOME_SENSOR_ACTIVE_STATE;
  config.limitSensorActiveState = EX_TURNTABLE_LIMIT_SENSOR_ACTIVE_STATE;
  config.relayActiveState = EX_TURNTABLE_RELAY_ACTIVE_STATE;
  config.debounceMs = EX_TURNTABLE_DEBOUNCE_MS;
  config.ledSlowMs = EX_TURNTABLE_LED_SLOW_MS;
  config.ledFastMs = EX_TURNTABLE_LED_FAST_MS;
#ifdef EX_TURNTABLE_MODE_TRAVERSER
  config.mode = TraverserMode;
#else
  config.mode = TurntableMode;
#endif
#ifdef EX_TURNTABLE_ROTATE_FORWARD_ONLY
  config.rotationMode = ForwardRotation;
#elif defined(EX_TURNTABLE_ROTATE_REVERSE_ONLY)
  config.rotationMode = ReverseRotation;
#else
  config.rotationMode = ShortestRotation;
#endif
#ifdef EX_TURNTABLE_MANUAL_PHASE_SWITCHING
  config.phaseSwitchMode = ManualPhaseSwitching;
#else
  config.phaseSwitchMode = AutomaticPhaseSwitching;
#endif
#if defined(EX_TURNTABLE_INVERT_DIRECTION) || defined(INVERT_DIRECTION)
  config.invertDirection = true;
#else
  config.invertDirection = false;
#endif
#if defined(EX_TURNTABLE_INVERT_STEP) || defined(INVERT_STEP)
  config.invertStep = true;
#else
  config.invertStep = false;
#endif
#if defined(EX_TURNTABLE_INVERT_ENABLE) || defined(INVERT_ENABLE)
  config.invertEnable = true;
#else
  config.invertEnable = false;
#endif
#ifdef EX_TURNTABLE_KEEP_OUTPUTS_ENABLED
  config.keepOutputsEnabled = true;
#else
  config.keepOutputsEnabled = false;
#endif
#ifdef USE_RT_EX_TURNTABLE
  config.extraOutputsEnabled = true;
#else
  config.extraOutputsEnabled = false;
#endif

#define LOAD_PIN(setting, field) do { \
  const int16_t value = storedSetting(setting); \
  if (value > 0 && value <= 255) config.field = (uint8_t)value; \
} while (0)
#define LOAD_POSITIVE(setting, field) do { \
  const int16_t value = storedSetting(setting); \
  if (value > 0) config.field = value; \
} while (0)
  LOAD_PIN(EXTurntableOnboard::StepPin, stepPin);
  LOAD_PIN(EXTurntableOnboard::DirPin, dirPin);
  LOAD_PIN(EXTurntableOnboard::EnablePin, enablePin);
  LOAD_PIN(EXTurntableOnboard::HomeSensorPin, homeSensorPin);
  LOAD_PIN(EXTurntableOnboard::LimitSensorPin, limitSensorPin);
  LOAD_PIN(EXTurntableOnboard::RelayPin, relayPin);
  LOAD_PIN(EXTurntableOnboard::LedPin, ledPin);
  LOAD_PIN(EXTurntableOnboard::AccessoryPin, accessoryPin);
  LOAD_PIN(EXTurntableOnboard::ExtraPin1, extraPins[0]);
  LOAD_PIN(EXTurntableOnboard::ExtraPin2, extraPins[1]);
  LOAD_PIN(EXTurntableOnboard::ExtraPin3, extraPins[2]);
  LOAD_PIN(EXTurntableOnboard::ExtraPin4, extraPins[3]);
  LOAD_POSITIVE(EXTurntableOnboard::MaxSpeed, maxSpeed);
  LOAD_POSITIVE(EXTurntableOnboard::Acceleration, acceleration);
  LOAD_POSITIVE(EXTurntableOnboard::FullStepCount, fullStepCount);
  LOAD_POSITIVE(EXTurntableOnboard::SanitySteps, sanitySteps);
  LOAD_POSITIVE(EXTurntableOnboard::HomeSensitivity, homeSensitivity);
  LOAD_POSITIVE(EXTurntableOnboard::PhaseSwitchAngle, phaseSwitchAngle);
  LOAD_POSITIVE(EXTurntableOnboard::GearingFactor, gearingFactor);
  LOAD_POSITIVE(EXTurntableOnboard::DebounceMs, debounceMs);
  LOAD_POSITIVE(EXTurntableOnboard::LedSlowMs, ledSlowMs);
  LOAD_POSITIVE(EXTurntableOnboard::LedFastMs, ledFastMs);
#undef LOAD_PIN
#undef LOAD_POSITIVE

  const int16_t mode = storedSetting(EXTurntableOnboard::Mode);
  if (mode == TurntableMode || mode == TraverserMode) config.mode = mode;
  const int16_t rotation = storedSetting(EXTurntableOnboard::RotationMode);
  if (rotation >= ShortestRotation && rotation <= ReverseRotation) {
    config.rotationMode = rotation;
  }
  const int16_t phase = storedSetting(EXTurntableOnboard::PhaseSwitchMode);
  if (phase == AutomaticPhaseSwitching || phase == ManualPhaseSwitching) {
    config.phaseSwitchMode = phase;
  }
  config.homeSensorActiveState = storedState(
      EXTurntableOnboard::HomeSensorActiveState, config.homeSensorActiveState);
  config.limitSensorActiveState = storedState(
      EXTurntableOnboard::LimitSensorActiveState, config.limitSensorActiveState);
  config.relayActiveState = storedState(
      EXTurntableOnboard::RelayActiveState, config.relayActiveState);
  config.invertDirection = storedBoolean(
      EXTurntableOnboard::InvertDirection, config.invertDirection);
  config.invertStep = storedBoolean(
      EXTurntableOnboard::InvertStep, config.invertStep);
  config.invertEnable = storedBoolean(
      EXTurntableOnboard::InvertEnable, config.invertEnable);
  config.keepOutputsEnabled = storedBoolean(
      EXTurntableOnboard::KeepOutputsEnabled, config.keepOutputsEnabled);
  config.extraOutputsEnabled = storedBoolean(
      EXTurntableOnboard::ExtraOutputsEnabled, config.extraOutputsEnabled);

  if (gearingFactor == 0) gearingFactor = 1;
  if (gearingFactor > 10) gearingFactor = 10;
  config.gearingFactor = gearingFactor;
  if (config.phaseSwitchAngle >= 180) {
    DIAG(F("EX-Turntable_OB phase switch angle %u is invalid; using 45"),
         config.phaseSwitchAngle);
    config.phaseSwitchAngle = 45;
  }
  fullTurnSteps = config.fullStepCount;
  halfTurnSteps = fullTurnSteps / 2;
  lastTarget = config.sanitySteps;
  stepper = new AccelStepper(AccelStepper::DRIVER, config.stepPin, config.dirPin);
  stepper->setMaxSpeed(config.maxSpeed);
  stepper->setAcceleration(config.acceleration);
  stepper->setEnablePin(config.enablePin);
  stepper->setPinsInverted(
      config.invertDirection, config.invertStep, config.invertEnable);
  configurationLoaded = true;
}

inline bool homeSensorState() {
  const bool currentState = digitalRead(config.homeSensorPin);
  if (currentState != lastHomeSensorState &&
      millis() - lastHomeDebounce > config.debounceMs) {
    lastHomeDebounce = millis();
    lastHomeSensorState = currentState;
  }
  return lastHomeSensorState;
}

inline bool limitSensorState() {
  const bool currentState = digitalRead(config.limitSensorPin);
  if (currentState != lastLimitSensorState &&
      millis() - lastLimitDebounce > config.debounceMs) {
    lastLimitDebounce = millis();
    lastLimitSensorState = currentState;
  }
  return lastLimitSensorState;
}

inline void setPhase(uint8_t phase) {
  const bool active = (phase != 0);
  digitalWrite(config.relayPin,
               active == (config.relayActiveState == HIGH) ? HIGH : LOW);
}

inline void updatePhaseSwitchSteps() {
  int phaseAngle = config.phaseSwitchAngle;
  if (phaseAngle < 0 || phaseAngle >= 180) {
    DIAG(F("EX-Turntable_OB phase angle %d is invalid; using 45 degrees"), phaseAngle);
    phaseAngle = 45;
  }
  phaseSwitchStartSteps = fullTurnSteps / 360 * phaseAngle;
  phaseSwitchStopSteps = fullTurnSteps / 360 * (phaseAngle + 180);
}

inline void moveHome() {
  setPhase(0);
  if (homeSensorState() == config.homeSensorActiveState) {
    stepper->stop();
    stepper->setCurrentPosition(0);
    lastStep = 0;
    homed = 1;
    DIAG(F("EX-Turntable_OB homed"));
  } else if (!stepper->isRunning()) {
    if (stepper->targetPosition() == lastTarget) {
      stepper->setCurrentPosition(0);
      lastStep = 0;
      homed = 2;
      DIAG(F("EX-Turntable_OB could not find home sensor"));
    } else {
      stepper->enableOutputs();
      stepper->move(config.sanitySteps);
      lastTarget = stepper->targetPosition();
      DIAG(F("EX-Turntable_OB homing started"));
    }
  }
}

inline void moveToPosition(long steps, uint8_t phaseSwitch) {
  if (steps == lastStep) return;

  long moveSteps;
  if (config.mode == TraverserMode) {
    moveSteps = lastStep - steps;
  } else if (config.rotationMode == ForwardRotation) {
    moveSteps = steps - lastStep;
    if (moveSteps < 0) moveSteps += fullTurnSteps;
  } else if (config.rotationMode == ReverseRotation) {
    moveSteps = steps - lastStep;
    if (moveSteps > 0) moveSteps -= fullTurnSteps;
  } else {
    if (steps - lastStep > halfTurnSteps) {
      moveSteps = steps - fullTurnSteps - lastStep;
    } else if (steps - lastStep < -halfTurnSteps) {
      moveSteps = fullTurnSteps - lastStep + steps;
    } else {
      moveSteps = steps - lastStep;
    }
  }

  if (config.phaseSwitchMode == AutomaticPhaseSwitching) {
    phaseSwitch = ((steps < phaseSwitchStartSteps) ||
                   (steps >= phaseSwitchStopSteps && steps <= fullTurnSteps)) ? 0 : 1;
  }
  setPhase(phaseSwitch);
  lastStep = steps;
  stepper->enableOutputs();
  stepper->move(moveSteps);
  lastTarget = stepper->targetPosition();
  DIAG(F("EX-Turntable_OB onboard move: steps=%ld"), moveSteps);
}

inline void setLedActivity(uint8_t activity) {
  ledState = activity;
}

inline void processLed() {
  const unsigned long currentMillis = millis();
  if (ledState == ActivityLedOn) {
    ledOutput = true;
  } else if (ledState == ActivityLedOff) {
    ledOutput = false;
  } else if ((ledState == ActivityLedSlow &&
              currentMillis - ledMillis >= config.ledSlowMs) ||
             (ledState == ActivityLedFast &&
              currentMillis - ledMillis >= config.ledFastMs)) {
    ledOutput = !ledOutput;
    ledMillis = currentMillis;
  }
  digitalWrite(config.ledPin, ledOutput);
}

inline void calibration() {
  setPhase(0);
  if (config.mode == TraverserMode && calibrationPhase == 3 &&
      limitSensorState() != config.limitSensorActiveState) {
    stepper->stop();
    fullTurnSteps = stepper->currentPosition();
    if (fullTurnSteps < 0) fullTurnSteps = -fullTurnSteps;
    halfTurnSteps = fullTurnSteps / 2;
    updatePhaseSwitchSteps();
    calibrating = false;
    calibrationPhase = 0;
    stepper->setCurrentPosition(stepper->currentPosition());
    homed = 0;
    lastTarget = config.sanitySteps;
    config.fullStepCount = fullTurnSteps;
    if (fullTurnSteps <= INT16_MAX) {
      NVSTable::setNVS(EXTurntableOnboard::FullStepCount, (int16_t)fullTurnSteps);
    } else {
      DIAG(F("EX-Turntable_OB calibration exceeds NVS range; configure full steps manually"));
    }
    DIAG(F("EX-Turntable_OB calibration complete: %ld steps"), fullTurnSteps);
  } else if (config.mode == TraverserMode && calibrationPhase == 2 &&
             limitSensorState() == config.limitSensorActiveState) {
    stepper->stop();
    stepper->setCurrentPosition(stepper->currentPosition());
    stepper->moveTo(0);
    lastStep = 0;
    calibrationPhase = 3;
  } else if (calibrationPhase == 1 && lastStep == config.sanitySteps &&
             homeSensorState() == config.homeSensorActiveState &&
             (config.mode == TraverserMode ||
              stepper->currentPosition() > config.homeSensitivity)) {
    stepper->stop();
    stepper->setCurrentPosition(0);
    calibrationPhase = 2;
    stepper->enableOutputs();
    stepper->moveTo(config.mode == TraverserMode ?
                    -config.sanitySteps : config.sanitySteps);
    lastStep = config.sanitySteps;
  } else if (calibrationPhase == 0 && !stepper->isRunning() && homed == 1) {
    calibrationPhase = 1;
    if (config.mode == TraverserMode &&
        homeSensorState() == config.homeSensorActiveState) {
      lastStep = config.sanitySteps;
    } else {
      stepper->enableOutputs();
      stepper->moveTo(config.sanitySteps);
      lastStep = config.sanitySteps;
    }
  } else if (!stepper->isRunning() &&
             ((calibrationPhase == 1 &&
               stepper->currentPosition() == config.sanitySteps) ||
              (config.mode == TraverserMode && calibrationPhase == 2 &&
               stepper->currentPosition() == -config.sanitySteps) ||
              (config.mode == TurntableMode && calibrationPhase == 2 &&
               stepper->currentPosition() == config.sanitySteps))) {
    DIAG(F("EX-Turntable_OB calibration failed: sensor was not reached"));
    calibrating = false;
    calibrationPhase = 0;
  }
}

inline void initiateHoming() {
  homed = 0;
  lastTarget = config.sanitySteps;
}

inline void initiateCalibration() {
  calibrating = true;
  homed = 0;
  calibrationPhase = 0;
  lastTarget = config.sanitySteps;
}

inline void setExtraOutput(uint8_t activity) {
  uint8_t pin = 0;
  bool on;
  switch (activity) {
    case 10: pin = config.extraPins[0]; on = true; break;
    case 11: pin = config.extraPins[0]; on = false; break;
    case 12: pin = config.extraPins[1]; on = true; break;
    case 13: pin = config.extraPins[1]; on = false; break;
    case 14: pin = config.extraPins[2]; on = true; break;
    case 15: pin = config.extraPins[2]; on = false; break;
    case 16: pin = config.extraPins[3]; on = true; break;
    case 17: pin = config.extraPins[3]; on = false; break;
    default: return;
  }
  digitalWrite(pin, on ? HIGH : LOW);
}
}

namespace EXTurntableOnboard {
using namespace EXTurntableOnboardDetail;

inline const Configuration &configuration() {
  return config;
}

inline void begin() {
  if (!configurationLoaded) loadConfiguration();
  pinMode(config.homeSensorPin,
          config.homeSensorActiveState == LOW ? INPUT_PULLUP : INPUT);
  pinMode(config.limitSensorPin,
          config.limitSensorActiveState == LOW ? INPUT_PULLUP : INPUT);
  pinMode(config.relayPin, OUTPUT);
  pinMode(config.ledPin, OUTPUT);
  pinMode(config.accessoryPin, OUTPUT);
  if (config.extraOutputsEnabled) {
    for (uint8_t pin : config.extraPins) pinMode(pin, OUTPUT);
  }

  lastHomeSensorState = digitalRead(config.homeSensorPin);
  lastLimitSensorState = digitalRead(config.limitSensorPin);
  setPhase(0);
  digitalWrite(config.accessoryPin, LOW);
  digitalWrite(config.ledPin, LOW);

  if (fullTurnSteps > 0) {
    updatePhaseSwitchSteps();
  } else {
    DIAG(F("EX-Turntable_OB: fullTurnSteps=%ld"), fullTurnSteps);
    calibrating = true;
  }
}

inline void service() {
  if (config.mode == TraverserMode &&
      limitSensorState() == config.limitSensorActiveState &&
      !calibrating && stepper->isRunning() &&
      stepper->targetPosition() < 0) {
    stepper->stop();
    stepper->setCurrentPosition(stepper->currentPosition());
    if (!homed) homed = 1;
  }
  if (config.mode == TraverserMode &&
      homeSensorState() == config.homeSensorActiveState &&
      homed && !calibrating && stepper->isRunning() &&
      stepper->distanceToGo() > 0) {
    stepper->stop();
    stepper->setCurrentPosition(0);
  }
  if (homed == 0) moveHome();
  if (calibrating) calibration();
  const bool running = stepper->run();
  processLed();
  if (!config.keepOutputsEnabled && running != lastRunningState) {
    lastRunningState = running;
    if (!running) stepper->disableOutputs();
  }
  lastRunningState = running;
}

inline bool isBusy() {
  return stepper && (stepper->isRunning() || calibrating || homed == 0);
}

inline bool command(long steps, uint8_t activity) {
  if (activity <= ActivityTurnPhaseInvert) {
    const long gearedSteps = steps * gearingFactor;
    if (isBusy() || fullTurnSteps <= 0 || gearedSteps > fullTurnSteps) {
      DIAG(F("EX-Turntable_OB rejected move: steps=%ld fullTurnSteps=%ld busy=%d"),
           gearedSteps, fullTurnSteps, isBusy());
      return false;
    }
    moveToPosition(gearedSteps, activity);
    return true;
  }

  if (activity == ActivityHome) {
    if (stepper->isRunning() || (calibrating && homed != 2)) return false;
    initiateHoming();
    return true;
  }
  if (activity == ActivityCalibrate) {
    if (stepper->isRunning() || (calibrating && homed != 2)) return false;
    initiateCalibration();
    return true;
  }
  if (activity >= ActivityLedOn && activity <= ActivityLedOff) {
    setLedActivity(activity);
    return true;
  }
  if (activity == ActivityAccessoryOn || activity == ActivityAccessoryOff) {
    digitalWrite(config.accessoryPin,
                 activity == ActivityAccessoryOn ? HIGH : LOW);
    return true;
  }
  if (config.extraOutputsEnabled && activity >= 10 && activity <= 17) {
    setExtraOutput(activity);
    return true;
  }
  DIAG(F("EX-Turntable_OB rejected unsupported activity %u"), activity);
  return false;
}
}

//inline void EXTurntable_OB::create(VPIN firstVpin, int nPins, I2CAddress i2cAddress) {
inline void EXTurntable_OB::create(VPIN firstVpin, int nPins) {
  const int16_t configuredVpin = NVSTable::getNVS(0, true);
  if (configuredVpin > 0) firstVpin = (VPIN)configuredVpin;
//  new EXTurntable_OB(firstVpin, nPins, i2cAddress);
  new EXTurntable_OB(firstVpin, nPins);
}

//inline EXTurntable_OB::EXTurntable_OB(VPIN firstVpin, int nPins, I2CAddress i2cAddress) {
inline EXTurntable_OB::EXTurntable_OB(VPIN firstVpin, int nPins) {
  _firstVpin = firstVpin;
  _nPins = nPins;
//  (void)i2cAddress;
  _stepperStatus = 0;
  _previousStatus = 0;

  const int16_t nvsFullStepCount =
      NVSTable::getNVS(EXTurntableOnboard::FullStepCount, true);
  _currentActivity = (nvsFullStepCount != 0) ? Home : Calibrate;

  addDevice(this);
}

inline void EXTurntable_OB::_begin() {
  EXTurntableOnboard::begin();
  _stepperStatus = EXTurntableOnboard::isBusy();
  _previousStatus = _stepperStatus;
#ifdef DIAG_IO
  _display();
#endif
}

inline void EXTurntable_OB::_loop(unsigned long currentMicros) {
  (void)currentMicros;
  EXTurntableOnboard::service();
  _stepperStatus = EXTurntableOnboard::isBusy();
  if (_stepperStatus != _previousStatus) {
    if (_stepperStatus == 0 && _currentActivity < 4) {
      _broadcastStatus(_firstVpin, _stepperStatus, _currentActivity);
    }
    _previousStatus = _stepperStatus;
  }
}

inline int EXTurntable_OB::_read(VPIN vpin) {
  (void)vpin;
  return _stepperStatus;
}

inline void EXTurntable_OB::_broadcastStatus(VPIN vpin, uint8_t status, uint8_t activity) {
  Turntable *turntable = Turntable::getByVpin(vpin);
  if (turntable && activity < 4) {
    turntable->setMoving(status);
    CommandDistributor::broadcastTurntable(
        turntable->getId(), turntable->getPosition(), status);
  }
}

inline void EXTurntable_OB::_writeAnalogue(
    VPIN vpin, int value, uint8_t activity, uint16_t duration) {
#ifdef DIAG_IO
  DIAG(F("EX-Turntable_OB onboard VPIN:%u Value:%d Activity:%d Duration:%d"),
       vpin, value, activity, duration);
#else
  (void)duration;
#endif
  if (value < 0 || !EXTurntableOnboard::command(value, activity)) return;

  _stepperStatus = EXTurntableOnboard::isBusy();
  _previousStatus = _stepperStatus;
  if (_stepperStatus && activity < 4) {
    _currentActivity = activity;
    _broadcastStatus(vpin, _stepperStatus, activity);
  }
}

inline void EXTurntable_OB::_display() {
  DIAG(F("EX-Turntable_OB driver configured on Vpins:%u-%u"),
       (int)_firstVpin, (int)_firstVpin + _nPins - 1);
}

#endif
