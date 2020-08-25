#include "BoardArduinoMotorShield.h"

void BoardArduinoMotorShield::setup() {
  pinMode(config.enable_pin, OUTPUT);
  writePin(config.enable_pin, LOW);

  pinMode(config.signal_a_pin, OUTPUT);
  writePin(config.signal_a_pin, LOW);

  pinMode(config.signal_b_pin, OUTPUT);
  writePin(config.signal_b_pin, LOW);

  pinMode(config.sense_pin, INPUT);

  tripped = false;
}

const char * BoardArduinoMotorShield::getName() {
  return config.track_name;
}

void BoardArduinoMotorShield::progMode(bool on) {
  inProgMode = on; // Not equipped with active current limiting, so just enable the programming mode variable
}

void BoardArduinoMotorShield::power(bool on, bool announce) {
  if(inProgMode) {
    progOverloadTimer = millis();
  }
  
  writePin(config.enable_pin, on);

  if(announce) {
    config.track_power_callback(config.track_name, on);
  }
}

void BoardArduinoMotorShield::signal(bool dir) {
  writePin(config.signal_a_pin, dir);
}

void BoardArduinoMotorShield::cutout(bool on) {
  writePin(config.signal_b_pin, on);
}

uint16_t BoardArduinoMotorShield::getCurrentRaw() {
  return analogReadFast(config.sense_pin);
}

uint16_t BoardArduinoMotorShield::getCurrentMilliamps() {
  uint16_t currentMilliamps;
  currentMilliamps = getCurrentMilliamps(getCurrentRaw());
  return currentMilliamps;
}

uint16_t BoardArduinoMotorShield::getCurrentMilliamps(uint16_t reading) {
  uint16_t currentMilliamps;
  currentMilliamps = reading / 1023.0 * config.board_voltage * 1000 * config.amps_per_volt;
  return currentMilliamps;
}

bool BoardArduinoMotorShield::getStatus() {
  return digitalRead(config.enable_pin);
}

void BoardArduinoMotorShield::checkOverload() {
  if(millis() - progOverloadTimer > config.prog_trip_time) config.prog_trip_time = 0; // Protect against wrapping 

  if(millis() - lastCheckTime > kCurrentSampleTime) {
    lastCheckTime = millis();
    reading = getCurrentRaw() * kCurrentSampleSmoothing + reading * (1.0 - kCurrentSampleSmoothing);
    uint16_t current = getCurrentMilliamps(reading);

    uint16_t current_trip = config.current_trip;
    if(isCurrentLimiting()) current_trip = 250;

    if(current > current_trip && getStatus()) {
      power(OFF, true); // TODO: Add track power notice callback
      tripped=true;
      lastTripTime=millis();
    } 
    else if(current < current_trip && tripped) {
      if (millis() - lastTripTime > kRetryTime) {
        // TODO: Add track power notice callback
        power(ON, true);
        tripped=false;
      }
    }
  }
}

bool BoardArduinoMotorShield::isCurrentLimiting() {
  // If we're in programming mode and it's been less than prog_trip_time since we turned the power on... or if the timeout is set to zero.
  if(inProgMode && ((millis() - progOverloadTimer < config.prog_trip_time) || config.prog_trip_time == 0))
    return true;

  return false;
}

uint16_t BoardArduinoMotorShield::setCurrentBase() {
  currentBase = getCurrentMilliamps();
  return currentBase;
}

uint16_t BoardArduinoMotorShield::getCurrentBase() {
  return currentBase;
}

uint8_t BoardArduinoMotorShield::getPreambles() {
  if(inProgMode) return config.prog_preambles;
  return config.main_preambles;
}

