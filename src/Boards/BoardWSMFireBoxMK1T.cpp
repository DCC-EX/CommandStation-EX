/*
 *  BoardWSMFireBoxMK1T.cpp
 * 
 *  This file is part of CommandStation-EX.
 *
 *  CommandStation-EX is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  CommandStation-EX is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with CommandStation-EX.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "BoardWSMFireBoxMK1T.h"

#if defined(ARDUINO_ARCH_SAMC)

void BoardWSMFireBoxMK1T::setup() {
  pinMode(config.enable_pin, OUTPUT);
  writePin(config.enable_pin, LOW);

  pinMode(config.signal_a_pin, OUTPUT);
  writePin(config.signal_a_pin, LOW);

  pinMode(config.signal_b_pin, OUTPUT);
  writePin(config.signal_b_pin, LOW);

  pinMode(config.sense_pin, INPUT);

  pinMode(config.limit_pin, OUTPUT);
  writePin(config.limit_pin, HIGH);

  pinMode(config.cutout_pin, OUTPUT);
  writePin(config.cutout_pin, HIGH);

  tripped = false;
  inCutout = false;

  config.serial->begin(config.railcom_baud);
}

const char * BoardWSMFireBoxMK1T::getName() {
  return config.track_name;
}

void BoardWSMFireBoxMK1T::progMode(bool on) {
  inProgMode = on; // Not equipped with active current limiting, so just enable the programming mode variable
  writePin(config.limit_pin, on);
}

void BoardWSMFireBoxMK1T::power(bool on, bool announce) {
  if(inProgMode) {
    progOverloadTimer = millis();
  }
  
  writePin(config.enable_pin, on);

  if(announce) {
    config.track_power_callback(config.track_name, on);
  }
}

void BoardWSMFireBoxMK1T::signal(bool dir) {
  if(!inCutout) {
    writePin(config.signal_a_pin, dir);
    writePin(config.signal_b_pin, !dir);
  }
}

uint16_t BoardWSMFireBoxMK1T::getCurrentRaw() {
  return analogReadFast(config.sense_pin);
}

uint16_t BoardWSMFireBoxMK1T::getCurrentMilliamps() {
  uint16_t currentMilliamps;
  currentMilliamps = getCurrentMilliamps(getCurrentRaw());
  return currentMilliamps;
}

uint16_t BoardWSMFireBoxMK1T::getCurrentMilliamps(uint16_t reading) {
  uint16_t currentMilliamps;
  currentMilliamps = reading / 1023.0 * config.board_voltage * 1000 * config.amps_per_volt;
  return currentMilliamps;
}

bool BoardWSMFireBoxMK1T::getStatus() {
  return digitalRead(config.enable_pin);
}

void BoardWSMFireBoxMK1T::checkOverload() {
  if(millis() - progOverloadTimer > config.prog_trip_time) config.prog_trip_time = 0; // Protect against wrapping 

  if(millis() - lastCheckTime > kCurrentSampleTime) {
    lastCheckTime = millis();
    reading = getCurrentRaw() * kCurrentSampleSmoothing + reading * (1.0 - kCurrentSampleSmoothing);
    uint16_t current = getCurrentMilliamps(reading);

    uint16_t current_trip = config.current_trip;
    if(isCurrentLimiting()) current_trip = config.current_trip_prog;

    if(current > current_trip && getStatus()) {
      power(OFF, true);
      tripped=true;
      lastTripTime=millis();
    } 
    else if(current < current_trip && tripped) {
      if (millis() - lastTripTime > kRetryTime) {
        power(ON, true);
        tripped=false;
      }
    }
  }
}

bool BoardWSMFireBoxMK1T::isCurrentLimiting() {
  // If we're in programming mode and it's been less than prog_trip_time since we turned the power on... or if the timeout is set to zero.
  if(inProgMode && ((millis() - progOverloadTimer < config.prog_trip_time) || config.prog_trip_time == 0))
    return true;

  return false;
}

uint16_t BoardWSMFireBoxMK1T::setCurrentBase() {
  currentBase = getCurrentMilliamps();
  return currentBase;
}

uint16_t BoardWSMFireBoxMK1T::getCurrentBase() {
  return currentBase;
}

uint8_t BoardWSMFireBoxMK1T::getPreambles() {
  if(inProgMode) return config.prog_preambles;
  return config.main_preambles;
}

void BoardWSMFireBoxMK1T::rcomCutout(bool on) {
  inCutout = on;  // Protect railcom against track power

  if(on) {
    writePin(config.signal_a_pin, OFF);
    writePin(config.signal_b_pin, OFF);
  }

  writePin(config.cutout_pin, !on);
}

void BoardWSMFireBoxMK1T::rcomEnable(bool on) {
  if(on)  // Flush the serial port RX buffer
    while(config.serial->available())
      config.serial->read();
}

void BoardWSMFireBoxMK1T::rcomRead() {
  // Check that the buffer is empty
  for(int i = 0; i < kRcomBufferSize; i++) {
    if(rcomBuffer[i] != 0x00) return;
  }

  // Determine how many bytes are available for read
  uint8_t bytes = config.serial->available();
  if(bytes > 8) bytes = 8;

  // Read available bytes into the RailCom buffer
  config.serial->readBytes(rcomBuffer, bytes);
}

#endif  // ARDUINO_ARCH_SAMC
