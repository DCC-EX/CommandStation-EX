/*
 *  Board.h
 * 
 *  This file is part of CommandStation.
 *
 *  CommandStation is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  CommandStation is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with CommandStation.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef COMMANDSTATION_BOARDS_BOARD_H_
#define COMMANDSTATION_BOARDS_BOARD_H_

#include <Arduino.h>
#include "AnalogReadFast.h"

#if defined(ARDUINO_ARCH_SAMD) || defined(ARDUINO_ARCH_SAMC)
#define writePin digitalWrite
#elif defined(ARDUINO_ARCH_AVR)
// Library DIO2.h is only compatible with AVR.
#include <DIO2.h>
#define writePin digitalWrite2
#endif

#ifndef A0
#define A0 -1
#endif
#ifndef A1
#define A1 -1
#endif

#define ON  true
#define OFF false

// Time between current samples (millis)
const int kCurrentSampleTime = 1;

// Smoothing constant for exponential current smoothing algorithm
const float kCurrentSampleSmoothing = 0.01;

// Number of milliseconds between retries when the "breaker" is tripped.
const int kRetryTime = 10000;

struct BoardConfig
{
  const char* track_name;
  uint8_t signal_a_pin;
  uint8_t signal_b_pin;
  uint8_t enable_pin;
  uint8_t sense_pin;  
  float board_voltage;
  float amps_per_volt;
  uint16_t current_trip;
  uint16_t current_trip_prog;
  uint16_t prog_trip_time;
  uint8_t main_preambles;
  uint8_t prog_preambles;
  void (*track_power_callback)(const char* name, bool status);
};


class Board
{
public:
  virtual const char* getName() = 0;

  virtual void power(bool, bool announce) = 0;
  virtual void signal(bool) = 0;
  // True to enter a railcom cutout, false to recover
  virtual void cutout(bool) = 0;
  // True to enter prog mode and limit current
  virtual void progMode(bool) = 0;

  // Returns current reading 0-1024
  virtual uint16_t getCurrentRaw() = 0;   
  // Returns current reading in mA
  virtual uint16_t getCurrentMilliamps() = 0;
  virtual uint16_t getCurrentMilliamps(uint16_t reading) = 0;

  virtual uint16_t setCurrentBase() = 0;
  virtual uint16_t getCurrentBase() = 0;

  virtual bool getStatus() = 0;

  virtual void checkOverload() = 0;

  virtual uint8_t getPreambles() = 0;
protected:
  // Current reading variables
  uint16_t reading;
  bool tripped;
  long int lastCheckTime;
  long int lastTripTime;

  // Programming mode
  bool inProgMode; 
  uint16_t progOverloadTimer;
  uint16_t currentBase;

  virtual bool isCurrentLimiting() = 0;
};

#endif  // COMMANDSTATION_BOARDS_BOARD_H_