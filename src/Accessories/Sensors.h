/*
 *  Sensors.h
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

#ifndef COMMANDSTATION_ACCESSORIES_SENSORS_H_
#define COMMANDSTATION_ACCESSORIES_SENSORS_H_

#include "Arduino.h"

#define  SENSOR_DECAY  0.03

struct SensorData {
  int snum;
  uint8_t pin;
  uint8_t pullUp;
};

struct Sensor{
  static Sensor *firstSensor;
  SensorData data;
  boolean active;
  float signal;
  Sensor *nextSensor;
  static void load(Print* stream);
  static void store();
  static Sensor *create(Print* stream, int, int, int, int=0);
  static Sensor* get(int);  
  static void remove(Print* stream, int);  
  static void show(Print* stream);
  static void status(Print* stream);
  static void parse(const char *c);
  static void check(Print* stream);   
};

#endif  // COMMANDSTATION_ACCESSORIES_SENSORS_H_

