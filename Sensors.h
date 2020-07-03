/*
 *  © 2020, Chris Harlow. All rights reserved.
 *  
 *  This file is part of Asbelos DCC API
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
#ifndef Sensor_h
#define Sensor_h

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
  static void load();
  static void store();
  static Sensor *create(int, int, int);
  static Sensor* get(int);  
  static bool remove(int);  
  static void show(Print & stream);
  static void status(Print & stream);
  static void check(Print & stream);   
}; // Sensor

#endif
