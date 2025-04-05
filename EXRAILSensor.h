/*
 *  © 2024 Chris Harlow
 *  All rights reserved.
 *
 *  This file is part of CommandStation-EX
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

#ifndef EXRAILSensor_h
#define EXRAILSensor_h
#include "IODevice.h"
class EXRAILSensor {
  static EXRAILSensor* firstSensor;
  static EXRAILSensor* readingSensor;
  static unsigned long lastReadCycle;

 public:
  static void checkAll();

  EXRAILSensor(VPIN _pin, int _progCounter, bool _onChange);
  bool check();

 private:
  static const unsigned int cycleInterval = 10000;  // min time between consecutive reads of each sensor in microsecs.
                                                    // should not be less than device scan cycle time.
  static const byte minReadCount = 4;               // number of additional scans before acting on change
                                                    // E.g. 1 means that a change is ignored for one scan and actioned on the next.
                                                    // Max value is 63

  EXRAILSensor* nextSensor;
  VPIN pin;
  int progCounter;
  bool active;
  bool inputState;
  bool onChange;
  byte latchDelay;
};
#endif
