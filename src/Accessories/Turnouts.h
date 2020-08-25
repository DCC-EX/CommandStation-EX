/*
 *  Turnouts.h
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

#ifndef COMMANDSTATION_ACCESSORIES_TURNOUTS_H_
#define COMMANDSTATION_ACCESSORIES_TURNOUTS_H_

#include <Arduino.h>
#include "../DCC/DCCMain.h"

struct TurnoutData {
  uint8_t tStatus;
  uint8_t subAddress;
  int id;
  int address;  
};

struct Turnout{
  static Turnout *firstTurnout;
  int num;
  struct TurnoutData data;
  Turnout *nextTurnout;
  void activate(Print* stream, int s, DCCMain* track);
  static Turnout* get(int);
  static void remove(Print* stream, int);
  static void load(Print* stream);
  static void store();
  static Turnout *create(Print* stream, int, int, int, int=0);
  static void show(Print* stream, int=0);
};
  
#endif  // COMMANDSTATION_ACCESSORIES_TURNOUTS_H_