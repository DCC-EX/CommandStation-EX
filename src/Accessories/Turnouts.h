/*
 *  Turnouts.h
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

#ifndef ACCESSORIES_TURNOUTS_H_
#define ACCESSORIES_TURNOUTS_H_

#include <Arduino.h>
#include "../DCC/DCC.h"

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
  void activate(Print* stream, int s, DCC* track);
  static Turnout* get(int);
  static void remove(Print* stream, int);
  static void load(Print* stream);
  static void store();
  static Turnout *create(Print* stream, int, int, int, int=0);
  static void show(Print* stream, int=0);
  static int turnoutlistHash;
};
  
#endif  // ACCESSORIES_TURNOUTS_H_