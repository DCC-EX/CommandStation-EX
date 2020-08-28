/*
 *  Outputs.h
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

#ifndef ACCESSORIES_OUTPUTS_H_
#define ACCESSORIES_OUTPUTS_H_

#include <Arduino.h>

struct OutputData {
  uint8_t oStatus;
  uint8_t id;
  uint8_t pin; 
  uint8_t iFlag; 
};

struct Output {
  static Output *firstOutput;
  int num;
  struct OutputData data;
  Output *nextOutput;
  void activate(Print* stream, int s);
  static void parse(Print* stream, const char *c);
  static Output* get(int);
  static void remove(Print* stream, int);
  static void load(Print* stream);
  static void store();
  static Output *create(Print* stream, int, int, int, int=0);
  static void show(Print* stream, int=0);
};
  
#endif  // ACCESSORIES_OUTPUTS_H_