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
#ifndef Outputs_h
#define Outputs_h

#include <Arduino.h>

struct OutputData {
  uint8_t oStatus;
  uint8_t id;
  uint8_t pin; 
  uint8_t iFlag; 
};

class Output{
  public:
  void activate(int s);
  static Output* get(int);
  static bool remove(int);
  static void load();
  static void store();
  static Output *create(int, int, int, int=0);
  static void show(Print * stream);
  static bool showAll(Print * stream);

  private:
  static Output *firstOutput;
  int num;
  struct OutputData data;
  Output *nextOutput;

}; // Output
  
#endif
